// 只读订阅 Studio WebSocket。不会打开机械臂串口，也不发送 Enable/Move/ResetZero。
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <json/json.h>

#include "joint_mapping.hpp"
#include "kinematics.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;
using Clock = std::chrono::steady_clock;

namespace {
struct Options {
    std::string host = "127.0.0.1";
    std::string port = "8080";
    int count = 0;              // 0 表示持续读取，Ctrl+C 退出。
    int interval_ms = 500;      // 只限制输出频率，仍持续接收消息，避免积压。
    int timeout_ms = 5000;      // 超时期间没有有效 StatusUpdate 就报错退出。
    std::string csv_path = "test_data/studio_pose_ik.csv";
    std::string seed_mode = "current";
    bool capture_only = false;
    std::string events_path;
};

void print_usage() {
    std::cout << "用法: read_studio_pose [选项]\n"
              << "  --host HOST           Studio 地址，默认 127.0.0.1\n"
              << "  --port PORT           Studio 端口，默认 8080\n"
              << "  --count N             输出 N 组后退出，0 表示持续读取\n"
              << "  --interval-ms MS      输出间隔，默认 500；0 表示每帧输出\n"
              << "  --timeout-ms MS       无有效状态超时，默认 5000\n"
              << "  --csv PATH            CSV 输出路径，默认 test_data/studio_pose_ik.csv\n"
              << "  --seed-mode MODE      current=当前反馈(默认)，previous=上一帧成功IK解\n"
              << "  --capture-only        只记录反馈，不运行本地 IK\n"
              << "  --events-jsonl PATH   保存每条 WebSocket 原文及本机接收时间（不受输出限频影响）\n"
              << "  --help                显示帮助\n";
}

int parse_integer(const std::string& value) {
    std::size_t used = 0;
    const int result = std::stoi(value, &used);
    if (used != value.size() || result < 0) throw std::invalid_argument("无效的非负整数: " + value);
    return result;
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--capture-only") { options.capture_only = true; continue; }
        if (i + 1 == argc) throw std::invalid_argument("选项缺少参数: " + arg);
        const std::string value = argv[++i];
        if (arg == "--host") options.host = value;
        else if (arg == "--port") {
            const int port = parse_integer(value);
            if (port < 1 || port > 65535) throw std::invalid_argument("端口必须为 1～65535");
            options.port = value;
        } else if (arg == "--count") options.count = parse_integer(value);
        else if (arg == "--interval-ms") options.interval_ms = parse_integer(value);
        else if (arg == "--timeout-ms") options.timeout_ms = parse_integer(value);
        else if (arg == "--csv") options.csv_path = value;
        else if (arg == "--events-jsonl") options.events_path = value;
        else if (arg == "--seed-mode") {
            if (value != "current" && value != "previous") {
                throw std::invalid_argument("--seed-mode 必须为 current 或 previous");
            }
            options.seed_mode = value;
        }
        else throw std::invalid_argument("未知选项: " + arg);
    }
    if (options.host.empty() || options.timeout_ms == 0 || options.csv_path.empty()) {
        throw std::invalid_argument("地址和 CSV 路径不能为空，超时须大于 0");
    }
    if (!options.events_path.empty()
        && std::filesystem::absolute(options.events_path).lexically_normal()
           == std::filesystem::absolute(options.csv_path).lexically_normal()) {
        throw std::invalid_argument("CSV 和原始消息必须使用不同文件");
    }
    return options;
}

std::string csv_escape(const std::string& value) {
    std::string escaped = "\"";
    for (const char character : value) {
        if (character == '"') escaped += '"';
        escaped += character;
    }
    escaped += '"';
    return escaped;
}

std::string utc_timestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    );
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(milliseconds);
    const auto fraction_ms = (milliseconds - seconds).count();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
    if (::gmtime_r(&time, &utc) == nullptr) {
        throw std::runtime_error("无法生成 CSV UTC 时间戳");
    }

    std::ostringstream output;
    output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S")
           << '.' << std::setw(3) << std::setfill('0') << fraction_ms << 'Z';
    return output.str();
}

struct Sample {
    std::array<double, 3> xyz_m{};
    std::array<double, 3> rpy_deg{};
    std::optional<joint_mapping::JointAngles> device_angles;
};

bool finite_number(const Json::Value& value) {
    return value.isNumeric() && std::isfinite(value.asDouble());
}

std::optional<Sample> parse_sample(const std::string& message) {
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    builder["allowComments"] = false;
    builder["failIfExtra"] = true;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value root;
    std::string error;
    if (!reader->parse(message.data(), message.data() + message.size(), &root, &error) || !root.isObject()) {
        throw std::runtime_error("Studio 返回了无效 JSON: " + error);
    }
    if (!root["event"].isString() || root["event"].asString() != "StatusUpdate") return std::nullopt;
    const auto& payload = root["payload"];
    if (!payload.isObject() || !payload["connected"].isBool() || !payload["connected"].asBool()) {
        return std::nullopt;
    }
    const auto& state = payload["jointState"];
    if (!state.isObject()) return std::nullopt;
    const auto& pose = state["cartesianPosition"];
    if (!pose.isObject()) return std::nullopt;

    Sample sample;
    constexpr std::array<const char*, 6> fields{"x", "y", "z", "roll", "pitch", "yaw"};
    for (std::size_t i = 0; i < fields.size(); ++i) {
        if (!finite_number(pose[fields[i]])) return std::nullopt;
        const double value = pose[fields[i]].asDouble();
        if (i < 3) sample.xyz_m[i] = value;
        else sample.rpy_deg[i - 3] = value;
    }

    // Studio 的当前关节反馈作为 IK 连续选解的种子。部分状态消息可能没有此字段；
    // 此时仍输出 XYZ/RPY，但不尝试 IK。
    const auto& joint_angles = state["jointAngle"];
    if (joint_angles.isArray() && joint_angles.size() == 6) {
        joint_mapping::JointAngles parsed_angles{};
        bool valid_angles = true;
        for (Json::ArrayIndex i = 0; i < parsed_angles.size(); ++i) {
            if (!finite_number(joint_angles[i])) {
                valid_angles = false;
                break;
            }
            parsed_angles[i] = joint_angles[i].asDouble();
        }
        if (valid_angles) sample.device_angles = parsed_angles;
    }
    return sample;
}

template<std::size_t N>
void print_vector(const std::array<double, N>& values) {
    std::cout << '[';
    for (std::size_t i = 0; i < N; ++i) std::cout << (i ? ", " : "") << values[i];
    std::cout << ']';
}

void print_joint_row(
    const char* label,
    const joint_mapping::JointAngles& angles
) {
    std::cout << "  " << std::left << std::setw(34) << label << std::right;
    for (const double angle : angles) {
        std::cout << std::setw(12) << angle;
    }
    std::cout << '\n';
}

void print_joint_header() {
    std::cout << "  " << std::left << std::setw(34) << "数据来源 / 角度定义" << std::right;
    for (int joint = 1; joint <= 6; ++joint) {
        std::cout << std::setw(12) << ("J" + std::to_string(joint));
    }
    std::cout << '\n';
}

class StudioReader {
public:
    explicit StudioReader(const Options& options)
        : options_(options), resolver_(io_), ws_(io_), deadline_(io_), signals_(io_, SIGINT, SIGTERM) {
        if (!options_.events_path.empty()) {
            const std::filesystem::path path(options_.events_path);
            if (std::filesystem::exists(path)) throw std::runtime_error("原始消息文件已存在，请使用新文件名");
            if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
            events_.open(path);
            if (!events_) throw std::runtime_error("无法创建原始消息文件: " + options_.events_path);
        }
        open_csv();
    }

    int run() {
        signals_.async_wait([this](beast::error_code ec, int) { if (!ec) stop(); });
        arm_deadline();
        ws_.read_message_max(1024 * 1024);
        const std::string host = options_.host.find(':') == std::string::npos
            ? options_.host : '[' + options_.host + ']';
        std::cerr << "只读连接 ws://" << host << ':' << options_.port << "/api/ws\n";
        resolver_.async_resolve(options_.host, options_.port,
            [this, host](beast::error_code ec, tcp::resolver::results_type results) {
                if (check_error(ec, "解析地址")) return;
                beast::get_lowest_layer(ws_).async_connect(results,
                    [this, host](beast::error_code error, const tcp::endpoint&) {
                        if (check_error(error, "连接 Studio")) return;
                        ws_.async_handshake(host + ':' + options_.port, "/api/ws",
                            [this](beast::error_code e) {
                                if (!check_error(e, "WebSocket 握手")) read_next(); // IK入口,里面用了IK的output()
                            });
                    });
            });
        io_.run();
        return exit_code_;
    }

private:
    bool check_error(beast::error_code ec, const char* operation) {
        if (stopped_) return true;
        if (ec) { stop(std::string(operation) + ": " + ec.message()); return true; }
        return false;
    }

    void stop(const std::string& error = {}) {
        if (stopped_) return;
        stopped_ = true;
        if (!error.empty()) { std::cerr << "读取失败: " << error << '\n'; exit_code_ = 1; }
        if (csv_.is_open()) csv_.flush();
        if (events_.is_open()) events_.flush();
        resolver_.cancel();
        beast::error_code ignored;
        deadline_.cancel(ignored);
        signals_.cancel(ignored);
        // 关闭本客户端连接；不向机械臂发送 Stop 或其他控制命令。
        beast::get_lowest_layer(ws_).socket().close(ignored);
    }

    void arm_deadline() {
        deadline_.expires_after(std::chrono::milliseconds(options_.timeout_ms));
        deadline_.async_wait([this](beast::error_code ec) {
            if (!ec) stop("超时未收到有效 StatusUpdate，请检查 Studio 是否已连接机械臂");
        });
    }

    void read_next() {
        ws_.async_read(buffer_, [this](beast::error_code ec, std::size_t) {
            if (check_error(ec, "接收状态")) return;
            try {
                if (!ws_.got_text()) throw std::runtime_error("收到非文本 WebSocket 消息");
                const std::string raw = beast::buffers_to_string(buffer_.data());
                if (events_.is_open()) {
                    Json::Value record;
                    record["received_utc"] = utc_timestamp();
                    record["message_text"] = raw;
                    Json::StreamWriterBuilder builder;
                    builder["indentation"] = "";
                    events_ << Json::writeString(builder, record) << '\n';
                    events_.flush();
                    if (!events_) throw std::runtime_error("原始 WebSocket 消息写入失败");
                }
                const auto sample = parse_sample(raw);
                buffer_.consume(buffer_.size());
                if (sample) {
                    arm_deadline();
                    const auto now = Clock::now();
                    if (printed_ == 0 || now - last_print_ >= std::chrono::milliseconds(options_.interval_ms)) {
                        output(*sample); // IK
                        last_print_ = now;
                        ++printed_;
                        if (options_.count > 0 && printed_ >= options_.count) { stop(); return; }
                    }
                }
            } catch (const std::exception& error) { stop(error.what()); return; }
            read_next();
        });
    }

    void output(const Sample& sample) {
        std::cout << std::fixed << std::setprecision(3)
                  << "\n========== 样本 #" << (printed_ + 1) << " ==========\n"
                  << "[Studio读取] 当前笛卡尔反馈位姿\n"
                  << "  XYZ (mm) = ["
                  << sample.xyz_m[0] * 1000.0 << ", "
                  << sample.xyz_m[1] * 1000.0 << ", "
                  << sample.xyz_m[2] * 1000.0 << "]\n"
                  << "  RPY (deg)= ";
        print_vector(sample.rpy_deg);
        std::cout << '\n';

        if (options_.capture_only) {
            std::optional<joint_mapping::JointAngles> model;
            if (sample.device_angles) {
                model = joint_mapping::device_to_model(*sample.device_angles);
                print_joint_row("Studio读取 / 设备反馈角 m", *sample.device_angles);
            }
            write_csv_row(sample, model, "not_run_capture_only", std::nullopt, std::nullopt,
                          std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
            std::cout << "  只读采集，未运行本地 IK\n========================================" << std::endl;
            return;
        }

        const kinematics::Pose target_pose{
            sample.xyz_m[0],
            sample.xyz_m[1],
            sample.xyz_m[2],
            kinematics::quaternion_from_studio_rpy(
                sample.rpy_deg[0],
                sample.rpy_deg[1],
                sample.rpy_deg[2]
            )
        };

        std::optional<joint_mapping::JointAngles> current_model_angles;
        if (sample.device_angles) {
            current_model_angles =
                joint_mapping::device_to_model(*sample.device_angles);
        }

        // current 验证反馈角还原；previous 验证历史分支跟随，首次以当前反馈初始化。
        const bool use_previous = options_.seed_mode == "previous" && previous_ik_solution_;
        const auto seed_model_angles = use_previous ? previous_ik_solution_ : current_model_angles;
        const std::string seed_source = use_previous
            ? "previous_ik_solution"
            : (current_model_angles ? "current_device_feedback" : "unavailable_current_feedback");
        std::optional<kinematics::JointAngles> solution_model_angles;
        if (seed_model_angles) {
            solution_model_angles =
                kinematics::inverse_kinematics(target_pose, *seed_model_angles);
        }

        std::optional<joint_mapping::JointAngles> solution_device_angles;
        std::optional<joint_mapping::JointAngles> device_angle_difference;
        std::optional<kinematics::Pose> achieved_pose;
        std::optional<double> position_error_mm;
        std::optional<double> orientation_error_deg;

        if (solution_model_angles) {
            solution_device_angles =
                joint_mapping::model_to_device(*solution_model_angles);
            achieved_pose =
                kinematics::forward_kinematics(*solution_model_angles);

            const double dx = achieved_pose->x_m - target_pose.x_m;
            const double dy = achieved_pose->y_m - target_pose.y_m;
            const double dz = achieved_pose->z_m - target_pose.z_m;
            position_error_mm = std::hypot(std::hypot(dx, dy), dz) * 1000.0;

            const auto& target_q = target_pose.orientation;
            const auto& achieved_q = achieved_pose->orientation;
            const double quaternion_dot = std::clamp(
                std::abs(target_q.w * achieved_q.w
                         + target_q.x * achieved_q.x
                         + target_q.y * achieved_q.y
                         + target_q.z * achieved_q.z),
                0.0,
                1.0
            );
            orientation_error_deg =
                2.0 * std::acos(quaternion_dot)
                * 180.0 / 3.14159265358979323846;

            if (sample.device_angles) {
                device_angle_difference.emplace();
                for (std::size_t i = 0; i < device_angle_difference->size(); ++i) {
                    (*device_angle_difference)[i] =
                        (*solution_device_angles)[i] - (*sample.device_angles)[i];
                }
            }
        }

        write_csv_row(
            sample,
            current_model_angles,
            seed_source,
            seed_model_angles,
            solution_model_angles,
            solution_device_angles,
            device_angle_difference,
            achieved_pose,
            position_error_mm,
            orientation_error_deg
        );

        std::cout << "\n[关节角对比] 单位均为度；m=设备角，q=模型角\n";
        print_joint_header();
        if (sample.device_angles && current_model_angles) {
            print_joint_row("Studio读取 / 设备反馈角 m", *sample.device_angles);
            print_joint_row("映射计算 / 当前模型角 q", *current_model_angles);
        } else {
            std::cout << "  Studio jointAngle 缺失或无效，当前关节角无法对照\n";
        }

        std::cout << "\n[IK计算] 模式: " << options_.seed_mode << "，选支参考: "
                  << (use_previous ? "上一帧成功的IK模型角"
                      : (current_model_angles ? "当前帧设备反馈映射后的模型角"
                                              : "不可用，当前关节反馈缺失或无效"))
                  << '\n';

        if (!seed_model_angles) {
            std::cout << "  跳过 IK：缺少有效的当前关节反馈\n";
        } else if (!solution_model_angles) {
            std::cout << "  无可行逆解（需同时满足软限位、相对种子单轴步长 <= "
                      << kinematics::kMaxJointStepDeg << "°及位姿校验）\n";
        } else {
            print_joint_row("IK输出 / 模型角 q", *solution_model_angles);
            print_joint_row("映射计算 / 设备目标角 m", *solution_device_angles);
            if (device_angle_difference) {
                print_joint_row("IK目标m - Studio反馈m", *device_angle_difference);
            }
            std::cout << "  IK解回代FK误差: 位置=" << *position_error_mm
                      << " mm，姿态=" << *orientation_error_deg << " deg\n";
            // 失败帧不替换历史解；previous 模式下下一次仍使用最近一次成功解。
            previous_ik_solution_ = *solution_model_angles;
        }

        std::cout << "========================================" << std::endl;
    }

    void open_csv() {
        const std::filesystem::path path(options_.csv_path);
        if (!path.parent_path().empty()) {
            std::filesystem::create_directories(path.parent_path());
        }
        csv_.open(path, std::ios::out | std::ios::trunc);
        if (!csv_) {
            throw std::runtime_error("无法创建 CSV 文件: " + options_.csv_path);
        }

        csv_ << "schema_version,sample_index,timestamp_utc,studio_host,studio_port,"
                "rpy_convention,studio_joint_angles_valid,"
                "studio_x_m,studio_y_m,studio_z_m,"
                "studio_roll_deg,studio_pitch_deg,studio_yaw_deg";
        append_joint_columns("mapping_m0_j", "_deg");
        csv_ << ",mapping_direction_j1,mapping_direction_j2,mapping_direction_j3,"
                "mapping_direction_j4,mapping_direction_j5,mapping_direction_j6";
        append_joint_columns("dh_theta_offset_j", "_deg");
        append_joint_columns("studio_device_m_j", "_deg");
        append_joint_columns("studio_model_q_j", "_deg");
        csv_ << ",ik_seed_source";
        append_joint_columns("ik_seed_q_j", "_deg");
        csv_ << ",ik_success";
        append_joint_columns("ik_model_q_j", "_deg");
        append_joint_columns("ik_device_target_m_j", "_deg");
        append_joint_columns("ik_minus_studio_device_m_j", "_deg");
        csv_ << ",fk_x_m,fk_y_m,fk_z_m,fk_w,fk_qx,fk_qy,fk_qz,"
                "fk_position_error_mm,fk_orientation_error_deg,ik_seed_mode\n";
        csv_.flush();
        if (!csv_) {
            throw std::runtime_error("无法写入 CSV 表头: " + options_.csv_path);
        }
        std::cerr << "复现数据 CSV: " << options_.csv_path << " (启动时覆盖旧文件)\n";
    }

    void append_joint_columns(const char* prefix, const char* suffix) {
        for (int joint = 1; joint <= 6; ++joint) {
            csv_ << ',' << prefix << joint << suffix;
        }
    }

    void write_csv_row(
        const Sample& sample,
        const std::optional<joint_mapping::JointAngles>& current_model_angles,
        const std::string& seed_source,
        const std::optional<kinematics::JointAngles>& seed_model_angles,
        const std::optional<kinematics::JointAngles>& solution_model_angles,
        const std::optional<joint_mapping::JointAngles>& solution_device_angles,
        const std::optional<joint_mapping::JointAngles>& device_angle_difference,
        const std::optional<kinematics::Pose>& achieved_pose,
        const std::optional<double>& position_error_mm,
        const std::optional<double>& orientation_error_deg
    ) {
        std::ostringstream row;
        row << std::setprecision(std::numeric_limits<double>::max_digits10)
            << 1 << ',' << (printed_ + 1) << ',' << csv_escape(utc_timestamp()) << ','
            << csv_escape(options_.host) << ',' << csv_escape(options_.port) << ','
            << csv_escape("Studio_Euler_XYZ_RxRyRz") << ','
            << (sample.device_angles ? 1 : 0) << ','
            << sample.xyz_m[0] << ',' << sample.xyz_m[1] << ',' << sample.xyz_m[2] << ','
            << sample.rpy_deg[0] << ',' << sample.rpy_deg[1] << ',' << sample.rpy_deg[2];

        for (const double zero_device_angle : joint_mapping::kM0Deg) {
            row << ',' << zero_device_angle;
        }
        for (const int direction : joint_mapping::kDirection) {
            row << ',' << direction;
        }
        for (const double offset : kinematics::kDhThetaOffsetDeg) {
            row << ',' << offset;
        }

        const auto append_angles = [&row](
            const std::optional<joint_mapping::JointAngles>& angles
        ) {
            for (std::size_t i = 0; i < 6; ++i) {
                row << ',';
                if (angles) row << (*angles)[i];
            }
        };
        append_angles(sample.device_angles);
        append_angles(current_model_angles);

        row << ',' << csv_escape(seed_source);
        append_angles(seed_model_angles);

        row << ',';
        if (!options_.capture_only) row << (solution_model_angles ? 1 : 0);
        append_angles(solution_model_angles);
        append_angles(solution_device_angles);
        append_angles(device_angle_difference);

        const auto append_optional_double = [&row](const std::optional<double>& value) {
            row << ',';
            if (value) row << *value;
        };
        for (const std::optional<double> value : {
                 achieved_pose ? std::optional<double>(achieved_pose->x_m) : std::nullopt,
                 achieved_pose ? std::optional<double>(achieved_pose->y_m) : std::nullopt,
                 achieved_pose ? std::optional<double>(achieved_pose->z_m) : std::nullopt,
                 achieved_pose ? std::optional<double>(achieved_pose->orientation.w) : std::nullopt,
                 achieved_pose ? std::optional<double>(achieved_pose->orientation.x) : std::nullopt,
                 achieved_pose ? std::optional<double>(achieved_pose->orientation.y) : std::nullopt,
                 achieved_pose ? std::optional<double>(achieved_pose->orientation.z) : std::nullopt,
                 position_error_mm,
                 orientation_error_deg}) {
            append_optional_double(value);
        }

        row << ',' << csv_escape(options_.capture_only ? "capture_only" : options_.seed_mode);
        csv_ << row.str() << '\n';
        csv_.flush(); // 每条记录立即落盘，Ctrl+C 时保留已采集样本。
        if (!csv_) {
            throw std::runtime_error("写入 CSV 失败: " + options_.csv_path);
        }
    }

    Options options_;
    asio::io_context io_;
    tcp::resolver resolver_;
    websocket::stream<beast::tcp_stream> ws_;
    beast::flat_buffer buffer_;
    asio::steady_timer deadline_;
    asio::signal_set signals_;
    Clock::time_point last_print_{};
    int printed_ = 0;
    int exit_code_ = 0;
    bool stopped_ = false;
    std::optional<kinematics::JointAngles> previous_ik_solution_;
    std::ofstream csv_;
    std::ofstream events_;
};
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") { print_usage(); return 0; }
        StudioReader reader(parse_options(argc, argv));
        // run() 已经通过异步 WebSocket 读取持续运行；输出频率由 --interval-ms 控制。
        return reader.run();
    } catch (const std::exception& error) {
        std::cerr << "错误: " << error.what() << '\n';
        return 1;
    }
}
