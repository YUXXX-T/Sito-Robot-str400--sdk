#include "joint_mapping.hpp"
#include "kinematics.hpp"
#include "movel.hpp"
#include "robot_reader.hpp"

#include <asm/termbits.h>  // Linux termios2，支持 2250000 波特率
#include <cerrno>
#include <cmath>
#include <chrono>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <system_error>
#include <thread>
#include <unistd.h>

namespace {

[[noreturn]] void throw_system_error(const char* operation) {
    throw std::system_error(errno, std::generic_category(), operation);
}

void configure_serial_port(int fd) {
    termios2 settings{};
    if (::ioctl(fd, TCGETS2, &settings) != 0) {
        throw_system_error("读取串口配置");
    }

    settings.c_iflag = 0;
    settings.c_oflag = 0;
    settings.c_lflag = 0;

    // 8N1：8 个数据位、无校验、1 个停止位。
    settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS | CBAUD | CIBAUD);
    settings.c_cflag |= CS8 | CLOCAL | CREAD | BOTHER;
    settings.c_ispeed = 2250000;
    settings.c_ospeed = 2250000;

    // 协议层使用 poll() 等待数据，因此 read() 不自行阻塞。
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;

    if (::ioctl(fd, TCSETS2, &settings) != 0) {
        throw_system_error("设置串口配置");
    }
    if (::ioctl(fd, TCFLSH, TCIOFLUSH) != 0) {
        throw_system_error("清空串口缓存");
    }
}

double parse_double(const char* text, const char* name) {
    try {
        const std::string value{text};
        std::size_t used = 0;
        const double result = std::stod(value, &used);
        if (used != value.size() || !std::isfinite(result)) {
            throw std::invalid_argument("不是有限数字");
        }
        return result;
    } catch (const std::exception&) {
        throw std::invalid_argument(
            std::string(name) + " 必须是有限数字"
        );
    }
}

movel::JointLimits device_joint_limits() {  // 角度映射
    movel::JointLimits result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
        result[i] = {
            joint_mapping::kDeviceJointLimitsDeg[i][0],
            joint_mapping::kDeviceJointLimitsDeg[i][1]
        };
    }
    return result;
}

void print_usage(const char* program) {
    std::cerr
        << "用法: " << program
        << " <x_mm> <y_mm> <z_mm> <roll_deg> <pitch_deg> <yaw_deg> <duration_s> [选项]\n"
        << "示例: " << program << " 0 157.7 203 180 0 -90 4\n"
        << "说明：位置是 base 坐标系下的绝对目标，单位 mm；不是相对位移。RPY 单位 deg，时间单位 s。\n"
        << "程序使用 Studio 的 RPY 约定 Rx(roll) * Ry(pitch) * Rz(yaw)。\n"
        << "程序会请求六轴使能，不自动回零；运动前检查故障、使能和运行状态。\n"
        << "日志文件：test_data/movel_run.csv（每次启动覆盖）。\n";
    std::cerr << "--no-replan：仅允许直达路径；默认直达失败时尝试两段 MOVEL。\n"
              << "--strict-singularities：启用原有严格避奇异拦截；默认允许通过有连续解的奇异位形。\n"
              << "--via x_mm y_mm z_mm roll_deg pitch_deg yaw_deg：指定中间位姿，可重复。\n"
              << "各段执行前全部预检查，中间点会到位并停止；每段至少 0.5 秒。\n";
}

void print_pose(const kinematics::Pose& pose) {
    const auto rpy =
        kinematics::studio_rpy_from_quaternion(pose.orientation);
    std::cout << std::fixed << std::setprecision(6)
              << "当前计算位姿（执行前）\n"
              << "  XYZ (m): [" << pose.x_m << ", "
              << pose.y_m << ", " << pose.z_m << "]\n"
              << "  XYZ (mm): [" << pose.x_m * 1000.0 << ", "
              << pose.y_m * 1000.0 << ", "
              << pose.z_m * 1000.0 << "]\n"
              << "  RPY (deg): [" << rpy[0] << ", "
              << rpy[1] << ", " << rpy[2] << "]\n";
}

void recover_running_joints_noexcept(int fd) noexcept {
    bool found_running = false;
    for (int joint = 1; joint <= 6; ++joint) {
        try {
            if (!robot_reader::is_joint_running(fd, joint)) {
                continue;
            }
            found_running = true;
            std::cerr << "检测到 J" << joint
                      << " 仍在运行，发送异常恢复 Stop\n";
            robot_reader::writeStop(fd, joint);
        } catch (const std::exception& error) {
            std::cerr << "恢复 J" << joint << " 失败："
                      << error.what() << '\n';
        } catch (...) {
            std::cerr << "恢复 J" << joint << " 发生未知异常\n";
        }
    }

    if (found_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
}

void cleanup_after_failure_noexcept(int fd) noexcept {
    std::cerr << "执行 MOVEL 异常清理：零速度、Stop、失能\n";

    for (int joint = 1; joint <= 6; ++joint) {
        try {
            robot_reader::send_write_request(
                fd, static_cast<std::uint8_t>(joint), 0x002F, 0
            );
        } catch (const std::exception& error) {
            std::cerr << "J" << joint << " 零速度清理失败："
                      << error.what() << '\n';
        } catch (...) {
            std::cerr << "J" << joint << " 零速度清理发生未知异常\n";
        }
    }

    for (int joint = 1; joint <= 6; ++joint) {
        try {
            robot_reader::writeStop(fd, joint);
        } catch (const std::exception& error) {
            std::cerr << "J" << joint << " Stop 清理失败："
                      << error.what() << '\n';
        } catch (...) {
            std::cerr << "J" << joint << " Stop 清理发生未知异常\n";
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    for (int joint = 1; joint <= 6; ++joint) {
        try {
            robot_reader::writeDisable(fd, joint);
        } catch (const std::exception& error) {
            std::cerr << "J" << joint << " 失能清理失败："
                      << error.what() << '\n';
        } catch (...) {
            std::cerr << "J" << joint << " 失能清理发生未知异常\n";
        }
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        print_usage(argv[0]);
        return 0;
    }

    if (argc < 8) {
        print_usage(argv[0]);
        return 1;
    }

    try {
        // 命令行沿用 Studio MOVEL：前三个位置参数使用毫米。
        const double x_m = parse_double(argv[1], "x_mm") / 1000.0;
        const double y_m = parse_double(argv[2], "y_mm") / 1000.0;
        const double z_m = parse_double(argv[3], "z_mm") / 1000.0;
        const double roll_deg = parse_double(argv[4], "roll_deg");
        const double pitch_deg = parse_double(argv[5], "pitch_deg");
        const double yaw_deg = parse_double(argv[6], "yaw_deg");
        const double duration_seconds = parse_double(argv[7], "duration_s");

        movel::PlanningOptions planning_options;
        for (int i = 8; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--no-replan") {
                planning_options.allow_replanning = false;
            } else if (option == "--strict-singularities") {
                planning_options.strict_singularity_avoidance = true;
            } else if (option == "--via") {
                if (argc - i <= 6) throw std::invalid_argument("--via 需要 XYZ(mm) 和 RPY(deg) 六个数值");
                const double x = parse_double(argv[i+1], "via x_mm") / 1000.0;
                const double y = parse_double(argv[i+2], "via y_mm") / 1000.0;
                const double z = parse_double(argv[i+3], "via z_mm") / 1000.0;
                const double roll = parse_double(argv[i+4], "via roll_deg");
                const double pitch = parse_double(argv[i+5], "via pitch_deg");
                const double yaw = parse_double(argv[i+6], "via yaw_deg");
                planning_options.via_poses.push_back({x, y, z,
                    kinematics::quaternion_from_studio_rpy(roll, pitch, yaw)});
                i += 6;
            } else {
                throw std::invalid_argument("未知 MOVEL 选项：" + option);
            }
        }

        if (duration_seconds < 0.5) {
            throw std::invalid_argument("duration_s 不能小于 0.5 秒");
        }

        const kinematics::Pose target{
            x_m, y_m, z_m,
            kinematics::quaternion_from_studio_rpy(roll_deg, pitch_deg, yaw_deg)
        };

        constexpr const char* port = "/dev/ttyUSB0";
        int fd = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) { throw_system_error("打开串口");}

        bool enable_phase_started = false;
        try {
            configure_serial_port(fd);

            movel::initialize_csv_log("test_data/movel_run.csv");

            // 清理上一次异常退出后可能残留的运行状态，再读取当前位姿。
            recover_running_joints_noexcept(fd);

            std::cout << "开始执行 MOVEL\n"
                      << "目标位置(mm): ["
                      << x_m * 1000.0 << ", "
                      << y_m * 1000.0 << ", "
                      << z_m * 1000.0 << "]\n"
                      << "目标姿态(deg): ["
                      << roll_deg << ", "
                      << pitch_deg << ", "
                      << yaw_deg << "]\n"
                      << "运动时间(s): " << duration_seconds << '\n';

            const auto current_pose = movel::read_current_pose(fd);
            print_pose(current_pose);
            movel::log_pose_record(
                "pre_motion_pose",
                current_pose,
                target,
                duration_seconds
            );

            std::cout << "开始预规划 MOVEL（暂不使能、不发送运动命令）\n";
            movel::validate_movel_plan(
                fd,
                target,
                duration_seconds,
                device_joint_limits(),
                planning_options
            );
            movel::log_pose_record(
                "preplan_success",
                current_pose,
                target,
                duration_seconds
            );
            std::cout << "MOVEL 预规划成功，开始使能\n";

            enable_phase_started = true;
            for (int i = 1; i <= 6; ++i) {
                const bool enable = robot_reader::writeEnable(fd, i);
                std::cout << "J" << i << "使能结果: " << (enable ? 1 : 0) << '\n';
            }

            // 六个驱动器连续使能后留出稳定时间，再开始读取故障和运行状态。
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            const bool success = movel::move_l_speed_follow(
                fd, target, duration_seconds, device_joint_limits(), planning_options);
            enable_phase_started = false;

            std::cout << "MOVEL 正常结束，六轴保持使能\n";

            ::close(fd);
            if (!success) {
                std::cerr << "MOVEL 执行失败\n";
                
                return 1;
            }

            std::cout << "MOVEL 执行完成\n";
            return 0;
        } catch (...) {
            if (enable_phase_started) {
                cleanup_after_failure_noexcept(fd);
            }
            ::close(fd);
            throw;
        }
    } catch (const std::exception& error) {
        std::cerr << "MOVEL 失败：" << error.what() << '\n';
        return 1;
    }
}
