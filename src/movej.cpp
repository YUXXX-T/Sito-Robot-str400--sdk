#include "movej.hpp"
#include "robot_reader.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <vector>

namespace movej{
namespace{
constexpr double kPi = 3.14159265358979323846;
constexpr double kStudioFrequencyHz = 50.0;
constexpr double kStudioKp = 1.8;
constexpr double kStudioKd = 0.008;
constexpr double kStudioDerivativeAlpha = 0.2;

using Matrix7 = std::array<std::array<double, 8>, 7>;
using SpeedCommands = std::array<std::int32_t, 6>;


double normalize_to_180(double degrees) {
    const double wrapped = std::atan2(
        std::sin(degrees * kPi / 180.0),
        std::cos(degrees * kPi / 180.0)
    ) * 180.0 / kPi;
    return std::round(wrapped * 100.0) / 100.0;
} // normalize_to_180


std::array<double, 7> solve_7x7(Matrix7 matrix){
    // 高斯消元，带部分主元选取
    for (std::size_t column=0; column<7; ++column){
        std::size_t pivot = column;
        for(std::size_t row = column+1; row<7; ++row){
            if(std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) {
                pivot = row;
            }
        }

        if(std::abs(matrix[pivot][column]) < 1e-12) {
            throw std::runtime_error("Studio MOVEJ 插值矩阵退化");
        }

        std::swap(matrix[column], matrix[pivot]);

        const double divisor = matrix[column][column];
        for (std::size_t j = column; j<8; ++j){
            matrix[column][j] /= divisor;
        }

        for(std::size_t row = 0; row<7; ++row){
            if (row==column) continue;
            const double factor = matrix[row][column];
            for(std::size_t j = column; j<8; ++j){
                matrix[row][j] -= factor * matrix[column][j];
            }
        }
    }

    std::array<double, 7> result{};
    for(std::size_t row = 0; row<7; ++row){
        result[row] = matrix[row][7];
    }
    return result;
} // solve_7x7

std::array<double, 7> movej_coefficients(double displacement, double segment_1_seconds, double segment_2_seconds, double segment_3_seconds){
    const double p1_delta = 0.3 * displacement;
    const double p2_delta = 0.6 * displacement;
    const double p3_delta = 0.1 * displacement;

    const double c = segment_1_seconds;
    const double a = segment_2_seconds;
    const double t = segment_3_seconds;

    // MoveLInterpolate 中的 7x7 矩阵。
    // MOVEJ 调用时，起点/终点速度和加速度均为 0。
    Matrix7 matrix{{
        {{1, 1, 0, 0, 0, 0, 0, p1_delta}},
        {{3/c, 4/c, -1/a, 0, 0, 0, 0, 0}},
        {{6/(c*c), 12/(c*c), 0, -2/(a*a), 0, 0, 0, 0}},
        {{0, 0, 1, 1, 1, 0, 0, p2_delta}},
        {{0, 0, 1/a, 2/a, 3/a, -3/t, 4/t, 0}},
        {{0, 0, 0, 2/(a*a), 6/(a*a), 6/(t*t), -12/(t*t), 0}},
        {{0, 0, 0, 0, 0, 1, -1, p3_delta}}
    }};

    return solve_7x7(matrix);
} // movej_coefficients

double evaluate_movej_axis(double start, double target, const std::array<double, 7>& h, double time_seconds, double total_seconds){
    // 计算的是某一个关节在指定时刻的轨迹参考角
    // 把一次 MOVEJ 分成三段：
    // - 0 到 0.30T：走到总角度变化的 30%；
    // - 0.30T 到 0.65T：走到总角度变化的 90%；
    // - 0.65T 到 T：走完剩余 10%。
    const double t1 = 0.30 * total_seconds;
    const double t2 = 0.65 * total_seconds;
    const double t3 = total_seconds;

    const double p1 = start + 0.30 * (target - start);
    const double p2 = start + 0.90 * (target - start);

    // h 是 movej_coefficients() 通过 7×7 方程求出的多项式系数。函数按当前时间 time_seconds 选择对应区段，计算这个关节此刻应该到达的角度。
    // 每个控制周期会对 J1～J6 分别调用它，得到六轴参考角，再交给 SpeedFollow 控制器计算速度。
    if (time_seconds <= t1) {
        const double r = time_seconds / t1;
        return start + h[0]*r*r*r + h[1]*r*r*r*r;
    }

    if (time_seconds <= t2) {
        const double r = (time_seconds - t1) / (t2 - t1);
        return p1 + h[2]*r + h[3]*r*r + h[4]*r*r*r;
    }

    const double r = (time_seconds - t2) / (t3 - t2);
    const double c1 = -4.0*h[6] + 3.0*h[5];
    const double c2 =  6.0*h[6] - 3.0*h[5];
    const double c3 = -4.0*h[6] + h[5];

    return p2 + c1*r + c2*r*r + c3*r*r*r + h[6]*r*r*r*r;
} // evaluate_movej_axis

bool inside_limit(double angle, const std::pair<double, double>& limit) {
    return std::isfinite(angle)
        && angle >= limit.first
        && angle <= limit.second;
} // inside_limit

class SpeedFollowController {
public:
    SpeedFollowController(
        const JointLimits& limits, const std::array<double, 6>& gear_ratio
    ):limits_(limits), gear_ratio_(gear_ratio){
        reset();
    }

    void reset(){
        last_error_.fill(0.0);
        filtered_derivative_.fill(0.0);
    } // reset()

    SpeedCommands update(
        const joint_mapping::JointAngles& actual,
        const joint_mapping::JointAngles& reference,
        double dt_seconds
    ) {
        if(!std::isfinite(dt_seconds) || dt_seconds<=0.0){
            throw std::invalid_argument("PD 控制周期无效");
        }
        SpeedCommands commands{};
        for(std::size_t i = 0; i<6; ++i){
            const double target = reference[i];
            if (!inside_limit(target, limits_[i])) {
                throw std::runtime_error("PD 目标角度超出关节软件范围");
            }

            // 关节角属于有限转角，误差直接相减，不按 360° 环绕。
            const double error = target - actual[i];

            // 使用相邻两次控制更新的实际时间间隔计算微分项。
            const double raw_derivative = (error - last_error_[i])/dt_seconds;
            // 一阶低通滤波：
            // filtered = 0.8 * old + 0.2 * current
            filtered_derivative_[i] = 0.8*filtered_derivative_[i] + 0.2*raw_derivative;
            // 实际角度超出软件范围时降低输出
            const double scale = inside_limit(actual[i], limits_[i])? 1.0 : 0.1;
            // 这个结果就是直接写入 0x002F 的值
            const double command =  scale*gear_ratio_[i]*(1.8*error + 0.008*filtered_derivative_[i]);

            if(!std::isfinite(command) || command < std::numeric_limits<std::int32_t>::min()
                || command > std::numeric_limits<std::int32_t>::max()){
                throw std::out_of_range("速度命令超出 int32_t 范围");
            }

            // 浮点转整数时向零截断
            commands[i] = static_cast<std::int32_t>(command);
            last_error_[i] = error;
            
        } // for

        return commands;
    } // update


private:
    JointLimits limits_;
    std::array<double, 6> gear_ratio_{};

    std::array<double, 6> last_error_{};
    std::array<double, 6> filtered_derivative_{};


}; //  class SpeedFollowController




JointAngles read_angles(int fd) {
    JointAngles result{};
    for (int joint = 1; joint <= 6; ++joint) {
        result[joint - 1] = robot_reader::read_joint_angle(fd, joint);
    }
    return result;
} // read_angles

void check_errors(int fd) {
    for (int joint = 1; joint <= 6; ++joint) {
        const auto error = robot_reader::read_joint_error_code(fd, joint);
        if (error != 0) {
            throw std::runtime_error("J" + std::to_string(joint) + " 故障码=" + std::to_string(error));
        }
    } // for
} // check_errors

void check_ready(int fd) {
    check_errors(fd);
    for (int joint = 1; joint <= 6; ++joint) {
        if (!robot_reader::is_joint_enabled(fd, joint)) {
            throw std::runtime_error("J" + std::to_string(joint) + " 未使能");
        }
        if (robot_reader::is_joint_running(fd, joint)) {
            throw std::runtime_error("J" + std::to_string(joint) + " 正在运行");
        }
    } // for
} // check_ready


void send_speeds(int fd, const std::array<std::int32_t, 6>& speed) {
    for (int joint = 1; joint <= 6; ++joint) {
        robot_reader::send_write_request(fd, static_cast<std::uint8_t>(joint), 0x002F, speed[joint - 1]);
    }
} // send_speeds

void send_zero_speed(int fd) noexcept {
    for (int joint = 1; joint <= 6; ++joint) {
        try {
            robot_reader::send_write_request(fd, static_cast<std::uint8_t>(joint), 0x002F, 0);
        } catch (...) {
            // 继续尝试给其余轴发送零速度。
        }
    }
} // send_zero_speed

void send_stop(int fd) {
    for (int joint = 1; joint <= 6; ++joint) {
        robot_reader::writeStop(fd, joint);
    }
} // send_stop

bool send_zero_speeds(int fd) noexcept {
    bool all_ok = true;
    for (int joint = 1; joint <= 6; ++joint) {
        try {
            robot_reader::send_write_request(
                fd, static_cast<std::uint8_t>(joint), 0x002F, 0
            );
        } catch (...) {
            all_ok = false;
        }
    }
    return all_ok;
} // send_zero_speeds

bool send_stop_all(int fd) noexcept {
    bool all_ok = true;
    for (int joint = 1; joint <= 6; ++joint) {
        try {
            robot_reader::writeStop(fd, joint);
        } catch (...) {
            all_ok = false;
        }
    }
    return all_ok;
} // send_stop_all

bool wait_for_stopped(int fd, std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        bool all_stopped = true;
        for (int joint = 1; joint <= 6; ++joint) {
            if (robot_reader::is_joint_running(fd, joint)) {
                all_stopped = false;
            }
        }
        if (all_stopped) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
} // wait_for_stopped


} // namespace

JointAngles resolve_target(const JointAngles& current, const JointAngles& command, TargetMode mode, const JointLimits& limits) {
    JointAngles target{};

    for (std::size_t i = 0; i < 6; ++i) {
        const double requested = mode == TargetMode::Absolute ? command[i] : current[i] + command[i];

        target[i] = normalize_to_180(requested);

        if (!std::isfinite(target[i]) || target[i] < limits[i].first || target[i] > limits[i].second) {
            throw std::out_of_range("J" + std::to_string(i + 1) + " MOVEJ 目标超出软限位");
        }
    }
    return target;
} // resolve_target

std::vector<JointAngles> sample_movej(const JointAngles& start, const JointAngles& target, double duration_seconds, std::chrono::milliseconds period){
    if (!std::isfinite(duration_seconds) || duration_seconds < 0.5 || duration_seconds > 3600.0) {
        throw std::invalid_argument("Studio MOVEJ 时间范围为 0.5～3600 秒");
    }
    if (period.count() <= 0) {
        throw std::invalid_argument("MOVEJ 采样周期必须大于 0");
    }


    const double period_seconds =std::chrono::duration<double>(period).count(); // 把 std::chrono::milliseconds 转成秒
    const auto intervals = static_cast<std::size_t>(std::llround(duration_seconds / period_seconds));
    if (intervals == 0) {
        throw std::invalid_argument("MOVEJ 采样点数为零");
    }

    const double t1 = 0.30 * duration_seconds;
    const double t2 = 0.65 * duration_seconds;
    
    std::array<std::array<double, 7>, 6> coefficients{};
    for (std::size_t joint = 0; joint < 6; ++joint) {
        coefficients[joint] = movej_coefficients(target[joint]-start[joint], t1, t2-t1, duration_seconds-t2);
    }

    
    std::vector<JointAngles> points;
    points.reserve(intervals + 1);

    for (std::size_t sample = 0; sample <= intervals; ++sample) {
        const double time = std::min(sample * period_seconds, duration_seconds);
        JointAngles point{};
        for (std::size_t joint = 0; joint < 6; ++joint) {
            point[joint] = evaluate_movej_axis(start[joint], target[joint], coefficients[joint], time, duration_seconds);
        }

        // 保证最后一个参考点精确等于目标。
        if (sample == intervals) {
            point = target;  // 保证末采样点精确等于目标。
        }
        points.push_back(point);
    }
    return points;
} // sample_movej

bool movej(int fd, const JointAngles& command, TargetMode mode, double duration_seconds, const JointLimits& limits, const Options& options){
    if (options.control_period.count() <= 0) {
        throw std::invalid_argument("MOVEJ 控制周期必须大于 0");
    }

    check_ready(fd);
    
    const JointAngles current = read_angles(fd);
    const JointAngles target = resolve_target(current, command, mode, limits);
    const auto trajectory = sample_movej(current, target, duration_seconds, options.control_period);

    SpeedFollowController controller(limits, options.gear_ratio);
    bool any_speed_sent = false;
    try {
        for (std::size_t index = 0; index < trajectory.size(); ++index) {
            const auto cycle_start = std::chrono::steady_clock::now();
            const JointAngles actual = read_angles(fd);
            check_errors(fd);
            const double dt_seconds =
                std::chrono::duration<double>(options.control_period).count();
            const SpeedCommands speeds =
                controller.update(actual, trajectory[index], dt_seconds);

            any_speed_sent = true;
            send_speeds(fd, speeds);
            std::this_thread::sleep_until(cycle_start + options.control_period);
        }

        // 继续用末点作反馈参考，直到角度误差连续稳定达到要求。
        std::optional<std::chrono::steady_clock::time_point> stable_since;
        const auto deadline = std::chrono::steady_clock::now() + options.motion_timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto cycle_start = std::chrono::steady_clock::now();
            const JointAngles actual = read_angles(fd);
            check_errors(fd);

            const double dt_seconds =
                std::chrono::duration<double>(options.control_period).count();
            const SpeedCommands speeds = controller.update(actual, target, dt_seconds);
            send_speeds(fd, speeds);

            bool in_tolerance = true;
            for (std::size_t i = 0; i < 6; ++i) {
                if (std::abs(target[i] - actual[i]) > options.angle_tolerance_deg) {
                    in_tolerance = false;
                }
            }
            if (in_tolerance) {
                if (!stable_since) stable_since = std::chrono::steady_clock::now();
                if (std::chrono::steady_clock::now() - *stable_since >= options.settle_time) break;
            } else {
                stable_since.reset();
            }

            std::this_thread::sleep_until(cycle_start + options.control_period);
        } // while

        if (!stable_since || std::chrono::steady_clock::now() - *stable_since < options.settle_time) {
            throw std::runtime_error("MOVEJ 超时，角度未稳定到目标范围");
        }
        if (!send_zero_speeds(fd)) {
            throw std::runtime_error("MOVEJ 零速度命令未能全部发送");
        }
        if (!send_stop_all(fd)) {
            throw std::runtime_error("MOVEJ Stop 未能全部发送");
        }
        if (!wait_for_stopped(fd, std::chrono::seconds(2))) {
            throw std::runtime_error("MOVEJ Stop 后仍有关节报告运行中");
        }

        return true;
    } catch (...) {
        if (any_speed_sent) {
            send_zero_speeds(fd);
            send_stop_all(fd);
        }
        throw;
    } // try-catch
} // movej

} // namespace movej
