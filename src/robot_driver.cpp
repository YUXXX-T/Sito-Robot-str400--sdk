#include "robot_reader.hpp"
#include "robot_driver.hpp"
#include "joint_mapping.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <array>
#include <string>

namespace {

// 当前实机测试使用默认模型：六个关节都使用 9192.96。
// 如果后续确认设备属于 Studio 的 oldModel，再改为 true。
constexpr bool kOldModel = false;

void validate_target_angle(int joint_id, double angle_deg) {
    if (joint_id < 1 || joint_id > 6) throw std::invalid_argument("joint_id 必须是 1～6");
    const auto range = joint_mapping::kDeviceJointLimitsDeg[static_cast<std::size_t>(joint_id-1)];
    if (!std::isfinite(angle_deg) || angle_deg < range[0] || angle_deg > range[1]) {
        throw std::invalid_argument("J" + std::to_string(joint_id) + " 目标设备角必须在 ["
            + std::to_string(range[0]) + ", " + std::to_string(range[1]) + "] 度范围内");
    }
}

double position_scale(int joint_id) {
    if (joint_id < 1 || joint_id >6) throw std::invalid_argument("joint_id 必须是 1～6");

    const double normal_model_scale[6]{
        9192.96, 9192.96, 9192.96,
        9192.96, 9192.96, 9192.96
    };

    const double old_model_scale[6]{
        9192.96, 9192.96, 4642.0,
        4642.0, 4642.0, 4642.0
    };

    const auto* scales = kOldModel
        ? old_model_scale
        : normal_model_scale;

    return scales[joint_id - 1];
} // position_scale

double angle_error(double current_angle, double target_angle) { // 计算出最小差距, 带符号
    double error = std::fmod(current_angle-target_angle+180.0, 360); // 取余，把角度差限制到一个周期内
    if (error<0.0) error += 360;
    
    return error - 180;
} // angle_error

std::uint16_t position_register(robot_driver::PositionMode mode){
    switch (mode) {
        case robot_driver::PositionMode::WithAcceleration:
            return 0x0082;
        case robot_driver::PositionMode::WithoutAcceleration:
            return 0x0081; 
    }

    throw std::invalid_argument("未知的位置控制模式");
} // position_register

std::int32_t position_from_angle(int joint_id, double angle_deg) {  // 把目标角度转换成设备位置控制命令需要的整数位置值
    const auto position64 = std::llround(
        angle_deg * position_scale(joint_id)
    );

    if (position64 < std::numeric_limits<std::int32_t>::min()
        || position64 > std::numeric_limits<std::int32_t>::max()) {
        throw std::out_of_range("目标位置超出 int32_t 范围");
    }

    return static_cast<std::int32_t>(position64);
}  // position_from_angle

void check_joint_ready_for_zero(int fd, int joint_id) {
    const auto error_code =
        robot_reader::read_joint_error_code(fd, joint_id);
    if (error_code != 0) {
        throw std::runtime_error(
            "J" + std::to_string(joint_id)
            + " 回零前检测到故障码 "
            + std::to_string(error_code)
        );
    }

    if (!robot_reader::is_joint_enabled(fd, joint_id)) {
        throw std::runtime_error(
            "J" + std::to_string(joint_id) + " 尚未使能"
        );
    }

    if (robot_reader::is_joint_running(fd, joint_id)) {
        throw std::runtime_error(
            "J" + std::to_string(joint_id) + " 当前正在运行"
        );
    }
}  // check_joint_ready_for_zero


void send_go_to_zero(int fd, int joint_id) {
    // GoToZero：0x0032 = 1；ResetZero 是另一条指令（0x0031）。
    robot_reader::send_write_request(
        fd,
        static_cast<std::uint8_t>(joint_id),
        0x0032,
        1
    );
    // move_joint_to_angle(
    //     fd,
    //     joint_id,
    //     0.0,
    //     robot_driver::PositionMode::WithAcceleration
    // );
}  // send_go_to_zero

void send_stop(int fd, int joint_id) {
    // Stop：0x0033 = 1。
    robot_reader::send_write_request(
        fd,
        static_cast<std::uint8_t>(joint_id),
        0x0033,
        1
    );
}  // send_stop

bool is_joint_at_zero(int fd, int joint_id, double tolerance_deg) {
    const auto error_code =
        robot_reader::read_joint_error_code(fd, joint_id);
    const auto running =
        robot_reader::is_joint_running(fd, joint_id);
    const auto angle =
        robot_reader::read_joint_angle(fd, joint_id);

    std::cout << "回零调试：J" << joint_id
              << " 角度=" << angle
              << "°，运行状态=" << (running ? 1 : 0)
              << "，故障码=" << error_code
              << '\n';

    if (error_code != 0) {
        throw std::runtime_error(
            "J" + std::to_string(joint_id)
            + " 回零过程中检测到故障码 "
            + std::to_string(error_code)
        );
    }

    return !running && std::abs(angle_error(angle, 0.0)) <= tolerance_deg;
}  // is_joint_at_zero

void stop_joints_noexcept(
    int fd,
    const std::array<int, 6>& joints,
    std::size_t count
) noexcept {
    for (std::size_t i = 0; i < count; ++i) {
        try {
            send_stop(fd, joints[i]);
        } catch (...) {
            // 回零失败后的停止是尽力而为，保留原始异常继续抛出。
            std::cout << "Stop Joints Noexcept" << " \n";
        }
    }
} // stop_joints_noexcept

bool move_to_zero_impl(
    int fd,
    const std::array<int, 6>& joints,
    std::size_t count
) {
    constexpr double zero_tolerance_deg = 0.08;
    const auto deadline =
        std::chrono::steady_clock::now()
        + std::chrono::seconds(30);
    bool motion_command_started = false;
    std::array<double, 6> initial_angles{};
    std::array<std::int32_t, 6> initial_positions{};

    try {
        // 1. 检查并记录所有目标关节的当前位置，再开始发送运动请求。
        for (std::size_t i = 0; i < count; ++i) {
            check_joint_ready_for_zero(fd, joints[i]);
            initial_angles[i] = robot_reader::read_joint_angle(fd, joints[i]);
            initial_positions[i] = position_from_angle(joints[i], initial_angles[i]);
        }

        // 2. 对照 Studio BackToZeroTask：先写各轴当前角度对应的位置整数。
        // 位置参考有偏移时，此写入可能引起运动，不能把它视作无运动同步。
        // 每次等待写回复即可；Studio 没有在这一步等待位置运动完成。
        for (std::size_t i = 0; i < count; ++i) {
            std::cout << "回零前置：J" << joints[i]
                      << " 当前角度=" << initial_angles[i]
                      << "°，位置整数=" << initial_positions[i]
                      << "，写入寄存器=0x0082\n";
            // 写请求可能已被设备接收但回复丢失，因此在发送前记录。
            motion_command_started = true;
            // robot_reader::send_write_request(
            //     fd, static_cast<std::uint8_t>(joints[i]), 0x0082, initial_positions[i]
            // );
        }

        // 3. 完成全部前置位置写入后，再逐个发送 GoToZero。
        for (std::size_t i = 0; i < count; ++i) {
            std::cout << "开始回零：J" << joints[i] << '\n';
            send_go_to_zero(fd, joints[i]);
        }

        // 4. 检查所有关节都接近 0° 且已经停止。
        while (std::chrono::steady_clock::now() < deadline) {
            bool all_done = true;

            for (std::size_t i = 0; i < count; ++i) {
                if (!is_joint_at_zero(fd, joints[i], zero_tolerance_deg)) {
                    all_done = false; 
                }
            }
            //全部到, 退出循环
            if (all_done) { 
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(30));  // 等待 30 ms，再检查下一轮
        }

        // 5. 最终超时检查
        bool all_done = true;
        for (std::size_t i = 0; i < count; ++i) {
            if (!is_joint_at_zero(fd, joints[i], zero_tolerance_deg)) {
                all_done = false;
            }
        }

        if (!all_done) {
            throw std::runtime_error("回零超时，未完成全部关节回零");
        }

        // 已确认全部到零且停止，直接返回，由调用方决定后续运动。
        return true;
    } catch (...) {
        if (motion_command_started) {
            stop_joints_noexcept(fd, joints, count);
        }
        throw;
    }
} // move_to_zero_impl

} // namespace

namespace robot_driver{

bool move_joint_to_angle(int fd, int joint_id, double target_angle_deg, PositionMode move_mode) { // 移动到绝对目标角度
    validate_target_angle(joint_id, target_angle_deg);

    // 运动模式识别: 是否带加速度运行
    const auto mode_reg = position_register(move_mode);

    // 运动前检查故障
    const auto error_code = robot_reader::read_joint_error_code(fd, joint_id);
    if (error_code!=0) throw std::runtime_error("运动前检测到关节故障");

    // 运动前使能
    if (!robot_reader::is_joint_enabled(fd, joint_id)) throw std::runtime_error("关节尚未使能");
    if (robot_reader::is_joint_running(fd, joint_id)) throw std::runtime_error("关节当前正在运行");

    // 读取当前位置
    const auto current_angle = robot_reader::read_joint_angle(fd, joint_id);
    // const auto difference = angle_error(current_angle, target_angle_deg);
    // if (std::abs(difference)>5.0) throw std::runtime_error("目标角度与当前位置相差超过 5°");

    
    const auto position64 = std::llround(target_angle_deg * position_scale(joint_id));
    if (position64 < std::numeric_limits<std::int32_t>::min() 
        || position64 > std::numeric_limits<std::int32_t>::max()) {
        throw std::out_of_range(
            "目标位置超出 int32_t 范围");
    }

    const auto target_position = static_cast<std::int32_t>(position64);

    std::cout << "运动调试：当前位置=" << current_angle
              << "°，目标角度=" << target_angle_deg
              << "°，位置整数=" << target_position
              << "，换算系数=" << position_scale(joint_id)
              << '\n';
    
    
    // 发送请求, 0x0082：带加减速的位置控制。
    robot_reader::send_write_request(fd, static_cast<std::uint8_t>(joint_id), mode_reg, target_position);

    // 运动过程中最多等待 3 秒。
    constexpr double tolerance_deg = 1;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
        // 故障状态
        const auto current_error = robot_reader::read_joint_error_code(fd, joint_id);
        const auto running = robot_reader::is_joint_running(fd, joint_id);
        const auto angle = robot_reader::read_joint_angle(fd, joint_id);

        std::cout << "运动调试：当前角度=" << angle
                  << "°，运行状态=" << (running ? 1 : 0)
                  << "，故障码=" << current_error
                  << '\n';

        if (current_error != 0) {
            throw std::runtime_error("运动过程中检测到关节故障");
        }

        const double remaining_error =
            std::abs(angle_error(angle, target_angle_deg));
        if (remaining_error <= tolerance_deg) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    throw std::runtime_error("运动超时，未到达目标角度");

} // move_joint_to_angle

bool move_joint_to_angle_after_home(
    int fd,
    int joint_id,
    double target_angle_deg,
    PositionMode move_mode
) {
    if (joint_id < 1 || joint_id > 6) {
        throw std::invalid_argument("joint_id 必须是 1～6");
    }

    validate_target_angle(joint_id, target_angle_deg);

    const auto initial_angle =
        robot_reader::read_joint_angle(fd, joint_id);
    const auto initial_error =
        robot_reader::read_joint_error_code(fd, joint_id);

    if (initial_error != 0) {
        throw std::runtime_error(
            "J" + std::to_string(joint_id)
            + " 运动前存在故障码 "
            + std::to_string(initial_error)
        );
    }

    if (robot_reader::is_joint_running(fd, joint_id)) {
        throw std::runtime_error(
            "J" + std::to_string(joint_id) + " 当前正在运行"
        );
    }

    std::cout << "特殊流程：J" << joint_id
              << " 初始角度=" << initial_angle << "°\n";

    if (!robot_reader::is_joint_enabled(fd, joint_id)) {
        std::cout << "特殊流程：使能 J" << joint_id << '\n';
        robot_reader::writeEnable(fd, joint_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 使用已验证的物理回零流程；这里不调用 ResetZero。
    std::cout << "特殊流程：物理回零 J" << joint_id << '\n';
    move_to_zero(fd, joint_id);
    // reset_zero(fd, joint_id);

    const auto zero_angle =
        robot_reader::read_joint_angle(fd, joint_id);
    const auto zero_error =
        robot_reader::read_joint_error_code(fd, joint_id);
    const auto zero_running =
        robot_reader::is_joint_running(fd, joint_id);

    std::cout << "特殊流程：回零反馈角度=" << zero_angle
              << "°，运行状态=" << (zero_running ? 1 : 0)
              << "，故障码=" << zero_error << '\n';

    if (zero_error != 0 || zero_running
        || std::abs(angle_error(zero_angle, 0.0)) > 0.8) {
        throw std::runtime_error(
            "J" + std::to_string(joint_id)
            + " 未能确认回到原点并停止"
        );
    }

    // move_joint_to_angle() 会等待角度反馈进入容差范围。
    return move_joint_to_angle(
        fd,
        joint_id,
        target_angle_deg,
        move_mode
    );
} // move_joint_to_angle_after_home

bool move_to_zero(int fd, int joint_id) { // 单个关节回到原点
    if (joint_id < 1 || joint_id > 6) {
        throw std::invalid_argument("joint_id 必须是 1～6");
    }

    const std::array<int, 6> joints{
        joint_id, 0, 0, 0, 0, 0
    };

    return move_to_zero_impl(fd, joints, 1);
} // move_to_zero (single joint)

bool move_to_zero(int fd) { // 六个关节回到原点
    const std::array<int, 6> joints{
        1, 2, 3, 4, 5, 6
    };

    return move_to_zero_impl(fd, joints, joints.size());
} // move_to_zero (all joints)

bool reset_zero(int fd, int joint_id){
    // 读取当前位置
    const auto current_angle = robot_reader::read_joint_angle(fd, joint_id);
    std::cout << "Current angle of Joint J" << joint_id << " :" << current_angle << "\n";

    const auto current_position = position_from_angle(joint_id, current_angle);

    std::cout << "J " << joint_id << " 当前角度=" << current_angle
              << "°，保持位置=" << current_position
              << '\n';
    
    // 0x0031 = 1：将当前位置重新定义为零点
    return robot_reader::send_write_request(
        fd,
        static_cast<std::uint8_t>(joint_id),
        0x0031,
        1
    );
} // reset_zero

bool move_joints_to_angles(int fd, const std::array<double, 6>& target_angles, 
        PositionMode move_mode) {
    
    constexpr double angle_tolerance_deg = 0.8;
    constexpr int required_stable_samples = 3;
    constexpr int joint_count = 6;

    const auto reg = position_register(move_mode);
    std::array<std::int32_t, 6> target_positions{};

    for (int i=0;i<joint_count;++i) validate_target_angle(i+1, target_angles[i]);

    // 先验证全部关节并计算全部目标，确保验证失败时还没有发出运动命令
    for (int i = 0; i < joint_count; ++i) {
        const int joint_id = i+1;
        const double target = target_angles[i];
        
        const auto error_code = robot_reader::read_joint_error_code(fd, joint_id);
        if (error_code != 0) {
            throw std::runtime_error(
                "J" + std::to_string(joint_id)
                + " 存在故障码 "
                + std::to_string(error_code)
            );
        }
        if (!robot_reader::is_joint_enabled(fd, joint_id)) {
            throw std::runtime_error(
                "J" + std::to_string(joint_id) + " 尚未使能"
            );
        }

        target_positions[i] = position_from_angle(joint_id, target);

    }
    
    bool any_command_sent = false;
    try {
        for (int i = 0; i<joint_count; ++i){
            const int joint_id = i+1;
            std::cout << "发送目标：J" << joint_id
                      << " -> " << target_angles[i]
                      << "°，位置整数=" << target_positions[i]
                      << '\n';
            
            // 每次调用会等待并校验这个关节的写回复
            any_command_sent = true;
            robot_reader::send_write_request(fd, static_cast<std::uint8_t>(joint_id),
                        reg, target_positions[i]);

        }

        // 角度连续多次进入容差后才判定完成，避免一次读数刚好落入容差。
        std::array<int, 6> stable_samples{};
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while(std::chrono::steady_clock::now()<deadline){
            bool all_done = true;
            for(int i = 0; i<joint_count; ++i){
                const int joint_id = i+1;
                const auto error_code = robot_reader::read_joint_error_code(fd, joint_id);

                if (error_code != 0) {
                    throw std::runtime_error(
                        "J" + std::to_string(joint_id)
                        + " 运动中出现故障码 "
                        + std::to_string(error_code)
                    );
                }

                const auto current_angle = robot_reader::read_joint_angle(fd, joint_id);
                const double error = std::abs(angle_error(current_angle, target_angles[i]));
                std::cout << "反馈：J" << joint_id
                          << " 当前=" << current_angle
                          << "°，目标=" << target_angles[i]
                          << "°，误差=" << error << "°\n";
                if (error < angle_tolerance_deg) {
                    ++stable_samples[i]; // 误差范围内次数
                } else {
                    stable_samples[i] = 0;
                }
                if (stable_samples[i]<required_stable_samples) all_done=false;

            }

            if (all_done) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

        }

        throw std::runtime_error("多关节运动超时，至少一个关节未到达目标");
    } catch (...) {
        // 部分目标可能已经被设备接受；失败时停止全部关节
        if (any_command_sent) {
            for (int joint_id = 1; joint_id<=joint_count; ++joint_id) {
                try {
                    robot_reader::writeStop(fd, joint_id);
                } catch (...) {
                    // 继续尝试停止其余关节，并保留最初的异常
                }
            }
        }
        throw;
    }

} // move_joints_to_angles


}
