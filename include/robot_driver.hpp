#pragma once

#include <cstdint>
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <limits>
#include <array>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

namespace robot_driver{  // 根据目标角度执行关节运动

enum class PositionMode : std::uint8_t {
    WithAcceleration = 0,    // 使用 0x0082 带加减速
    WithoutAcceleration = 1  // 使用 0x0081 无加减速
};

bool move_joint_to_angle(int fd, int joint_id, double target_angle_deg,
     PositionMode move_mode = PositionMode::WithAcceleration); // 使用带/不带加减速模式移动到绝对目标角度
bool move_joint_to_angle_after_home(
    int fd,
    int joint_id,
    double target_angle_deg,
    PositionMode move_mode = PositionMode::WithAcceleration
); // 使能、物理回零确认后，再执行小幅单关节运动
bool move_to_zero(int fd, int joint_id); // 单个关节回到原点
bool move_to_zero(int fd);               // 六个关节回到原点
bool reset_zero(int fd, int joint_id); // 单个关节设置当前位置为原点

bool move_joints_to_angles(int fd, const std::array<double, 6>& target_angles, 
        PositionMode move_mode = PositionMode::WithAcceleration); // 按 J1～J6 顺序发送六轴绝对目标


} // robot_driver
