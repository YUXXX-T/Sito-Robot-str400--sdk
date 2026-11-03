#pragma once
#include "joint_mapping.hpp"
#include "kinematics.hpp"

#include <array>
#include <string>
#include <utility>
#include <vector>
#include <chrono>


namespace movej{
// MOVEJ 的角度参数和轨迹都使用设备角 m，单位度。
using  JointAngles = std::array<double, 6>;
using JointLimits = std::array<std::pair<double, double>, 6>;

enum class TargetMode {
    Absolute,  // A:MOVEJ，参数是绝对设备角
    Relative   // R:MOVEJ，参数是相对当前设备角的增量
};

struct Options{
    TargetMode target_mode{TargetMode::Absolute};
    // StatusUpdate 默认报告的减速比。
    std::array<double, 6> gear_ratio{
        101.0, 101.0, 101.0, 101.0, 101.0, 101.0
    };
    std::chrono::milliseconds control_period{20};
    double angle_tolerance_deg{0.8};
    std::chrono::seconds settle_time{1};
    std::chrono::seconds motion_timeout{30};

}; 
// command 是六轴绝对角或相对角度增量，取决于 options.target_mode。
// limits 使用设备角范围。
// 运动结束发送零速度、等待角度稳定、发送 Stop 并确认停止；正常结束保持使能。
bool movej(int fd, const JointAngles& command, TargetMode mode, double duration_seconds, const JointLimits& limits, const Options& options = {});


} // namespace movej