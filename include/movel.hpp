#pragma once

#include "kinematics.hpp"

#include <array>
#include <string>
#include <utility>
#include <vector>

namespace movel {

// 设备角 m 的最小、最大角度，单位度，数组顺序为 J1～J6。
using JointLimits = std::array<std::pair<double, double>, 6>;

// 严格避奇异模式的轴线预留距离。普通连续求解模式不以该距离直接拒绝。
inline constexpr double kMinWristAxisDistanceM = 0.001;

struct PlanningOptions {
    // 直达失败时尝试先转姿态再平移，以及先平移再转姿态；各段均完整预检查。
    bool allow_replanning{true};
    // 可显式指定已确认环境允许经过的中间位姿（米、单位四元数）。按顺序强制经过。
    std::vector<kinematics::Pose> via_poses;
    // 默认允许有连续解的奇异过渡；true 恢复原来的 1 mm/1° 严格拦截。
    // 连续模式仍检查限位、单轴步长和候选之间的 FK 位姿残差。
    bool strict_singularity_avoidance{false};
};

struct PlannedSegment {
    kinematics::Pose start;
    kinematics::Pose target;
    double duration_seconds{};
    std::vector<kinematics::JointAngles> device_trajectory;
};

struct MovelPlan {
    std::string strategy;
    std::vector<PlannedSegment> segments;
};

// 纯离线规划：无设备读写。总时长分配到各段，每段至少 0.5 秒，因此可能延长总时长。
// 自动拆分只处理模型可达性和奇异约束，不检查环境碰撞。无法验证全部段则抛出异常。
// 仅 strict_singularity_avoidance 模式以肩部分支相反/奇异距离直接拒绝。
MovelPlan plan_movel(
    const kinematics::JointAngles& current_device_angles,
    const kinematics::Pose& target, double duration_seconds, const JointLimits& limits,
    const PlanningOptions& options = {}
);

// 初始化本次 MOVEL 的 CSV 日志；会覆盖旧文件并创建父目录。
void initialize_csv_log(const std::string& path);

// 写入一条执行前位姿记录。
void log_pose_record(
    const char* record_type,
    const kinematics::Pose& current_pose,
    const kinematics::Pose& target_pose,
    double duration_seconds
);

// 读取当前设备角并通过 FK 计算当前末端位姿，不发送使能或运动命令。
kinematics::Pose read_current_pose(int fd);

// 只做完整 MOVEL 轨迹预规划：读取当前角度、采样、逐点 IK、限位和奇异检查。
// 预规划不发送运动命令；失败时抛出异常，适合在自动使能前调用。
void validate_movel_plan(
    int fd,
    const kinematics::Pose& target,
    double duration_seconds,
    const JointLimits& limits,
    const PlanningOptions& options = {}
);

// 执行绝对 MOVEL：笛卡尔轨迹采样、逐点 IK、SpeedFollow 和 0x002F 速度下发。
// 内部负责设备角/模型角转换；limits 和控制器使用设备角，目标位置单位米。
bool move_l_speed_follow(
    int fd,
    const kinematics::Pose& target,
    double duration_seconds,
    const JointLimits& limits,
    const PlanningOptions& options = {}
);

}  // namespace movel
