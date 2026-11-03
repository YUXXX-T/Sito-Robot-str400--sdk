#pragma once

#include <array>
#include <optional>
#include <vector>

namespace kinematics {

// 表示末端相对 base 的姿态，分量顺序为 w、x、y、z
struct Quaternion{
    double w{1.0};
    double x{};
    double y{};
    double z{};
    
};
// 末端位姿：位置单位为米；orientation 是末端相对 base 的单位四元数，顺序为 w、x、y、z。
struct Pose {
    double x_m{};
    double y_m{};
    double z_m{};
    Quaternion orientation{};
};

// J1～J6 的模型关节角 q，单位度；不是设备角 m，也不是 DH theta。
// 设备读数须先经 joint_mapping::device_to_model() 转换；
// IK 输出须经 joint_mapping::model_to_device() 后才能下发。
using JointAngles = std::array<double, 6>;

// FK/IK 使用的 DH 角偏置，theta_deg = q_deg + 此数组对应项。
// 单独公开是为了让诊断数据能够记录并复现当时使用的运动学坐标约定。
inline constexpr JointAngles kDhThetaOffsetDeg{90.0, 90.0, 0.0, 0.0, -90.0, 0.0};

// 带种子的 IK 与 MOVEL 共用的单关节最大步长，单位度。
// 这是跳解筛选阈值，不是允许的速度/加速度。
inline constexpr double kMaxJointStepDeg = 90.0;

// 正向运动学：模型角 q -> 加 DH 偏置得到 theta -> 末端位姿。
Pose forward_kinematics(const JointAngles& joints);

// 从 TCP 位姿扣除模型中的 d6（含工具长度），返回 base 中的腕心坐标，单位米。
// 用于按真实腕心检测 J1 轴线奇异，不能用 TCP 的 XY 距离替代。
std::array<double, 3> wrist_center_position(const Pose& pose);

// 枚举所有通过软限位、FK 校验的离散逆解分支；不预先应用步长阈值或只保留最近解。
// reference 为模型角，用于奇异时分配自由角。奇异解族有无穷多解，返回各区间的
// 有限代表（包含按最大单轴变化细化的解），而不是声称穷举连续解族。
std::vector<JointAngles> inverse_kinematics_candidates(
    const Pose& pose, const JointAngles& reference
);

// 逆向运动学：内部求六个 DH 角 theta，再减去 DH 偏置返回模型角 q（度）。
// previous_solution 同样传模型角；这里不重复应用设备角映射。
// 候选任一轴与 previous_solution 的直接角差超过 kMaxJointStepDeg 即排除（不环绕）。
// 在通过步长、限位和 FK 校验的候选中选择最近解；没有合格候选返回 std::nullopt。
std::optional<JointAngles> inverse_kinematics(
    const Pose& pose,
    const JointAngles& previous_solution
);

// 无历史关节解时使用的 IK 入口，不限制相对模型零位的步长，仍检查软限位和 FK。
// 若存在多个可行分支，按接近模型零位的规则确定性选一个；
// 后续连续轨迹应改用带 previous_solution 的重载维持分支连续。
std::optional<JointAngles> inverse_kinematics(const Pose& pose);

// 按 R = Rz(yaw) * Ry(pitch) * Rx(roll) 将 RPY（度）转为四元数。
// 用于把外部使用 RPY 表示的目标姿态转换成内部 Pose。
Quaternion quaternion_from_rpy(double roll_deg, double pitch_deg, double yaw_deg);
// Studio 状态使用 Three.js Euler XYZ：R = Rx(roll) * Ry(pitch) * Rz(yaw)。
// 从 read_studio_pose 读取 Studio 的 RPY 时应调用此函数。
Quaternion quaternion_from_studio_rpy(
    double roll_deg,
    double pitch_deg,
    double yaw_deg
);
// 将单位四元数转换为 Studio 使用的 XYZ RPY，返回 roll、pitch、yaw（度）。
std::array<double, 3> studio_rpy_from_quaternion(const Quaternion& orientation);
// 在两个姿态之间沿较短旋转路径进行球面线性插值（SLERP）。
Quaternion slerp(const Quaternion& start, const Quaternion& target, double t);

}  // namespace kinematics
