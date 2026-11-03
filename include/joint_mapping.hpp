#pragma once

#include <array>
#include <cstddef>

// 独立的设备角 m <-> 模型关节角 q 映射；全部使用度，不涉及 DH theta 或通信。
namespace joint_mapping {

using JointAngles = std::array<double, 6>;
using JointLimits = std::array<std::array<double, 2>, 6>;

// 模型 q=0 时对应的设备角。当前为参考资料推导的初值，实机标定后在此修改。
// 此偏移与 kinematics.cpp 中的 DH 偏置不同，两者各应用一次。
inline constexpr JointAngles kM0Deg{0.0, 0.0, 90.0, 0.0, 90.0, 0.0};
// 设备角增加时，模型角同向取 +1，反向取 -1。
inline constexpr std::array<int, 6> kDirection{-1, -1, -1, -1, -1, -1};

// 设备角软限位，2026-10-31 按用户在 Studio 中测得的范围配置，顺序为 [最小, 最大]。
// J6 尚未重新测量，暂保留原 ±170°；此表不是模型角限位。
inline constexpr JointLimits kDeviceJointLimitsDeg{{
    {{-170.0, 150.0}},
    {{-125.0, 125.0}},
    {{-125.0, 125.0}},
    {{-170.0, 170.0}},
    {{-115.0, 115.0}},
    {{-170.0, 170.0}}
}};

// q = direction * (m - m0)。不折回 ±180°，模型 J3/J5 可以超过 180°。
// 输入或转换结果非有限值时抛出 std::invalid_argument；不在此进行限位裁剪。
JointAngles device_to_model(const JointAngles& device_angles);
// m = m0 + direction * q；用于将 IK 输出交回设备角控制器。
JointAngles model_to_device(const JointAngles& model_angles);

// IK 使用的模型角范围由设备限位自动换算，避免修改 m0 后忘记同步限位。
constexpr JointLimits model_joint_limits() {
    JointLimits result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
        const double a = kDirection[i] * (kDeviceJointLimitsDeg[i][0] - kM0Deg[i]);
        const double b = kDirection[i] * (kDeviceJointLimitsDeg[i][1] - kM0Deg[i]);
        result[i] = a < b ? std::array<double, 2>{a, b}
                          : std::array<double, 2>{b, a};
    }
    return result;
}

static_assert([] {
    for (const int direction : kDirection) {
        if (direction != -1 && direction != 1) return false;
    }
    return true;
}(), "关节方向系数必须为 +1 或 -1");

} // namespace joint_mapping
