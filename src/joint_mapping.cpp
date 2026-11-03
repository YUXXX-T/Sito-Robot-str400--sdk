#include "joint_mapping.hpp"

#include <cmath>
#include <stdexcept>

namespace joint_mapping {
namespace {

double finite_angle(double angle) {
    if (!std::isfinite(angle)) {
        throw std::invalid_argument("关节角映射要求有限角度和零位偏移");
    }
    return angle;
}

} // namespace

JointAngles device_to_model(const JointAngles& device_angles) {
    JointAngles result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
        result[i] = finite_angle(kDirection[i] * (finite_angle(device_angles[i]) - kM0Deg[i]));
    }
    return result;
}

JointAngles model_to_device(const JointAngles& model_angles) {
    JointAngles result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
        result[i] = finite_angle(kM0Deg[i] + kDirection[i] * finite_angle(model_angles[i]));
    }
    return result;
}

} // namespace joint_mapping
