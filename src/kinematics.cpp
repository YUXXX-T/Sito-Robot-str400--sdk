#include "kinematics.hpp"
#include "joint_mapping.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <vector>

namespace kinematics {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
// 工具长度加在法兰 DH 的 d6 上，单位米；实机工具长度确认后再调整。
constexpr double kToolLengthM = 0.050;

// FK 和 IK 共用的 DH 坐标零位偏置，单位度。
// q 是 joint_mapping 转换后的模型角：theta = (q + 偏置) * kDegToRad。
// 此处保留原 DH 建系；设备零位 m0 和方向只在 joint_mapping 中应用。
// 标准 DH 参数，长度单位米。
constexpr std::array<double, 6> kD{
    0.173, 0.0, 0.0, 0.1577, 0.0, 0.098 + kToolLengthM
};
constexpr std::array<double, 6> kA{
    0.0, 0.178, 0.0, 0.0, 0.0, 0.0
};
constexpr std::array<double, 6> kAlphaDeg{
    90.0, 0.0, 90.0, -90.0, 90.0, 0.0
};

// IK 始终在模型角 q 中求解和选支；模型限位由设备限位及 m0/方向自动换算。
constexpr auto kModelJointLimitsDeg = joint_mapping::model_joint_limits();

constexpr double kArmPositionCheckToleranceM = 1e-6;
constexpr double kAxisSingularityToleranceM = 1e-10;
constexpr double kIkPositionToleranceM = 1e-4;
constexpr double kIkOrientationToleranceRad = 0.1 * kDegToRad;
// 把 sin(theta5) 极小的姿态按腕部奇异处理，吸收肘部退化反算带来的浮点噪声。
constexpr double kWristSingularitySinTolerance = 1e-6;
// 仅吸收弧度/度转换的舍入误差；最终返回角度仍限制在原软限位内。
// 越界误差最多为 1e-5 度，接受后会夹回软限位。
constexpr double kJointLimitRoundoffDeg = 1e-5;

using Matrix3 = std::array<std::array<double, 3>, 3>;
using Matrix4 = std::array<std::array<double, 4>, 4>;
// 以下内部类型全部存放 DH 的 theta（弧度），可直接代入 DH 矩阵。
// 它们与公开接口的 JointAngles（模型关节角，度）含义不同。
using ArmTheta = std::array<double, 3>;       // theta1、theta2、theta3
using WristTheta = std::array<double, 3>;     // theta4、theta5、theta6
using DhAngles = std::array<double, 6>;       // theta1～theta6

DhAngles to_dh_angles(const JointAngles& joints){ // 把模型关节角 q 转成 DH 角 theta
    DhAngles theta{};
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (!std::isfinite(joints[i])) {
            throw std::invalid_argument("关节角度必须是有限数值");
        }
        theta[i] = (joints[i] + kDhThetaOffsetDeg[i]) * kDegToRad;
    }
    return theta;
} // to_dh_angles

Matrix4 identity_matrix(){ 
    Matrix4 result{};
    for (std::size_t i = 0; i < 4; ++i) {
        result[i][i] = 1.0;
    }
    return result;
} // identity_matrix

Matrix4 multiply(const Matrix4& lhs, const Matrix4& rhs){
    Matrix4 result{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t col = 0; col < 4; ++col) {
            for (std::size_t k = 0; k < 4; ++k) {
                result[row][col] += lhs[row][k] * rhs[k][col];
            }
        }
    }
    return result;
}  // multiply

Matrix4 make_standard_dh(double theta_rad, double d_m, double a_m, double alpha_rad){
    const double ct = std::cos(theta_rad);
    const double st = std::sin(theta_rad);
    const double ca = std::cos(alpha_rad);
    const double sa = std::sin(alpha_rad);

    return Matrix4{{
        {{ct, -st * ca,  st * sa, a_m * ct}},
        {{st,  ct * ca, -ct * sa, a_m * st}},
        {{0.0,      sa,      ca,       d_m}},
        {{0.0,     0.0,     0.0,       1.0}}
    }};
} // make_standard_dh

Matrix3 rotation_part(const Matrix4& transform){ // 提取旋转矩阵
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
            result[row][col] = transform[row][col];
        }
    }
    return result;
} // rotation_part

Matrix3 multiply(const Matrix3& lhs, const Matrix3& rhs){
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
            for (std::size_t k = 0; k < 3; ++k) {
                result[row][col] += lhs[row][k] * rhs[k][col];
            }
        }
    }
    return result;
}  // multiply

Matrix3 transpose(const Matrix3& matrix){
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
            result[row][col] = matrix[col][row];
        }
    }
    return result;
} // transpose 3x3

Quaternion normalize_quaternion(const Quaternion& q){ // 单位四元数
    const double norm = std::sqrt(
        q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z
    );
    if (!std::isfinite(norm) || norm < 1e-12) {
        throw std::invalid_argument("无效的零长度四元数");
    }

    return Quaternion{
        q.w / norm,
        q.x / norm,
        q.y / norm,
        q.z / norm
    };
} // normalize_quaternion 

Quaternion conjugate(const Quaternion& q){ // 共轭四元数
    return Quaternion{q.w, -q.x, -q.y, -q.z};
} // conjugate

Quaternion multiply_quaternion(const Quaternion& a, const Quaternion& b){
    return Quaternion{
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
} // multiply_quaternion

Quaternion quaternion_from_rotation_matrix(const Matrix3& matrix){
    Quaternion q{};
    const double trace =
        matrix[0][0] + matrix[1][1] + matrix[2][2];

    if (trace > 0.0) {
        const double s = 2.0 * std::sqrt(trace + 1.0);
        q.w = 0.25 * s;
        q.x = (matrix[2][1] - matrix[1][2]) / s;
        q.y = (matrix[0][2] - matrix[2][0]) / s;
        q.z = (matrix[1][0] - matrix[0][1]) / s;
    } else if (matrix[0][0] > matrix[1][1]
               && matrix[0][0] > matrix[2][2]) {
        const double s = 2.0 * std::sqrt(
            1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]
        );
        q.w = (matrix[2][1] - matrix[1][2]) / s;
        q.x = 0.25 * s;
        q.y = (matrix[0][1] + matrix[1][0]) / s;
        q.z = (matrix[0][2] + matrix[2][0]) / s;
    } else if (matrix[1][1] > matrix[2][2]) {
        const double s = 2.0 * std::sqrt(
            1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]
        );
        q.w = (matrix[0][2] - matrix[2][0]) / s;
        q.x = (matrix[0][1] + matrix[1][0]) / s;
        q.y = 0.25 * s;
        q.z = (matrix[1][2] + matrix[2][1]) / s;
    } else {
        const double s = 2.0 * std::sqrt(
            1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]
        );
        q.w = (matrix[1][0] - matrix[0][1]) / s;
        q.x = (matrix[0][2] + matrix[2][0]) / s;
        q.y = (matrix[1][2] + matrix[2][1]) / s;
        q.z = 0.25 * s;
    }

    return normalize_quaternion(q);
} // quaternion_from_rotation_matrix

Matrix3 rotation_from_quaternion(const Quaternion& input){ // 目标姿态转换成旋转矩阵
    const Quaternion q = normalize_quaternion(input);
    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;
    const double xy = q.x * q.y;
    const double xz = q.x * q.z;
    const double yz = q.y * q.z;
    const double wx = q.w * q.x;
    const double wy = q.w * q.y;
    const double wz = q.w * q.z;

    return Matrix3{{
        {{1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)}},
        {{2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)}},
        {{2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)}}
    }};
} // rotation_from_quaternion

double wrap_pi(double angle){
    return std::atan2(std::sin(angle), std::cos(angle));
}  

// 直接连乘前 joint_count 节的 DH 矩阵，不在内部重复添加偏置。
Matrix4 make_dh_transform(const DhAngles& theta, std::size_t joint_count){
    Matrix4 transform = identity_matrix();

    for (std::size_t i = 0; i < joint_count; ++i) {
        const double alpha = kAlphaDeg[i] * kDegToRad;
        transform = multiply(
            transform,
            make_standard_dh(theta[i], kD[i], kA[i], alpha)
        );
    }

    return transform;
} // make_dh_transform

std::array<double, 3> wrist_center(const Pose& pose, const Matrix3& target_rotation){
    // pose 给的是 TCP 在 base 中的位置, 腕心位置 = TCP位置 - d6 * 工具z轴方向
    // p_wrist = p_tool - d6 * z_tool；R 的第三列是工具 z 轴在 base 中的方向。
    return {
        pose.x_m - kD[5] * target_rotation[0][2],
        pose.y_m - kD[5] * target_rotation[1][2],
        pose.z_m - kD[5] * target_rotation[2][2]
    };
}  // wrist_center  

bool arm_position_matches(const ArmTheta& arm_theta, const std::array<double, 3>& target_wrist){
    // 候选的前三轴代回 DH 矩阵后，是否确实到达腕心
    DhAngles theta{};
    theta[0] = arm_theta[0];
    theta[1] = arm_theta[1];
    theta[2] = arm_theta[2];
    // A4 的平移沿 z3，且 a4=0，因此 theta4 不影响腕心位置，可暂取 0。
    const Matrix4 transform = make_dh_transform(theta, 4);

    const double dx = transform[0][3] - target_wrist[0];
    const double dy = transform[1][3] - target_wrist[1];
    const double dz = transform[2][3] - target_wrist[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz)
        <= kArmPositionCheckToleranceM;
} // arm_position_matches

std::vector<ArmTheta> solve_arm_theta(const std::array<double, 3>& wrist, double seed_theta1){
    std::vector<ArmTheta> result;

    const double planar_radius = std::hypot(wrist[0], wrist[1]); // 计算腕心到 J1 竖直轴的水平距离
    std::vector<double> theta1_candidates;

    if (planar_radius < kAxisSingularityToleranceM) {
        // 轴线上先以种子求 theta2/theta3，theta1 的连续可行区间在完整 IK 中处理。
        theta1_candidates.push_back(seed_theta1);
    } else {
        // xw = r*cos(theta1)，yw = r*sin(theta1)。
        // 两个肩部分支分别对应 r>0 和 r<0。
        const double azimuth = std::atan2(wrist[1], wrist[0]);
        theta1_candidates.push_back(wrap_pi(azimuth));
        theta1_candidates.push_back(wrap_pi(azimuth + kPi));
    }

    const double d1 = kD[0];
    const double a2 = kA[1];
    const double d4 = kD[3];

    for (const double theta1 : theta1_candidates) {
        // frame 1原点坐标为(0, 0, d1) 腕心坐标(xw, yw, zw), 从 O1 指向腕心的向量(xw, yw, zw - d1)
        const double vertical = wrist[2] - d1;
        // 将腕心投影到 theta1 对应的水平轴上，直接得到有正负号的 r
        // r 的符号由肩部分支确定，不再独立枚举 radial_sign。
        // 投影到的基座表轴为((cos(theta1), sin(theta1), 0), (0, 0, 1))这一组, 垂直高度vertical直接可以得到
        // 另一轴需要进行投影 (xw, yw, zw - d1) · (cos(theta1), sin(theta1), 0)
        const double radial = wrist[0] * std::cos(theta1) + wrist[1] * std::sin(theta1);

        // 直接使用 DH 角展开腕心位置：
        // r = a2*cos(theta2) + d4*sin(theta2+theta3)
        // h = a2*sin(theta2) - d4*cos(theta2+theta3)
        // a2 的方向角是 theta2，d4 的方向角是 theta2+theta3-pi/2。
        // 这里的 pi/2 来自 alpha3 的固定轴扭转，不是工程零位偏置。
        // 两段正向连杆之间的有符号转角 delta = theta3 - pi/2。
        // 因而 L² = a2² + d4² + 2*a2*d4*cos(delta)。
        double cos_delta = (radial*radial + vertical*vertical - a2*a2 - d4*d4) / (2.0 * a2 * d4);

        // 仅修正浮点误差；明显超出 [-1,1] 的腕心位置不可达。
        if (cos_delta < -1.0 - 1e-8 || cos_delta > 1.0 + 1e-8) {
            continue;
        }
        cos_delta = std::clamp(cos_delta, -1.0, 1.0);
        const double sin_magnitude = std::sqrt(std::max(0.0, 1.0 - cos_delta * cos_delta));

        for (const double elbow_sign : {1.0, -1.0}) {
            const double delta = std::atan2(elbow_sign * sin_magnitude, cos_delta); 
            const double theta3 = wrap_pi(delta + kPi / 2.0); // delta = q3 - 90; delta有两个，带动q3有两个，不用单独wrap_pi(q3+pi)

            // phi 是腕心方向；gamma 是合成位移相对第一段连杆的偏转。
            // 第一段连杆方向直接就是 theta2，因此 theta2 = phi - gamma。
            const double phi = std::atan2(vertical, radial);
            const double gamma = std::atan2(d4 * std::sin(delta), a2 + d4 * std::cos(delta));
            const double theta2 = wrap_pi(phi - gamma);

            const ArmTheta candidate{theta1, theta2, theta3};
            if (!arm_position_matches(candidate, wrist)) {
                continue;
            }

            // 不在这里按角度去重：近奇异时两个退化根可能只差极小角度，
            // 但其腕部候选对限位的影响不同。让完整 IK 先做限位和 FK 校验。
            result.push_back(candidate);
        } // for
    } // for 

    return result;
} // solve_arm_theta

struct WristPairTheta {
    double theta4_rad;
    double theta6_rad;
};

std::array<double, 2> search_limits(
    std::size_t joint, const JointAngles& seed, bool enforce_step
) {
    auto limits = kModelJointLimitsDeg[joint];
    if (enforce_step) {
        limits[0] = std::max(limits[0], seed[joint] - kMaxJointStepDeg);
        limits[1] = std::min(limits[1], seed[joint] + kMaxJointStepDeg);
    }
    return limits;
}

// 腕部奇异时，旋转矩阵只给出 theta4+theta6 或 theta4-theta6。
// 在模型关节角软限位内分配这组和/差，并选离上一组关节角最近的解；
// 返回值再转回 DH theta，保持 IK 内部的角度表示统一。
std::optional<WristPairTheta> closest_singular_wrist_pair(
    double constrained_sum_or_difference_rad,
    bool is_sum_constraint,
    const JointAngles& seed_model_deg,
    bool enforce_step,
    std::vector<WristPairTheta>* all_pairs = nullptr
) {
    constexpr std::size_t kJ4 = 3;
    constexpr std::size_t kJ6 = 5;
    const double j4_offset = kDhThetaOffsetDeg[kJ4];
    const double j6_offset = kDhThetaOffsetDeg[kJ6];
    const auto j4_limits = search_limits(kJ4, seed_model_deg, enforce_step);
    const auto j6_limits = search_limits(kJ6, seed_model_deg, enforce_step);
    const double principal_deg =
        constrained_sum_or_difference_rad * kRadToDeg;

    bool found = false;
    double best_distance = 0.0;
    double best_q4 = 0.0;
    double best_q6 = 0.0;

    // 角度关系按整圈等价；检查足够邻近的 360° 周期表示。
    for (int turns = -2; turns <= 2; ++turns) {
        double q4_low = j4_limits[0];
        double q4_high = j4_limits[1];
        double relation_deg = principal_deg + 360.0 * turns;

        if (is_sum_constraint) {
            // theta4+theta6 = relation；theta=q+offset。
            relation_deg -= j4_offset + j6_offset;
            q4_low = std::max(
                q4_low,
                relation_deg - j6_limits[1]
            );
            q4_high = std::min(
                q4_high,
                relation_deg - j6_limits[0]
            );
        } else {
            // theta4-theta6 = relation；theta=q+offset。
            relation_deg += -j4_offset + j6_offset;
            q4_low = std::max(
                q4_low,
                relation_deg + j6_limits[0]
            );
            q4_high = std::min(
                q4_high,
                relation_deg + j6_limits[1]
            );
        }

        if (q4_low > q4_high) {
            if (q4_low - q4_high > kJointLimitRoundoffDeg) {
                continue;
            }
            // 交集仅因浮点舍入出现极小倒置时收敛到共同边界，保证 clamp 区间有效。
            const double boundary = 0.5 * (q4_low + q4_high);
            q4_low = boundary;
            q4_high = boundary;
        }

        // 在线性约束线上求离 seed (q4,q6) 最近的点，再投影到可行区间。
        const double unconstrained_q4 = is_sum_constraint
            ? (seed_model_deg[kJ4] + relation_deg
               - seed_model_deg[kJ6]) * 0.5
            : (seed_model_deg[kJ4] + relation_deg
               + seed_model_deg[kJ6]) * 0.5;
        const double q4 = std::clamp(unconstrained_q4, q4_low, q4_high);
        const double q6 = is_sum_constraint
            ? relation_deg - q4
            : q4 - relation_deg;

        const double dq4 = q4 - seed_model_deg[kJ4];
        const double dq6 = q6 - seed_model_deg[kJ6];
        const double distance = dq4 * dq4 + dq6 * dq6;

        if (all_pairs) {
            all_pairs->push_back({(q4 + j4_offset) * kDegToRad, (q6 + j6_offset) * kDegToRad});
        }

        if (!found || distance < best_distance) {
            found = true;
            best_distance = distance;
            best_q4 = q4;
            best_q6 = q6;
        }
    }

    if (!found) {
        return std::nullopt;
    }

    return WristPairTheta{
        (best_q4 + j4_offset) * kDegToRad,
        (best_q6 + j6_offset) * kDegToRad
    };
}

std::vector<WristTheta> solve_wrist_theta(
    const Matrix3& wrist_rotation,
    const JointAngles& seed_model_deg,
    bool enforce_step,
    bool all_candidates = false
) {
    std::vector<WristTheta> result;
    const double sin_theta5_magnitude =
        std::hypot(wrist_rotation[0][2], wrist_rotation[1][2]);

    if (sin_theta5_magnitude > kWristSingularitySinTolerance) {
        // R36 = Rz(theta4) * Ry(theta5) * Rz(theta6)。
        // hypot 得到 |sin(theta5)|，所以 atan2 先选 sin(theta5)>=0 的主解，
        // theta5_principal 落在 [0, pi]
        // 此处三个角都是 DH theta，之后统一转换成模型关节角。
        const double theta5_principal = std::atan2(
            sin_theta5_magnitude,
            std::clamp(wrist_rotation[2][2], -1.0, 1.0)
        );
        const double theta4_principal = std::atan2(
            wrist_rotation[1][2],
            wrist_rotation[0][2]
        );
        const double theta6_principal = std::atan2(
            wrist_rotation[2][1],
            -wrist_rotation[2][0]
        );

        // 第一组：theta5 取 [0, pi] 的主解。
        result.push_back(WristTheta{
            wrap_pi(theta4_principal),
            wrap_pi(theta5_principal),
            wrap_pi(theta6_principal)
        });

        // 第二组腕部支：三个角按此关系变化后，R36 保持不变：
        // (theta4 + pi, -theta5, theta6 + pi)。
        result.push_back(WristTheta{
            wrap_pi(theta4_principal + kPi),
            wrap_pi(-theta5_principal),
            wrap_pi(theta6_principal + kPi)
        });
    } else if (wrist_rotation[2][2] >= 0.0) {
        // theta5=0（腕部奇异）：R36 只能确定 theta4+theta6。
        const double combined = std::atan2(
            wrist_rotation[1][0],
            wrist_rotation[0][0]
        );
        std::vector<WristPairTheta> pairs;
        const auto pair = closest_singular_wrist_pair(
            combined, true, seed_model_deg, enforce_step, all_candidates ? &pairs : nullptr
        );
        if (!all_candidates && pair) pairs.push_back(*pair);
        for (const auto& p : pairs) {
            result.push_back(WristTheta{
                p.theta4_rad,
                0.0,
                p.theta6_rad
            });
        }
    } else {
        // theta5=pi（腕部奇异）：R36 只能确定 theta4-theta6。
        const double difference = std::atan2(
            -wrist_rotation[1][0],
            -wrist_rotation[0][0]
        );
        std::vector<WristPairTheta> pairs;
        const auto pair = closest_singular_wrist_pair(
            difference, false, seed_model_deg, enforce_step, all_candidates ? &pairs : nullptr
        );
        if (!all_candidates && pair) pairs.push_back(*pair);
        for (const auto& p : pairs) {
            result.push_back(WristTheta{
                p.theta4_rad,
                kPi,
                p.theta6_rad
            });
        }
    }

    return result;
} // solve_wrist_theta

bool choose_model_angle(double theta_rad, std::size_t joint, double seed_deg, double& result_deg){
    // 六轴统一转换：模型关节角 q = DH角 theta（度）- 该轴零位偏置。
    const double base_deg = theta_rad * kRadToDeg - kDhThetaOffsetDeg[joint];
    if (!std::isfinite(base_deg)) {
        return false;
    }

    bool found = false;
    double best_distance = 0.0;

    // 同一旋转角可能相差整圈；只保留模型关节范围内且最接近种子的值。
    for (int turns = -2; turns <= 2; ++turns) {
        double candidate = base_deg + 360.0 * turns;
        if (candidate < kModelJointLimitsDeg[joint][0] - kJointLimitRoundoffDeg
            || candidate > kModelJointLimitsDeg[joint][1] + kJointLimitRoundoffDeg) {
            continue;
        }
        candidate = std::clamp(
            candidate,
            kModelJointLimitsDeg[joint][0],
            kModelJointLimitsDeg[joint][1]
        );

        const double distance = std::abs(candidate - seed_deg);
        if (!found || distance < best_distance) {
            found = true;
            best_distance = distance;
            result_deg = candidate;
        }
    }

    return found;
} // choose_model_angle

// theta2/theta3 固定时，R36 的每个元素都是 a*cos(q1)+b*sin(q1)+c。
// 用腕部角度约束的解析交点划分 J1 区间，极窄区间也不会落在固定网格之间。
struct AxisSinusoid {
    double a, b, c;
};

std::vector<double> axis_search_boundaries(
    const ArmTheta& arm, const Matrix3& target_rotation,
    const JointAngles& seed, bool enforce_step
) {
    const auto rotation_at = [&](double q1_deg) {
        const DhAngles theta{(q1_deg + kDhThetaOffsetDeg[0]) * kDegToRad,
                             arm[1], arm[2], 0, 0, 0};
        return multiply(transpose(rotation_part(make_dh_transform(theta, 3))), target_rotation);
    };
    const auto r0 = rotation_at(0);
    const auto r90 = rotation_at(90);
    const auto r180 = rotation_at(180);
    std::array<std::array<AxisSinusoid, 3>, 3> coefficients{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
            const double c = (r0[row][col] + r180[row][col]) * 0.5;
            coefficients[row][col] = {(r0[row][col] - r180[row][col]) * 0.5,
                                     r90[row][col] - c, c};
        }
    }
    const auto j1_limits = search_limits(0, seed, enforce_step);
    std::vector<double> points{j1_limits[0], j1_limits[1],
                              std::clamp(seed[0], j1_limits[0], j1_limits[1])};
    const auto add_roots = [&](AxisSinusoid f) {
        const double magnitude = std::hypot(f.a, f.b);
        if (magnitude < 1e-14) return; // 常量约束不产生区间交点。
        const double cosine = -f.c / magnitude;
        if (cosine < -1.0 - 1e-12 || cosine > 1.0 + 1e-12) return;
        const double phase = std::atan2(f.b, f.a);
        const double delta = std::acos(std::clamp(cosine, -1.0, 1.0));
        for (double angle : {phase - delta, phase + delta}) {
            const double principal = angle * kRadToDeg;
            const int first = static_cast<int>(std::ceil((j1_limits[0] - principal) / 360.0));
            const int last = static_cast<int>(std::floor((j1_limits[1] - principal) / 360.0));
            for (int turn = first; turn <= last; ++turn) {
                points.push_back(std::clamp(principal + 360.0 * turn, j1_limits[0], j1_limits[1]));
            }
        }
    };
    // atan2(y,x) 等于指定射线角（模 pi）的边界；同时覆盖腕部正/反两支。
    const auto add_ray = [&](AxisSinusoid x, AxisSinusoid y, double angle_deg) {
        const double c = std::cos(angle_deg * kDegToRad);
        const double s = std::sin(angle_deg * kDegToRad);
        add_roots({y.a*c - x.a*s, y.b*c - x.b*s, y.c*c - x.c*s});
    };
    const auto negative = [](AxisSinusoid f) { return AxisSinusoid{-f.a, -f.b, -f.c}; };
    const auto x4 = coefficients[0][2], y4 = coefficients[1][2];
    const auto x6 = negative(coefficients[2][0]), y6 = coefficients[2][1];
    for (std::size_t joint : {std::size_t{3}, std::size_t{5}}) {
        const auto limits = search_limits(joint, seed, enforce_step);
        for (double q : {limits[0], limits[1], seed[joint]}) {
            add_ray(joint == 3 ? x4 : x6, joint == 3 ? y4 : y6,
                    q + kDhThetaOffsetDeg[joint]);
        }
        add_ray(joint == 3 ? x4 : x6, joint == 3 ? y4 : y6, 0); // atan2 分支切线。
    }
    const auto z = coefficients[2][2]; // cos(theta5)
    const auto add_cosine = [&](double value) { add_roots({z.a, z.b, z.c - value}); };
    const auto j5_limits = search_limits(4, seed, enforce_step);
    for (double q5 : {j5_limits[0], j5_limits[1], seed[4]}) {
        add_cosine(std::cos((q5 + kDhThetaOffsetDeg[4]) * kDegToRad));
    }
    add_cosine(1.0);
    add_cosine(-1.0);
    const double singular_cos = std::sqrt(1.0 - kWristSingularitySinTolerance*kWristSingularitySinTolerance);
    add_cosine(singular_cos);
    add_cosine(-singular_cos);
    add_roots({z.b, -z.a, 0.0}); // theta5 的极值：避免近奇异区间被浮点舍入吞掉。

    // 全段腕奇异时，只有 theta4±theta6 可确定；加入这组和/差的可行边界。
    const auto l4 = search_limits(3, seed, enforce_step);
    const auto l6 = search_limits(5, seed, enforce_step);
    for (double sign : {-1.0, 1.0}) {
        for (double q4 : {l4[0], l4[1], seed[3]}) {
            for (double q6 : {l6[0], l6[1], seed[5]}) {
                add_ray(coefficients[0][0], coefficients[1][0],
                        q4 + kDhThetaOffsetDeg[3] + sign*(q6 + kDhThetaOffsetDeg[5]));
            }
        }
    }
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end(), [](double a, double b) {
        return std::abs(a - b) < 1e-11;
    }), points.end());
    return points;
}

double orientation_error_rad(const Quaternion& target, const Quaternion& actual){
    const Quaternion error = multiply_quaternion(
        normalize_quaternion(target),
        conjugate(normalize_quaternion(actual))
    );
    const Quaternion q = normalize_quaternion(error);
    const double vector_norm =
        std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    return 2.0 * std::atan2(vector_norm, std::abs(q.w));
} // orientation_error_rad

}  // namespace

Quaternion quaternion_from_rpy(double roll_deg, double pitch_deg, double yaw_deg){
    const double half_roll = roll_deg * kDegToRad * 0.5;
    const double half_pitch = pitch_deg * kDegToRad * 0.5;
    const double half_yaw = yaw_deg * kDegToRad * 0.5;

    const double cr = std::cos(half_roll);
    const double sr = std::sin(half_roll);
    const double cp = std::cos(half_pitch);
    const double sp = std::sin(half_pitch);
    const double cy = std::cos(half_yaw);
    const double sy = std::sin(half_yaw);

    // 当前工程 RPY 约定：R = Rz(yaw) * Ry(pitch) * Rx(roll)。
    return normalize_quaternion(Quaternion{
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    });
} // quaternion_from_rpy

Quaternion quaternion_from_studio_rpy(
    double roll_deg,
    double pitch_deg,
    double yaw_deg
) {
    const double half_roll = roll_deg * kDegToRad * 0.5;
    const double half_pitch = pitch_deg * kDegToRad * 0.5;
    const double half_yaw = yaw_deg * kDegToRad * 0.5;

    const Quaternion rotation_x{
        std::cos(half_roll), std::sin(half_roll), 0.0, 0.0
    };
    const Quaternion rotation_y{
        std::cos(half_pitch), 0.0, std::sin(half_pitch), 0.0
    };
    const Quaternion rotation_z{
        std::cos(half_yaw), 0.0, 0.0, std::sin(half_yaw)
    };

    // Studio/Three.js Euler XYZ 按矩阵乘法约定对应 Rx * Ry * Rz。
    return normalize_quaternion(
        multiply_quaternion(
            multiply_quaternion(rotation_x, rotation_y),
            rotation_z
        )
    );
} // quaternion_from_studio_rpy

std::array<double, 3> studio_rpy_from_quaternion(
    const Quaternion& orientation
) {
    const Matrix3 rotation = rotation_from_quaternion(orientation);
    const double pitch_rad = std::asin(
        std::clamp(rotation[0][2], -1.0, 1.0)
    );
    const double cos_pitch = std::cos(pitch_rad);

    double roll_rad = 0.0;
    double yaw_rad = 0.0;
    if (std::abs(cos_pitch) > 1e-9) {
        roll_rad = std::atan2(-rotation[1][2], rotation[2][2]);
        yaw_rad = std::atan2(-rotation[0][1], rotation[0][0]);
    } else {
        // 万向节锁附近固定 roll=0，保留一个等价的 yaw。
        yaw_rad = std::atan2(rotation[1][0], rotation[1][1]);
    }

    return {
        roll_rad * kRadToDeg,
        pitch_rad * kRadToDeg,
        yaw_rad * kRadToDeg
    };
} // studio_rpy_from_quaternion

Quaternion slerp(const Quaternion& start, const Quaternion& target, double t){ 
    // 姿态使用 quaternion slerp 插值: 四元数球面线性插值
    
    Quaternion a = normalize_quaternion(start);
    Quaternion b = normalize_quaternion(target);
    t = std::clamp(t, 0.0, 1.0);

    double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    if (dot < 0.0) {
        b.w = -b.w;
        b.x = -b.x;
        b.y = -b.y;
        b.z = -b.z;
        dot = -dot;
    }
    dot = std::clamp(dot, -1.0, 1.0);

    if (dot > 0.9995) {
        return normalize_quaternion(Quaternion{
            a.w + t * (b.w - a.w),
            a.x + t * (b.x - a.x),
            a.y + t * (b.y - a.y),
            a.z + t * (b.z - a.z)
        });
    }

    const double angle = std::acos(dot);
    const double sin_angle = std::sin(angle);
    const double weight_a = std::sin((1.0 - t) * angle) / sin_angle;
    const double weight_b = std::sin(t * angle) / sin_angle;

    return normalize_quaternion(Quaternion{
        weight_a * a.w + weight_b * b.w,
        weight_a * a.x + weight_b * b.x,
        weight_a * a.y + weight_b * b.y,
        weight_a * a.z + weight_b * b.z
    });
} // slerp

Pose forward_kinematics(const JointAngles& joints){
    // 模型角只在入口换算一次，此后的矩阵运算全部使用 DH 角。
    const DhAngles theta = to_dh_angles(joints);
    const Matrix4 transform = make_dh_transform(theta, theta.size());

    return Pose{
        transform[0][3],
        transform[1][3],
        transform[2][3],
        quaternion_from_rotation_matrix(rotation_part(transform))
    };
} // forward_kinematics

std::array<double, 3> wrist_center_position(const Pose& pose) {
    if (!std::isfinite(pose.x_m) || !std::isfinite(pose.y_m) || !std::isfinite(pose.z_m)) {
        throw std::invalid_argument("腕心计算要求有限位置");
    }
    return wrist_center(pose, rotation_from_quaternion(pose.orientation));
}

namespace {

std::optional<JointAngles> solve_inverse_kinematics(
    const Pose& pose, const JointAngles& previous_solution, bool enforce_joint_step,
    std::vector<JointAngles>* candidates = nullptr
) {
    if (!std::isfinite(pose.x_m)
        || !std::isfinite(pose.y_m)
        || !std::isfinite(pose.z_m)) {
        throw std::invalid_argument("IK 目标位置必须是有限数值");
    }

    for (std::size_t i = 0; i < previous_solution.size(); ++i) {
        if (!std::isfinite(previous_solution[i])) {
            throw std::invalid_argument("IK 初始关节角必须是有限数值");
        }
        // previous_solution 是公开接口的模型关节角，不是 DH theta。
        if (previous_solution[i] < kModelJointLimitsDeg[i][0]
            || previous_solution[i] > kModelJointLimitsDeg[i][1]) {
            return std::nullopt;
        }
    }

    const Matrix3 target_rotation =
        rotation_from_quaternion(pose.orientation);
    const auto wrist = wrist_center(pose, target_rotation);

    // 上一组模型角在入口统一转换为 theta，供奇异情况下的连续选解使用。
    const DhAngles seed_theta = to_dh_angles(previous_solution);

    const auto arm_candidates = solve_arm_theta(wrist, seed_theta[0]);
    if (arm_candidates.empty()) {
        return std::nullopt;
    }

    bool found_solution = false;
    double best_distance = 0.0;
    JointAngles best_solution{};

    const bool on_axis = std::hypot(wrist[0], wrist[1]) < kAxisSingularityToleranceM;
    for (const auto& arm_reference : arm_candidates) {
        const auto evaluate = [&](double q1_deg, int wrist_branch = -1) {
            ArmTheta arm_theta = arm_reference;
            arm_theta[0] = (q1_deg + kDhThetaOffsetDeg[0]) * kDegToRad;
            double local_distance = std::numeric_limits<double>::infinity();
            DhAngles partial_theta{};
            partial_theta[0] = arm_theta[0];
            partial_theta[1] = arm_theta[1];
            partial_theta[2] = arm_theta[2];

            const Matrix4 transform_03 = make_dh_transform(partial_theta, 3);
            const Matrix3 rotation_03 = rotation_part(transform_03);
            const Matrix3 rotation_36 = multiply(
                transpose(rotation_03),
                target_rotation
            );

            const auto wrist_candidates =
                solve_wrist_theta(rotation_36, previous_solution, enforce_joint_step, candidates != nullptr);

            for (std::size_t wrist_index=0;wrist_index<wrist_candidates.size();++wrist_index) {
                if (wrist_branch >= 0 && wrist_index != static_cast<std::size_t>(wrist_branch)) continue;
                const auto& wrist_theta = wrist_candidates[wrist_index];
                const DhAngles theta{
                    arm_theta[0], arm_theta[1], arm_theta[2],
                    wrist_theta[0], wrist_theta[1], wrist_theta[2]
                };

                // 六个 DH 角已经全部求出，再统一转回模型角并按模型软限位筛选。
                JointAngles candidate{};
                bool within_limits = true;
                for (std::size_t joint = 0; joint < candidate.size(); ++joint) {
                    if (!choose_model_angle(
                            theta[joint],
                            joint,
                            previous_solution[joint],
                            candidate[joint])) {
                        within_limits = false;
                        break;
                    }
                }
                if (!within_limits) {
                    continue;
                }

                // 在选最近解之前逐候选筛选，避免最近解越过单轴阈值，
                // 却遗漏总距离稍远、各轴步长都合格的另一组解。
                bool within_step = true;
                double distance = 0.0;
                double maximum_step = 0.0;
                for (std::size_t joint = 0; joint < candidate.size(); ++joint) {
                    const double difference = candidate[joint] - previous_solution[joint];
                    // 模型角与设备角的差仅差一个 ±1 符号，绝对步长相同；不做 ±180° 环绕。
                    if (enforce_joint_step && std::abs(difference) > kMaxJointStepDeg) {
                        within_step = false;
                        break;
                    }
                    distance += difference * difference;
                    maximum_step = std::max(maximum_step, std::abs(difference));
                }
                if (!within_step) {
                    continue;
                }

                // 解析公式可能在奇异或边界处产生退化根；用同一 FK 检查位置和姿态残差。
                const Pose achieved = forward_kinematics(candidate);
                const double dx = achieved.x_m - pose.x_m;
                const double dy = achieved.y_m - pose.y_m;
                const double dz = achieved.z_m - pose.z_m;
                const double position_error =
                    std::sqrt(dx * dx + dy * dy + dz * dz);
                const double orientation_error = orientation_error_rad(
                    pose.orientation,
                    achieved.orientation
                );

                if (position_error > kIkPositionToleranceM
                    || orientation_error > kIkOrientationToleranceRad) {
                    continue;
                }

                if (candidates) {
                    const bool duplicate = std::any_of(candidates->begin(), candidates->end(),
                        [&](const JointAngles& existing) {
                            for (std::size_t j = 0; j < candidate.size(); ++j) {
                                if (std::abs(existing[j] - candidate[j]) > 1e-8) return false;
                            }
                            return true;
                        });
                    if (!duplicate) candidates->push_back(candidate);
                }
                // MOVEL 的轴线自由度按最大单轴变化细化，旧单解 API 保留平方和排序。
                local_distance = std::min(local_distance, candidates ? maximum_step : distance);
                if (!found_solution || distance < best_distance) {
                    found_solution = true;
                    best_distance = distance;
                    best_solution = candidate;
                }
            }

            return local_distance;
        };
        if (!on_axis) {
            evaluate(arm_reference[0] * kRadToDeg - kDhThetaOffsetDeg[0]);
            continue;
        }

        const auto boundaries = axis_search_boundaries(
            arm_reference, target_rotation, previous_solution, enforce_joint_step);
        for (double q1 : boundaries) evaluate(q1); // 包括孤立可行点和精确腕奇异点。
        for (std::size_t i = 1; i < boundaries.size(); ++i) {
            const double low = boundaries[i - 1], high = boundaries[i];
            if (high - low < 1e-10) continue;
            // 约束交点之间，非奇异腕支的可行性不变；先验证区间内部。
            if (!std::isfinite(evaluate(0.5 * (low + high)))) continue;
            // 同一腕支的角差目标也可能多峰，不能对整个区间直接假设单峰。
            // 在约束/奇异边界附近按比例细分，分别寻找各腕支的局部低谷再优化。
            // 可行区间仍由解析交点划分；这里的节点只用于距离优化，不替代可行性检查。
            std::vector<double> knots{low, high, (low+high)*0.5};
            for (int depth=2;depth<=16;++depth) {
                const double offset=std::ldexp(high-low,-depth);
                knots.push_back(low+offset);
                knots.push_back(high-offset);
            }
            std::sort(knots.begin(),knots.end());
            knots.erase(std::unique(knots.begin(),knots.end(),[](double a,double b) {
                return std::abs(a-b)<1e-11;
            }),knots.end());
            for (int wrist_branch=0;wrist_branch<2;++wrist_branch) {
                std::vector<double> scores;
                for (double q1 : knots) scores.push_back(evaluate(q1,wrist_branch));
                for (std::size_t k=1;k+1<knots.size();++k) {
                    if (!std::isfinite(scores[k]) || scores[k]>scores[k-1] || scores[k]>scores[k+1]
                        || (scores[k]>=scores[k-1]-1e-12 && scores[k]>=scores[k+1]-1e-12)) continue;
                    double a=knots[k-1], b=knots[k+1];
                    constexpr double golden=0.6180339887498948482;
                    double c=b-golden*(b-a), d=a+golden*(b-a);
                    double fc=evaluate(c,wrist_branch), fd=evaluate(d,wrist_branch);
                    for (int iteration=0;iteration<48 && b-a>1e-10;++iteration) {
                        if (fc<fd) {
                            b=d; d=c; fd=fc;
                            c=b-golden*(b-a); fc=evaluate(c,wrist_branch);
                        } else {
                            a=c; c=d; fc=fd;
                            d=a+golden*(b-a); fd=evaluate(d,wrist_branch);
                        }
                    }
                }
            }
        }
    }

    if (!found_solution) {
        return std::nullopt;
    }
    return best_solution;
}  // solve_inverse_kinematics

} // namespace

std::vector<JointAngles> inverse_kinematics_candidates(const Pose& pose, const JointAngles& reference) {
    std::vector<JointAngles> candidates;
    solve_inverse_kinematics(pose, reference, false, &candidates);
    return candidates;
}

std::optional<JointAngles> inverse_kinematics(const Pose& pose, const JointAngles& previous_solution) {
    return solve_inverse_kinematics(pose, previous_solution, true);
}

std::optional<JointAngles> inverse_kinematics(const Pose& pose) {
    // 首次求解没有轨迹历史；用模型零位作为确定性选支参考。
    return solve_inverse_kinematics(pose, JointAngles{}, false);
}

}  // namespace kinematics
