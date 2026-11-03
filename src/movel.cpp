#include "movel.hpp"
#include "joint_mapping.hpp"
#include "robot_reader.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace movel{

namespace{
// Studio 的 SpeedFollow 控制周期为 20 ms
constexpr double kControlPeriod = 0.020; // 控制周期，单位：秒. 代表也就是每 20 ms 计算一次速度并发送一轮六轴命令
// 最大单关节步长沿用 kinematics.hpp 的当前配置（与单解 IK 共用）。
constexpr double kMaxJointStep = kinematics::kMaxJointStepDeg;
constexpr double kSingularityToleranceDeg = 1.0;
constexpr double kSingularityEscapeEpsilonDeg = 1e-10;
// 速度寄存器 0x002F 使用的命令类型
using SpeedCommands = std::array<std::int32_t, 6>;

std::ofstream g_csv_log;

std::string csv_escape(const std::string& value) {
    std::string result = "\"";
    for (const char character : value) {
        if (character == '"') result += '"';
        result += character;
    }
    result += '"';
    return result;
}

std::string csv_timestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    );
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(milliseconds);
    const auto fraction = (milliseconds - seconds).count();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
    ::gmtime_r(&time, &utc);

    std::ostringstream output;
    output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S")
           << '.' << std::setw(3) << std::setfill('0') << fraction << 'Z';
    return output.str();
}

std::array<double, 3> pose_rpy(const kinematics::Pose& pose) {
    return kinematics::studio_rpy_from_quaternion(pose.orientation);
}

void append_pose_csv(std::ostream& output, const kinematics::Pose& pose) {
    const auto rpy = pose_rpy(pose);
    output << ',' << pose.x_m << ',' << pose.y_m << ',' << pose.z_m
           << ',' << rpy[0] << ',' << rpy[1] << ',' << rpy[2];
}

void append_joint_csv(
    std::ostream& output,
    const std::optional<joint_mapping::JointAngles>& values
) {
    for (std::size_t i = 0; i < 6; ++i) {
        output << ',';
        if (values) output << (*values)[i];
    }
}

void write_csv_record(
    const char* record_type,
    std::size_t index,
    const char* stage,
    const std::optional<kinematics::Pose>& current_pose,
    const std::optional<kinematics::Pose>& target_pose,
    const std::optional<joint_mapping::JointAngles>& actual,
    const std::optional<joint_mapping::JointAngles>& reference,
    const std::optional<SpeedCommands>& commands,
    const std::optional<double>& dt_seconds,
    const std::optional<double>& position_error_mm,
    const std::optional<double>& orientation_error_deg,
    const std::optional<double>& max_joint_error_deg,
    const std::optional<int>& running_count,
    const std::string& detail
) {
    if (!g_csv_log.is_open()) return;

    g_csv_log << std::setprecision(std::numeric_limits<double>::max_digits10)
              << csv_timestamp() << ',' << record_type << ',' << index << ','
              << stage;
    if (current_pose) append_pose_csv(g_csv_log, *current_pose);
    else g_csv_log << ",,,,,,";
    if (target_pose) append_pose_csv(g_csv_log, *target_pose);
    else g_csv_log << ",,,,,,";
    append_joint_csv(g_csv_log, actual);
    append_joint_csv(g_csv_log, reference);

    for (std::size_t i = 0; i < 6; ++i) {
        g_csv_log << ',';
        if (commands) g_csv_log << (*commands)[i];
    }

    g_csv_log << ',';
    if (dt_seconds) g_csv_log << *dt_seconds;
    g_csv_log << ',';
    if (position_error_mm) g_csv_log << *position_error_mm;
    g_csv_log << ',';
    if (orientation_error_deg) g_csv_log << *orientation_error_deg;
    g_csv_log << ',';
    if (max_joint_error_deg) g_csv_log << *max_joint_error_deg;
    g_csv_log << ',';
    if (running_count) g_csv_log << *running_count;
    g_csv_log << ',' << csv_escape(detail) << '\n';
    g_csv_log.flush();
}

/*
// 以下两个函数曾用于对 RPY 姿态角做逐轴插值。
// MOVEL 姿态已改为四元数 SLERP，因此不再调用它们。
// 有限转角关节也不能使用 ±180° 环绕差值，关节误差应直接相减。
double wrap_angle(double angle) {
    angle = std::fmod(angle + 180.0, 360.0);
    if (angle < 0.0) {
        angle += 360.0;
    }
    return angle - 180.0;
}

double shortest_angle_difference(double target, double current) {
    return wrap_angle(target - current);
}
*/

// 五次平滑曲线。
// 起点和终点的速度、加速度都为 0
double smooth_step(double u) {
    // 五次平滑曲线，起点和终点速度、加速度为 0
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
} // smooth_step

// 对两个笛卡尔位姿进行插值
kinematics::Pose interpolate_pose(const kinematics::Pose& start, const kinematics::Pose& target, double u) {

    const double s = smooth_step(u);
    return kinematics::Pose{
        start.x_m + (target.x_m - start.x_m) * s,
        start.y_m + (target.y_m - start.y_m) * s,
        start.z_m + (target.z_m - start.z_m) * s,
        kinematics::slerp(start.orientation, target.orientation, s)
    };
} // interpolate_pose：位置平滑插值，姿态使用四元数 SLERP

// 生成 MOVEL 的笛卡尔轨迹点
std::vector<kinematics::Pose> sample_movel(
    const kinematics::Pose& start,
    const kinematics::Pose& target,
    double duration_seconds
) {
    if (!std::isfinite(duration_seconds) || duration_seconds<0.5) throw std::invalid_argument("MOVEL 时间不能小于 0.5 秒");
    const auto steps = static_cast<std::size_t>(
        std::ceil(duration_seconds/kControlPeriod)
    );

    std::vector<kinematics::Pose> result;
    result.reserve(steps+1);
    for (std::size_t i = 0; i <= steps; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(steps);
        result.push_back(interpolate_pose(start, target, u));
    }
    return result;
} // sample_movel

bool inside_limit(double angle, const std::pair<double, double>& limit){
    return angle>=limit.first && angle<=limit.second;
} // inside_limit

void validate_joint_limits(const JointLimits& limits){
    for(const auto& limit: limits){
        if(!std::isfinite(limit.first) || !std::isfinite(limit.second) 
           || limit.first>=limit.second){
            throw std::invalid_argument("关节角度限制无效");
        }
    }
}

void validate_start_angles(const joint_mapping::JointAngles& joints, const JointLimits& limits) {
    validate_joint_limits(limits);
    for (std::size_t j = 0; j < joints.size(); ++j) {
        const auto model_limit = joint_mapping::kDeviceJointLimitsDeg[j];
        if (!std::isfinite(joints[j]) || !inside_limit(joints[j], limits[j])
            || joints[j] < model_limit[0] || joints[j] > model_limit[1]) {
            throw std::invalid_argument("MOVEL 起始设备角超出软件范围");
        }
    }
}

double signed_wrist_radius(const kinematics::Pose& pose, const kinematics::JointAngles& model) {
    const auto wrist = kinematics::wrist_center_position(pose);
    constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
    const double theta1 = (model[0] + kinematics::kDhThetaOffsetDeg[0]) * degrees_to_radians;
    return wrist[0]*std::cos(theta1) + wrist[1]*std::sin(theta1);
}

// 腕心不在 J1 轴线上时，肩部分支由沿 DH x1 方向的有符号半径确定。
// r(q) 是关节角的连续函数；起终点若只能取相反符号，任何连续路径均须 r=0。
// 因此不是加几个中间位姿或放宽单步角度就能得到严格避轴线的路径。
void validate_shoulder_connection(
    const kinematics::Pose& start, const kinematics::JointAngles& start_model,
    const kinematics::Pose& target, const JointLimits& limits
) {
    const double start_radius = signed_wrist_radius(start, start_model);
    const auto target_wrist = kinematics::wrist_center_position(target);
    if (std::abs(start_radius) <= kMinWristAxisDistanceM
        || std::hypot(target_wrist[0], target_wrist[1]) <= kMinWristAxisDistanceM) {
        return; // 起点退出/奇异终点仍交给逐段检查，不能把接近轴线当作新的自由分支。
    }
    const auto targets = kinematics::inverse_kinematics_candidates(target, start_model);
    bool have_legal_target = false;
    double opposite_radius = 0.0;
    for (const auto& q : targets) {
        const auto device = joint_mapping::model_to_device(q);
        bool legal = true;
        for (std::size_t j = 0; j < device.size(); ++j) legal = legal && inside_limit(device[j], limits[j]);
        if (!legal) continue;
        have_legal_target = true;
        const double radius = signed_wrist_radius(target, q);
        if (start_radius * radius > 0.0) return; // 至少存在一个同肩部分支的合法终点。
        opposite_radius = radius;
    }
    if (have_legal_target) {
        std::ostringstream message;
        message << std::fixed << std::setprecision(3)
                << "MOVEL 目标位姿可达，但当前软件限位内的所有目标解均在相反肩部分支"
                << "（起点有符号腕心半径=" << start_radius*1000.0
                << " mm，目标=" << opposite_radius*1000.0 << " mm）。"
                << "从当前关节状态连续到达必须经过 J1 轴线奇异；"
                << "严格避奇异的 MOVEL 及其中间位姿拆分无法完成。"
                << "需要单独规划允许该奇异过渡的关节运动，不能通过增大步长阈值解决。";
        throw std::runtime_error(message.str());
    }
}

std::optional<kinematics::JointAngles> select_minimax_candidate(
    const std::vector<kinematics::JointAngles>& eligible,
    const kinematics::JointAngles& reference
) {
    std::optional<kinematics::JointAngles> selected;
    double best_maximum = std::numeric_limits<double>::infinity();
    double best_squared = std::numeric_limits<double>::infinity();
    for (const auto& candidate : eligible) {
        double maximum = 0.0, squared = 0.0;
        for (std::size_t j = 0; j < candidate.size(); ++j) {
            const double d = candidate[j] - reference[j];
            maximum = std::max(maximum, std::abs(d));
            squared += d*d;
        }
        if (maximum < best_maximum - 1e-10
            || (std::abs(maximum - best_maximum) <= 1e-10 && squared < best_squared)) {
            selected = candidate;
            best_maximum = maximum;
            best_squared = squared;
        }
    }
    return selected;
}

bool continuous_pose_matches(const kinematics::Pose& a, const kinematics::Pose& b) {
    constexpr double position_tolerance_m = 1e-4;
    constexpr double orientation_tolerance_rad = 0.1 * 3.14159265358979323846 / 180.0;
    if (std::hypot(std::hypot(a.x_m-b.x_m,a.y_m-b.y_m),a.z_m-b.z_m) > position_tolerance_m) return false;
    // 用相对四元数的 atan2 计算误差，避免小角度 acos 精度损失。
    const auto x=a.orientation, y=b.orientation;
    const double w=x.w*y.w+x.x*y.x+x.y*y.y+x.z*y.z;
    const double i=-x.w*y.x+x.x*y.w-x.y*y.z+x.z*y.y;
    const double j=-x.w*y.y+x.x*y.z+x.y*y.w-x.z*y.x;
    const double k=-x.w*y.z-x.x*y.y+x.y*y.x+x.z*y.w;
    return 2*std::atan2(std::hypot(std::hypot(i,j),k),std::abs(w)) <= orientation_tolerance_rad;
}

bool continuous_joint_edge(
    const kinematics::JointAngles& before, const kinematics::JointAngles& after,
    const kinematics::Pose& start, const kinematics::Pose& end
) {
    // 末端同位姿不代表关节分支可跳换：检查采样点之间的关节插值是否也跟随
    // 同一笛卡尔位置/姿态路径。有兼容解的轴线穿越可通过，离散的换支会被拒绝。
    for (double t : {0.25, 0.5, 0.75}) {
        auto q = before;
        for (std::size_t j=0;j<q.size();++j) q[j] += t*(after[j]-before[j]);
        const kinematics::Pose expected{
            start.x_m+t*(end.x_m-start.x_m),start.y_m+t*(end.y_m-start.y_m),start.z_m+t*(end.z_m-start.z_m),
            kinematics::slerp(start.orientation,end.orientation,t)};
        if (!continuous_pose_matches(kinematics::forward_kinematics(q),expected)) return false;
    }
    return true;
}

// 将笛卡尔轨迹逐点转换成设备角轨迹；只有 FK/IK 及奇异检查使用模型角。
std::vector<joint_mapping::JointAngles> calculate_joint_trajectory(
    const kinematics::Pose& start_pose,
    const kinematics::Pose& target_pose,
    double duration_seconds,
    const joint_mapping::JointAngles& current_joints,
    const JointLimits& limits,
    const PlanningOptions& options = {}
) {
    validate_start_angles(current_joints, limits);

    const auto cartesian_points = sample_movel(start_pose, target_pose, duration_seconds);
    std::vector<joint_mapping::JointAngles> result;
    result.reserve(cartesian_points.size());

    // IK 种子使用模型角，控制器参考和步长检查使用设备角。
    auto previous_model_solution = joint_mapping::device_to_model(current_joints);
    auto previous_device_joints = current_joints;
    std::optional<std::array<double, 3>> previous_wrist;
    double previous_wrist_singularity_distance =
        std::abs(std::remainder(previous_model_solution[4] - 90.0, 180.0));
    double previous_elbow_singularity_distance =
        std::abs(std::remainder(previous_model_solution[2] - 90.0, 180.0));
    const bool started_at_wrist_singularity =
        previous_wrist_singularity_distance <= kSingularityToleranceDeg;
    const bool started_at_elbow_singularity =
        previous_elbow_singularity_distance <= kSingularityToleranceDeg;

    const auto start_wrist = kinematics::wrist_center_position(start_pose);
    const bool started_on_axis = std::hypot(start_wrist[0], start_wrist[1]) <= kMinWristAxisDistanceM;
    bool escaped_axis = !started_on_axis;

    const auto same_pose = [](const kinematics::Pose& lhs,
                              const kinematics::Pose& rhs) {
        const double position_error = std::hypot(
            std::hypot(lhs.x_m - rhs.x_m, lhs.y_m - rhs.y_m),
            lhs.z_m - rhs.z_m
        );
        const double orientation_dot = std::clamp(
            std::abs(
                lhs.orientation.w * rhs.orientation.w
                + lhs.orientation.x * rhs.orientation.x
                + lhs.orientation.y * rhs.orientation.y
                + lhs.orientation.z * rhs.orientation.z
            ),
            0.0,
            1.0
        );
        return position_error <= 1e-8
            && 2.0 * std::acos(orientation_dot) <= 1e-8;
    };

    if (options.strict_singularity_avoidance
        && (started_at_wrist_singularity || started_at_elbow_singularity)
        && same_pose(start_pose, target_pose)) {
        throw std::runtime_error("MOVEL 路径经过奇异位置");
    }

    for(std::size_t i = 0; i<cartesian_points.size(); ++i){
        const auto wrist = kinematics::wrist_center_position(cartesian_points[i]);
        const double current_axis_distance = std::hypot(wrist[0], wrist[1]);
        double axis_distance = current_axis_distance;
        const bool start_on_axis = i == 0 && started_on_axis;
        const double previous_axis_distance = previous_wrist
            ? std::hypot((*previous_wrist)[0], (*previous_wrist)[1]) : 0.0;
        const bool leaving_start_axis = started_on_axis && !escaped_axis && i > 0
            && current_axis_distance > previous_axis_distance + 1e-12;

        if (previous_wrist) {
            // 检查相邻腕心弦线，避免两采样点分处轴线两侧而漏掉中间穿越。
            const double dx = wrist[0] - (*previous_wrist)[0];
            const double dy = wrist[1] - (*previous_wrist)[1];
            if (!leaving_start_axis) {
                const double length_squared = dx*dx + dy*dy;
                const double t = length_squared > 0.0
                    ? std::clamp(-((*previous_wrist)[0]*dx + (*previous_wrist)[1]*dy)
                                 / length_squared, 0.0, 1.0)
                    : 0.0;
                axis_distance = std::hypot((*previous_wrist)[0] + t*dx,
                                           (*previous_wrist)[1] + t*dy);
                // 姿态插值会使腕心偏离弦线。SLERP 的弦误差上界为 d6*角度²/8，
                // 再留一倍裕量覆盖小角度时的归一化线性插值；距离下界不足则拒绝。
                const auto& a = cartesian_points[i - 1].orientation;
                const auto& b = cartesian_points[i].orientation;
                const double dot = std::clamp(
                    std::abs(a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z),
                    0.0,
                    1.0
                );
                const double angle = 2.0 * std::acos(dot);
                const double tool_offset = std::hypot(
                    std::hypot(cartesian_points[i].x_m - wrist[0],
                               cartesian_points[i].y_m - wrist[1]),
                    cartesian_points[i].z_m - wrist[2]
                );
                axis_distance -= tool_offset * angle * angle / 4.0;
            }
        }
        if (options.strict_singularity_avoidance
            && !start_on_axis && !leaving_start_axis && axis_distance <= kMinWristAxisDistanceM) {
            throw std::runtime_error("MOVEL 路径过于接近或穿过 J1 轴线（腕心距离下界 <= "
                                     + std::to_string(kMinWristAxisDistanceM * 1000.0) + " mm）");
        }
        if (current_axis_distance > kMinWristAxisDistanceM) escaped_axis = true;
        previous_wrist = wrist;

        // 先保留限位内各分支，再由 MOVEL 按实际设备角评估；首点固定为反馈，
        // 防止精确奇异的自由角分配改变运动的起始关节状态。
        const auto candidates = i == 0
            ? std::vector<kinematics::JointAngles>{previous_model_solution}
            : kinematics::inverse_kinematics_candidates(cartesian_points[i], previous_model_solution);
        std::vector<kinematics::JointAngles> eligible;
        bool any_limit_candidate = false, any_step_candidate = false, any_continuous_candidate = false;
        double minimum_required_step = std::numeric_limits<double>::infinity();
        const auto singular_distance = [](double q) {
            return std::abs(std::remainder(q - 90.0, 180.0));
        };
        const auto crosses_singular_band = [](double a, double b) {
            const double low = std::min(a, b) - kSingularityToleranceDeg;
            const double high = std::max(a, b) + kSingularityToleranceDeg;
            return std::ceil((low - 90.0) / 180.0) <= std::floor((high - 90.0) / 180.0);
        };
        for (const auto& candidate : candidates) {
            const auto device = joint_mapping::model_to_device(candidate);
            bool within_limits = true;
            double maximum_step = 0.0;
            for (std::size_t j = 0; j < device.size(); ++j) {
                within_limits = within_limits && inside_limit(device[j], limits[j]);
                const double d = device[j] - previous_device_joints[j];
                maximum_step = std::max(maximum_step, std::abs(d));
            }
            if (!within_limits) continue;
            any_limit_candidate = true;
            minimum_required_step = std::min(minimum_required_step, maximum_step);
            if (maximum_step > kMaxJointStep) continue;
            any_step_candidate = true;

            if (!options.strict_singularity_avoidance && i > 0
                && !continuous_joint_edge(previous_model_solution,candidate,
                                          cartesian_points[i-1],cartesian_points[i])) continue;
            any_continuous_candidate = true;

            const double wd = singular_distance(candidate[4]);
            const double ed = singular_distance(candidate[2]);
            const bool wrist_leaving = started_at_wrist_singularity
                && previous_wrist_singularity_distance <= kSingularityToleranceDeg
                && wd > previous_wrist_singularity_distance + kSingularityEscapeEpsilonDeg
                && (previous_wrist_singularity_distance < 1e-7
                    || std::remainder(previous_model_solution[4] - 90.0, 180.0)
                       * std::remainder(candidate[4] - 90.0, 180.0) > 0);
            const bool elbow_leaving = started_at_elbow_singularity
                && previous_elbow_singularity_distance <= kSingularityToleranceDeg
                && ed > previous_elbow_singularity_distance + kSingularityEscapeEpsilonDeg
                && (previous_elbow_singularity_distance < 1e-7
                    || std::remainder(previous_model_solution[2] - 90.0, 180.0)
                       * std::remainder(candidate[2] - 90.0, 180.0) > 0);
            if (options.strict_singularity_avoidance && i > 0) {
                if (crosses_singular_band(previous_model_solution[4], candidate[4]) && !wrist_leaving) continue;
                if (crosses_singular_band(previous_model_solution[2], candidate[2]) && !elbow_leaving) continue;
            }
            eligible.push_back(candidate);
        }
        const auto selected_model = select_minimax_candidate(eligible, previous_model_solution);
        if (!selected_model) {
            std::ostringstream message;
            message << "MOVEL 采样点 " << i << "/" << cartesian_points.size() - 1 << "：";
            if (!any_limit_candidate) message << "没有满足设备软限位的逆解";
            else if (!any_step_candidate) message << "全部逆解超过最大单轴步长 " << kMaxJointStep
                << " 度（所有限位内候选所需的最小最大单轴变化=" << minimum_required_step << " 度）";
            else if (!any_continuous_candidate) message << "所有限位内候选的关节插值偏离笛卡尔路径，不能连续通过；"
                << "请延长运动时间、改变路径或调整起始分支";
            else message << "MOVEL 路径经过奇异位置，全部连续候选被拒绝";
            throw std::runtime_error(message.str());
        }
        const auto& model_joints = *selected_model;
        const auto joints = joint_mapping::model_to_device(model_joints);
        const double wrist_singularity_distance = singular_distance(model_joints[4]);
        const double elbow_singularity_distance = singular_distance(model_joints[2]);

        previous_wrist_singularity_distance = wrist_singularity_distance;
        previous_elbow_singularity_distance = elbow_singularity_distance;
        
        result.push_back(joints);
        previous_model_solution = model_joints;
        previous_device_joints = joints;
    }
    if (options.strict_singularity_avoidance
        && (!escaped_axis || previous_wrist_singularity_distance <= kSingularityToleranceDeg
            || previous_elbow_singularity_distance <= kSingularityToleranceDeg)) {
        throw std::runtime_error("MOVEL 终点仍处于奇异区域，不能靠拆分路径消除终点奇异");
    }
    return result;
} // calculate_joint_trajectory

// SpeedFollow 的 PD 控制器
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

// 读取六个关节的当前角度。
joint_mapping::JointAngles read_joint_angles(int fd) {
    joint_mapping::JointAngles result{};
    for (int i = 0; i < 6; ++i) {
        // 控制器和到位检查使用设备角 m；进入运动学时才转换到模型角 q。
        result[i] = robot_reader::read_joint_angle(fd, i + 1);
    }
    return result;
} // read_joint_angles

// 检查六个关节的故障码。
void check_joint_errors(int fd) {
    for (int i = 0; i < 6; ++i) {
        const auto error = robot_reader::read_joint_error_code(fd, i + 1);

        if (error != 0) {
            throw std::runtime_error("J" + std::to_string(i + 1) + " 故障码=" + std::to_string(error));
        }
    } // for 
} // check_joint_errors

// 运动开始前检查关节状态
void check_joint_ready(int fd){
    for (int i = 0; i<6; ++i){
        const int joint_id = i+1;
        const auto error = robot_reader::read_joint_error_code(fd, joint_id);
        if (error != 0)  throw std::runtime_error("J" + std::to_string(joint_id) + " 存在故障码");
        if (!robot_reader::is_joint_enabled(fd, joint_id)) throw std::runtime_error("J" + std::to_string(joint_id) + " 存在故尚未使能");
        if (robot_reader::is_joint_running(fd, joint_id)) throw std::runtime_error("J" + std::to_string(joint_id) + " 当前正在运行");
    
    }
} // check_joint_ready

void send_speed_commands(int fd, const SpeedCommands& commands){
    for(int i = 0; i<6; ++i){
        const bool ok = robot_reader::send_write_request(
            fd, static_cast<std::uint8_t>(i+1), 0x002F, commands[i]
        );
        if (!ok) {
            throw std::runtime_error("J" + std::to_string(i + 1) + " 速度命令发送失败");
        }
    } // for 
} // send_speed_commands

void print_joint_values(
    const char* label,
    const joint_mapping::JointAngles& values
) {
    std::cerr << label << '[';
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) std::cerr << ", ";
        std::cerr << values[i];
    }
    std::cerr << "]\n";
}

void print_speed_values(const SpeedCommands& values) {
    std::cerr << "  速度命令=[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) std::cerr << ", ";
        std::cerr << values[i];
    }
    std::cerr << "]\n";
}

// void send_zero_speed(int fd) {
//     send_speed_commands(fd, SpeedCommands{});
// } //  send_zero_speed
bool send_zero_speed(int fd) noexcept {
    // 逐轴发送零速度, 某一轴失败后继续尝试其余关节. 返回值表示六个关节是否全部发送成功
    bool all_ok = true;
    for(int i = 0; i<6; ++i){
        const int joint_id = i+1;
        try{
            const bool ok = robot_reader::send_write_request(
                fd, static_cast<std::uint8_t>(joint_id), 0x002F, 0);
            if (!ok){
                all_ok = false;
                std::cerr <<  "J" << joint_id << " 零速度发送失败\n";
            }
        } catch(const std::exception& e){
            all_ok = false;
            std::cerr << "J" << joint_id << " 零速度发送异常：" << e.what() << '\n';
        } catch(...){
            all_ok = false;
            std::cerr << "J" << joint_id << " 零速度发送发生未知异常\n";
        } // try-catch

    } // for

    return all_ok;
} //  send_zero_speed

bool wait_until_target_stable(
    int fd,
    const joint_mapping::JointAngles& target,
    double tolerance_deg,
    std::chrono::seconds stable_duration,
    std::chrono::milliseconds sample_period,
    std::chrono::seconds timeout
) {
    // 到位确认只判断角度误差；Stop 后再单独确认 running=0。
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::optional<std::chrono::steady_clock::time_point> stable_since;
    while(std::chrono::steady_clock::now() < deadline){
        bool all_done = true;
        joint_mapping::JointAngles actual_angles{};
        int running_count = 0;
        double maximum_error = 0.0;

        for(int i = 0; i<6; ++i){
            const int joint_id = i+1;
            const auto error_code = robot_reader::read_joint_error_code(fd, joint_id);
            if(error_code!=0) {
                throw std::runtime_error("J" + std::to_string(joint_id) + " 到位检查时出现故障码：" + std::to_string(error_code));
            }
            const double actual_angle = robot_reader::read_joint_angle(fd, joint_id);
            const bool running = robot_reader::is_joint_running(fd, joint_id);
            const double angle_error = std::abs(target[i] - actual_angle);
            actual_angles[i] = actual_angle;
            if (running) ++running_count;
            maximum_error = std::max(maximum_error, angle_error);
            std::cout << "最终检查：J" << joint_id
                << " 当前=" << actual_angle << "°，目标=" << target[i]
                << "°，误差=" << angle_error << "°，运行状态=" << (running ? 1 : 0) << '\n';
            // running 仍为 1 是正常的：此阶段还没有发送 Stop。
            if (angle_error > tolerance_deg) {
                all_done = false;
            }
        } // for

        write_csv_record(
            "final_check",
            0,
            all_done ? "stable" : "waiting",
            std::nullopt,
            std::nullopt,
            actual_angles,
            target,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            maximum_error,
            running_count,
            "running_count=" + std::to_string(running_count)
        );
        if(all_done){
            if (!stable_since) {
                stable_since = std::chrono::steady_clock::now();
            }
            if (std::chrono::steady_clock::now() - *stable_since
                >= stable_duration) {
                return true;
            }
        } else {
            stable_since.reset();
        } // if - else
        std::this_thread::sleep_for(sample_period);
    } // while
    return false;
}  // wait_until_target_stable

bool send_stop_commands(int fd) noexcept {
    bool all_ok = true;
    for (int joint = 1; joint <= 6; ++joint) {
        try {
            if (!robot_reader::writeStop(fd, joint)) {
                all_ok = false;
            }
        } catch (const std::exception& error) {
            all_ok = false;
            std::cerr << "J" << joint << " Stop 发送失败："
                      << error.what() << '\n';
        } catch (...) {
            all_ok = false;
            std::cerr << "J" << joint << " Stop 发送发生未知异常\n";
        }
    }
    return all_ok;
}

bool wait_until_stopped(
    int fd,
    std::chrono::milliseconds sample_period,
    std::chrono::seconds timeout
) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        bool all_stopped = true;
        int running_count = 0;
        for (int joint = 1; joint <= 6; ++joint) {
            if (robot_reader::is_joint_running(fd, joint)) {
                all_stopped = false;
                ++running_count;
            }
        }

        write_csv_record(
            "stop_check", 0, all_stopped ? "stopped" : "waiting",
            std::nullopt, std::nullopt, std::nullopt, std::nullopt,
            std::nullopt, std::nullopt, std::nullopt, std::nullopt,
            std::nullopt, running_count,
            "running_count=" + std::to_string(running_count)
        );

        if (all_stopped) return true;
        std::this_thread::sleep_for(sample_period);
    }
    return false;
}

// MOVEL 的内部执行流程
bool execute_joint_trajectory(
    int fd, const kinematics::Pose& target,
    const std::vector<joint_mapping::JointAngles>& trajectory,
    const JointLimits& limits
) {
    check_joint_ready(fd);
    const auto actual_start = read_joint_angles(fd);
    if (trajectory.empty()) throw std::runtime_error("MOVEL 轨迹为空");
    for (std::size_t j = 0; j < actual_start.size(); ++j) {
        if (std::abs(actual_start[j] - trajectory.front()[j]) > 0.8) {
            throw std::runtime_error("MOVEL 实际关节角偏离预规划分段起点，拒绝执行后续段");
        }
    }
    // 当前减速比
    const std::array<double, 6> gear_ratio{
        101.0, 101.0, 101.0,
        101.0, 101.0, 101.0
    };

    SpeedFollowController controller{
        limits, gear_ratio
    };
    controller.reset();

    bool motion_started = false;
    bool first_update = true;
    auto last_control_update = std::chrono::steady_clock::now();

    try{
        for (std::size_t index = 0; index < trajectory.size(); ++index) {
            const auto& reference = trajectory[index];
            // 每轮单独计时，避免累计已过期的 next_tick 后追赶旧时间表。
            const auto cycle_start = std::chrono::steady_clock::now();

            // 每个周期读取实际角度
            const auto actual = read_joint_angles(fd);
            // 再读取故障状态
            check_joint_errors(fd);

            // 微分项使用两次控制更新之间的实际时间。
            const auto control_update_time = std::chrono::steady_clock::now();
            double dt_seconds = kControlPeriod;
            if (!first_update) {
                dt_seconds = std::chrono::duration<double>(control_update_time - last_control_update).count();
            }
            first_update = false;
            last_control_update = control_update_time;

            // 根据目标角度和实际角度计算速度命令
            const auto commands = controller.update(actual, reference, dt_seconds);
            const bool print_control_debug =
                index < 3
                || index + 1 == trajectory.size()
                || index % 25 == 0;
            if (print_control_debug) {
                std::cerr << "MOVEL 控制点 " << index << '/' << trajectory.size()
                          << "，dt=" << dt_seconds << '\n';
                print_joint_values("  实际设备角=", actual);
                print_joint_values("  参考设备角=", reference);
                print_speed_values(commands);
            }
            write_csv_record(
                "control", index, "speed_follow", std::nullopt,
                std::nullopt, actual, reference, commands, dt_seconds,
                std::nullopt, std::nullopt, std::nullopt, std::nullopt, ""
            );
            motion_started = true;
            // 顺序发送 J1～J6 的 0x002F。
            send_speed_commands(fd, commands);
            
            // 每轮至少间隔 20 ms；通信超时则本轮不再等待，轨迹整体变长。
            std::this_thread::sleep_until(
                cycle_start + std::chrono::milliseconds(20)
            );
        }
        // 正常结束时先发送零速度，让关节进入目标角度稳定阶段。
        const bool zero_speed_ok = send_zero_speed(fd);
        if(!zero_speed_ok) throw std::runtime_error("部分关节零速度命令发送失败");
        
        // 轨迹最后一个点就是最终关节目标
        if (trajectory.empty()) throw std::runtime_error("MOVEL 轨迹为空");

        const bool reached = wait_until_target_stable(
            fd, trajectory.back(),
            0.8,  // 角度误差容差
            std::chrono::seconds(1),
            std::chrono::milliseconds(20), std::chrono::seconds(5));
        if(!reached) throw std::runtime_error("MOVEL 结束时未确认六轴角度稳定");

        // SpeedFollow 的零速度不会必然清除 running 状态；
        // 按 Studio 的结束流程发送 Stop，再确认六轴真正停止。
        if (!send_stop_commands(fd)) {
            throw std::runtime_error("部分关节 Stop 发送失败");
        }
        if (!wait_until_stopped(
                fd,
                std::chrono::milliseconds(20),
                std::chrono::seconds(2))) {
            throw std::runtime_error("发送 Stop 后六轴仍未停止");
        }

        return true;
    } catch (const std::exception& error) {
        write_csv_record(
            "event", 0, "exception", std::nullopt, target,
            std::nullopt, std::nullopt, std::nullopt, std::nullopt,
            std::nullopt, std::nullopt, std::nullopt, std::nullopt,
            error.what()
        );
        // 异常时尽力发送零速度
        if (motion_started) {
            send_zero_speed(fd);
        }

        throw;
    } catch (...) {
        write_csv_record(
            "event", 0, "exception", std::nullopt, target,
            std::nullopt, std::nullopt, std::nullopt, std::nullopt,
            std::nullopt, std::nullopt, std::nullopt, std::nullopt,
            "unknown exception"
        );
        if (motion_started) {
            send_zero_speed(fd);
        }
        throw;
    } // try-catch
} // execute_joint_trajectory

} // namespace


MovelPlan plan_movel(
    const kinematics::JointAngles& current_device_angles,
    const kinematics::Pose& target, double duration_seconds, const JointLimits& limits,
    const PlanningOptions& options
) {
    if (!std::isfinite(duration_seconds) || duration_seconds < 0.5) {
        throw std::invalid_argument("MOVEL 时间不能小于 0.5 秒");
    }
    validate_start_angles(current_device_angles, limits);
    // Normalize and validate external quaternions even for stationary or explicitly split routes.
    auto normalized_target = target;
    normalized_target.orientation = kinematics::slerp(target.orientation, target.orientation, 0.0);
    (void)kinematics::wrist_center_position(normalized_target);
    const auto start_model = joint_mapping::device_to_model(current_device_angles);
    const auto start = kinematics::forward_kinematics(start_model);
    if (options.strict_singularity_avoidance) {
        validate_shoulder_connection(start, start_model, normalized_target, limits);
    }
    const auto build = [&](const std::vector<kinematics::Pose>& intermediate, const std::string& strategy) {
        auto targets = intermediate;
        targets.push_back(normalized_target);
        const double segment_duration = std::max(0.5, duration_seconds / targets.size());
        MovelPlan plan{strategy, {}};
        auto joints = current_device_angles;
        auto pose = start;
        for (auto next : targets) {
            next.orientation = kinematics::slerp(next.orientation, next.orientation, 0.0);
            auto trajectory = calculate_joint_trajectory(pose, next, segment_duration, joints, limits, options);
            joints = trajectory.back();
            plan.segments.push_back({pose, next, segment_duration, std::move(trajectory)});
            pose = next;
        }
        return plan;
    };
    if (!options.via_poses.empty()) return build(options.via_poses, "explicit_via");
    std::string direct_error;
    try { return build({}, "direct"); }
    catch (const std::runtime_error& error) {
        if (!options.allow_replanning) throw;
        direct_error = error.what();
    }
    // Each alternative changes the Cartesian path, but preserves the exact final pose.
    // A waypoint is accepted only when both MOVEL segments pass the complete preflight.
    auto orientation_first = start;
    orientation_first.orientation = normalized_target.orientation;
    std::string orientation_error;
    try { return build({orientation_first}, "orientation_then_position"); }
    catch (const std::runtime_error& error) { orientation_error = error.what(); }
    auto position_first = normalized_target;
    position_first.orientation = start.orientation;
    try { return build({position_first}, "position_then_orientation"); }
    catch (const std::runtime_error& error) {
        throw std::runtime_error("MOVEL 直达及两段重规划均失败。直达：" + direct_error
            + "；先调姿态：" + orientation_error + "；先移位置：" + error.what()
            + "。请提供经验证的中间位姿或调整起终点。");
    }
}

namespace {
void log_plan(const MovelPlan& plan) {
    double total = 0.0;
    for (const auto& segment : plan.segments) total += segment.duration_seconds;
    std::cerr << "MOVEL 规划策略=" << plan.strategy << "，分段=" << plan.segments.size()
              << "，总规划时间(s)=" << total << '\n';
    for (std::size_t i = 0; i < plan.segments.size(); ++i) {
        const auto& segment = plan.segments[i];
        write_csv_record("plan", i, plan.strategy.c_str(), segment.start, segment.target,
            segment.device_trajectory.front(), segment.device_trajectory.back(),
            std::nullopt, std::nullopt, std::nullopt, std::nullopt,
            std::nullopt, std::nullopt, "points=" + std::to_string(segment.device_trajectory.size())
                + ";duration_seconds=" + std::to_string(segment.duration_seconds));
        const auto rpy = pose_rpy(segment.target);
        std::cerr << "  段 " << i + 1 << " 目标XYZ(mm)=[" << segment.target.x_m*1000 << ", "
                  << segment.target.y_m*1000 << ", " << segment.target.z_m*1000 << "] RPY(deg)=["
                  << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << "] 时间(s)="
                  << segment.duration_seconds << '\n';
    }
}

MovelPlan plan_from_feedback(
    int fd, const kinematics::Pose& target, double duration_seconds,
    const JointLimits& limits, const PlanningOptions& options
) {
    const auto current = read_joint_angles(fd);
    const auto start_model = joint_mapping::device_to_model(current);
    const auto start = kinematics::forward_kinematics(start_model);
    // 即使后续规划失败也保留这次实际使用的六轴角；仅记录 TCP 无法复现起始分支。
    write_csv_record("planning_input", 0, "preflight", start, target, current, std::nullopt,
        std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
        "duration_seconds=" + std::to_string(duration_seconds)
        + ";signed_wrist_radius_m=" + std::to_string(signed_wrist_radius(start, start_model))
        + ";strict_singularity_avoidance=" + std::to_string(options.strict_singularity_avoidance));
    try {
        return plan_movel(current, target, duration_seconds, limits, options);
    } catch (const std::exception& error) {
        write_csv_record("planning_failure", 0, "preflight", start, target, current, std::nullopt,
            std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, error.what());
        throw;
    }
}
} // namespace

bool move_l_speed_follow(
    int fd, const kinematics::Pose& target, double duration_seconds,
    const JointLimits& limits, const PlanningOptions& options
) {
    check_joint_ready(fd);
    const auto plan = plan_from_feedback(fd, target, duration_seconds, limits, options);
    log_plan(plan);
    // All segments have been checked before the first speed command. At each waypoint
    // complete the existing settling/Stop sequence before executing the next segment.
    for (const auto& segment : plan.segments) {
        if (!execute_joint_trajectory(fd, segment.target, segment.device_trajectory, limits)) return false;
    }
    return true;
}

void initialize_csv_log(const std::string& path) {
    const std::filesystem::path csv_path(path);
    if (!csv_path.parent_path().empty()) {
        std::filesystem::create_directories(csv_path.parent_path());
    }

    g_csv_log.close();
    g_csv_log.open(csv_path, std::ios::out | std::ios::trunc);
    if (!g_csv_log) {
        throw std::runtime_error("无法创建 MOVEL CSV 日志：" + path);
    }

    g_csv_log
        << "timestamp_utc,record_type,index,stage,"
           "current_x_m,current_y_m,current_z_m,current_roll_deg,current_pitch_deg,current_yaw_deg,"
           "target_x_m,target_y_m,target_z_m,target_roll_deg,target_pitch_deg,target_yaw_deg,"
           "actual_j1_deg,actual_j2_deg,actual_j3_deg,actual_j4_deg,actual_j5_deg,actual_j6_deg,"
           "reference_j1_deg,reference_j2_deg,reference_j3_deg,reference_j4_deg,reference_j5_deg,reference_j6_deg,"
           "command_j1,command_j2,command_j3,command_j4,command_j5,command_j6,"
           "dt_seconds,position_error_mm,orientation_error_deg,max_joint_error_deg,running_count,detail\n";
    g_csv_log.flush();
}

void log_pose_record(
    const char* record_type,
    const kinematics::Pose& current_pose,
    const kinematics::Pose& target_pose,
    double duration_seconds
) {
    write_csv_record(
        record_type, 0, "pose", current_pose, target_pose,
        std::nullopt, std::nullopt, std::nullopt,
        std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
        "duration_seconds=" + std::to_string(duration_seconds)
    );
}

kinematics::Pose read_current_pose(int fd) {
    const auto current_device_angles = read_joint_angles(fd);
    const auto current_model_angles =
        joint_mapping::device_to_model(current_device_angles);
    return kinematics::forward_kinematics(current_model_angles);
}

void validate_movel_plan(
    int fd, const kinematics::Pose& target, double duration_seconds,
    const JointLimits& limits, const PlanningOptions& options
) {
    log_plan(plan_from_feedback(fd, target, duration_seconds, limits, options));
}

} // movel
