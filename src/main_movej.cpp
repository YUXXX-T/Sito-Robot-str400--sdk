#include "joint_mapping.hpp"
#include "movej.hpp"
#include "robot_reader.hpp"

#include <asm/termbits.h>
#include <cerrno>
#include <cmath>
#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <system_error>
#include <thread>
#include <unistd.h>

namespace {

[[noreturn]] void throw_system_error(const char* operation) {
    throw std::system_error(errno, std::generic_category(), operation);
}

void configure_serial_port(int fd) {
    termios2 settings{};
    if (::ioctl(fd, TCGETS2, &settings) != 0) {
        throw_system_error("读取串口配置");
    }

    settings.c_iflag = 0;
    settings.c_oflag = 0;
    settings.c_lflag = 0;
    settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS | CBAUD | CIBAUD);
    settings.c_cflag |= CS8 | CLOCAL | CREAD | BOTHER;
    settings.c_ispeed = 2250000;
    settings.c_ospeed = 2250000;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;

    if (::ioctl(fd, TCSETS2, &settings) != 0) {
        throw_system_error("设置串口配置");
    }
    if (::ioctl(fd, TCFLSH, TCIOFLUSH) != 0) {
        throw_system_error("清空串口缓存");
    }
}

double parse_double(const char* text, const char* name) {
    try {
        const std::string value{text};
        std::size_t used = 0;
        const double result = std::stod(value, &used);
        if (used != value.size() || !std::isfinite(result)) {
            throw std::invalid_argument("不是有限数字");
        }
        return result;
    } catch (const std::exception&) {
        throw std::invalid_argument(std::string(name) + " 必须是有限数字");
    }
}

void print_usage(const char* program) {
    std::cerr
        << "用法: " << program
        << " <abs|rel> <J1> <J2> <J3> <J4> <J5> <J6> <duration_s>\n"
        << "示例（绝对 MOVEJ）: " << program << " abs 0 0 90 0 90 0 5\n"
        << "示例（相对 MOVEJ）: " << program << " rel 0 0 5 0 0 0 2\n"
        << "关节角单位为度；duration_s 最小 0.5 秒。\n"
        << "程序不会回零；会自动使能未使能的关节。\n";
}

movej::JointLimits device_limits() {
    movej::JointLimits limits{};
    for (std::size_t joint = 0; joint < limits.size(); ++joint) {
        limits[joint] = {
            joint_mapping::kDeviceJointLimitsDeg[joint][0],
            joint_mapping::kDeviceJointLimitsDeg[joint][1]
        };
    }
    return limits;
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        print_usage(argv[0]);
        return 0;
    }
    if (argc != 9) {
        print_usage(argv[0]);
        return 1;
    }

    try {
        movej::Options options{};
        const std::string mode{argv[1]};
        if (mode == "abs") {
            options.target_mode = movej::TargetMode::Absolute;
        } else if (mode == "rel") {
            options.target_mode = movej::TargetMode::Relative;
        } else {
            throw std::invalid_argument("mode 必须是 abs 或 rel");
        }

        movej::JointAngles command{};
        for (std::size_t joint = 0; joint < command.size(); ++joint) {
            command[joint] = parse_double(argv[joint + 2], "关节角");
        }
        const double duration_seconds = parse_double(argv[8], "duration_s");
        if (duration_seconds < 0.5 || duration_seconds > 3600.0) {
            throw std::invalid_argument("duration_s 必须在 0.5～3600 秒之间");
        }

        constexpr const char* port = "/dev/ttyUSB0";
        int fd = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            throw_system_error("打开串口");
        }

        try {
            configure_serial_port(fd);

            for (int joint = 1; joint <= 6; ++joint) {
                if (!robot_reader::is_joint_enabled(fd, joint)) {
                    std::cout << "使能 J" << joint << '\n';
                    robot_reader::writeEnable(fd, joint);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            std::cout << "MOVEJ 模式=" << mode << "，目标/增量设备角(deg)=[";
            for (std::size_t joint = 0; joint < command.size(); ++joint) {
                if (joint != 0) std::cout << ", ";
                std::cout << command[joint];
            }
            std::cout << "]，时间=" << duration_seconds << " s\n";

            const bool completed = movej::movej(
                fd,
                command,
                options.target_mode,
                duration_seconds,
                device_limits(),
                options
            );

            ::close(fd);
            if (!completed) {
                std::cerr << "MOVEJ 未完成\n";
                return 1;
            }
            std::cout << "MOVEJ 完成，关节保持使能\n";
            return 0;
        } catch (...) {
            ::close(fd);
            throw;
        }
    } catch (const std::exception& error) {
        std::cerr << "MOVEJ 测试失败：" << error.what() << '\n';
        return 1;
    }
}
