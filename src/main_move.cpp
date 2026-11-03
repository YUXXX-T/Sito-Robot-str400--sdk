#include "robot_reader.hpp"
#include "robot_driver.hpp"

#include <asm/termbits.h>  // Linux 的 termios2，可设置 2250000 这样的自定义波特率
#include <array>
#include <cerrno>
#include <chrono>
#include <fcntl.h>         // open、O_RDWR、O_NONBLOCK 等
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>     // ioctl、TCGETS2、TCSETS2
#include <system_error>
#include <thread>
#include <unistd.h>        // close
#include <vector>

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

    // 8N1：8 个数据位、无校验、1 个停止位。
    settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS | CBAUD | CIBAUD);
    settings.c_cflag |= CS8 | CLOCAL | CREAD | BOTHER;

    // NORMAL-RS485 使用 2250000 波特率。
    settings.c_ispeed = 2250000;
    settings.c_ospeed = 2250000;

    // 由协议层的 poll() 控制等待时间。
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;

    if (::ioctl(fd, TCSETS2, &settings) != 0) {
        throw_system_error("设置串口配置");
    }

    // 丢弃上一次测试残留的数据。
    if (::ioctl(fd, TCFLSH, TCIOFLUSH) != 0) {
        throw_system_error("清空串口缓存");
    }
}

int parse_int(const char* text, const char* name) {
    try {
        std::string value{text};
        std::size_t position = 0;
        const int result = std::stoi(value, &position);
        if (position != value.size()) {
            throw std::invalid_argument("不是完整整数");
        }
        return result;
    } catch (const std::exception&) {
        throw std::invalid_argument(
            std::string(name) + " 必须是整数"
        );
    }
}

double parse_double(const char* text, const char* name) {
    try {
        std::string value{text};
        std::size_t position = 0;
        const double result = std::stod(value, &position);
        if (position != value.size()) {
            throw std::invalid_argument("不是完整数字");
        }
        return result;
    } catch (const std::exception&) {
        throw std::invalid_argument(
            std::string(name) + " 必须是数字"
        );
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    bool home_first = false;
    std::vector<const char*> positional_args;
    for (int i = 1; i < argc; ++i) {
        if (std::string{argv[i]} == "--home-first") {
            home_first = true;
        } else {
            positional_args.push_back(argv[i]);
        }
    }

    const bool single_joint_command =
        positional_args.size() == 2 || positional_args.size() == 3;
    const bool multi_joint_command =
        positional_args.size() == 6 || positional_args.size() == 7;

    if (!single_joint_command && !multi_joint_command) {
        std::cerr << "用法: " << argv[0]
                  << " <joint_id> <target_angle> [mode] [--home-first]\n";
        std::cerr << "或: " << argv[0]
                  << " <J1角度> <J2角度> <J3角度>"
                     " <J4角度> <J5角度> <J6角度> [mode]\n";
        std::cerr << "mode: 0=带加减速，1=不带加减速，默认 0\n";
        std::cerr << "普通单关节: " << argv[0] << " 5 2.0 0\n";
        std::cerr << "先回零再小幅运动: " << argv[0]
                  << " 5 2.0 0 --home-first\n";
        std::cerr << "六关节示例: " << argv[0] << " 5 0 0 0 0 0 0\n";
        return 1;
    }

    if (home_first && !single_joint_command) {
        std::cerr << "--home-first 目前只支持单关节命令\n";
        return 1;
    }

    int joint_id = 0;
    double target_angle_deg = 0.0;
    std::array<double, 6> target_angles{};
    robot_driver::PositionMode mode =
        robot_driver::PositionMode::WithAcceleration;

    try {
        if (single_joint_command) {
            joint_id = parse_int(positional_args[0], "joint_id");
            target_angle_deg = parse_double(
                positional_args[1],
                "target_angle_deg"
            );
        } else {
            for (std::size_t i = 0; i < target_angles.size(); ++i) {
                target_angles[i] = parse_double(
                    positional_args[i],
                    "目标角度"
                );
            }
        }

        const bool has_mode =
            single_joint_command
                ? positional_args.size() == 3
                : positional_args.size() == 7;
        if (has_mode) {
            const int mode_value = parse_int(
                positional_args.back(),
                "mode"
            );
            if (mode_value == 0) {
                mode = robot_driver::PositionMode::WithAcceleration;
            } else if (mode_value == 1) {
                mode = robot_driver::PositionMode::WithoutAcceleration;
            } else {
                throw std::invalid_argument("mode 必须为 0 或 1");
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "参数错误：" << error.what() << '\n';
        return 1;
    }

    constexpr const char* port = "/dev/ttyUSB0";
    int fd = -1;

    try {
        fd = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            throw_system_error("打开串口");
        }

        configure_serial_port(fd);

        if (single_joint_command) {
            if (home_first) {
                std::cout << "启动回零后小幅运动流程：J" << joint_id
                          << " -> " << target_angle_deg << "°\n";
                robot_driver::move_joint_to_angle_after_home(
                    fd,
                    joint_id,
                    target_angle_deg,
                    mode
                );
                std::cout << "回零后单关节运动完成\n";
            } else {
                if (!robot_reader::is_joint_enabled(fd, joint_id)) {
                    std::cout << "开始使能：J" << joint_id << '\n';
                    robot_reader::writeEnable(fd, joint_id);
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(500)
                    );
                }

                std::cout << "开始单关节运动：J" << joint_id
                          << " -> " << target_angle_deg << "°\n";
                robot_driver::move_joint_to_angle(
                    fd,
                    joint_id,
                    target_angle_deg,
                    mode
                );

                std::cout << "单关节运动完成\n";
            }
        } else {
            for (int id = 1; id <= 6; ++id) {
                if (!robot_reader::is_joint_enabled(fd, id)) {
                    std::cout << "开始使能：J" << id << '\n';
                    robot_reader::writeEnable(fd, id);
                }
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(500)
            );

            std::cout << "六轴回零\n";
            robot_driver::move_to_zero(fd);

            std::cout << "开始六关节运动：";
            for (std::size_t i = 0; i < target_angles.size(); ++i) {
                std::cout << " J" << (i + 1)
                          << '=' << target_angles[i] << "°";
            }
            std::cout << '\n';

            robot_driver::move_joints_to_angles(
                fd,
                target_angles,
                mode
            );

            std::cout << "六关节运动完成\n";
        }


        ::close(fd);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "运动失败："
                  << error.what() << '\n';

        if (fd >= 0) {
            ::close(fd);
        }

        return 1;
    }
}
