#include "protocol.hpp"
#include "robot_reader.hpp"

#include <asm/termbits.h>  // Linux 的 termios2，可设置 2250000 这样的自定义波特率
#include <cerrno>
#include <fcntl.h>         // open、O_RDWR、O_NONBLOCK 等
#include <iostream>
#include <stdexcept>
#include <sys/ioctl.h>     // ioctl、TCGETS2、TCSETS2
#include <system_error>
#include <unistd.h>        // close
#include <string>

namespace {

[[noreturn]] void throw_system_error(const char* operation) {
    throw std::system_error(errno, std::generic_category(), operation);
}

void configure_serial_port(int fd) {
    termios2 settings{};
    if (::ioctl(fd, TCGETS2, &settings) != 0) {
        throw_system_error("读取串口配置");
    }

    // 清除输入、输出和本地模式，关闭串口驱动的额外处理。
    settings.c_iflag = 0;
    settings.c_oflag = 0;
    settings.c_lflag = 0;

    // 8N1：8 个数据位、无校验、1 个停止位。
    settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS | CBAUD | CIBAUD);
    settings.c_cflag |= CS8 | CLOCAL | CREAD | BOTHER;

    // 2250000 不是普通 termios 常量，使用 BOTHER 设置实际数值。
    settings.c_ispeed = 2250000;
    settings.c_ospeed = 2250000;

    // 由 poll() 控制等待时间，不让 read() 自己阻塞。
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;

    if (::ioctl(fd, TCSETS2, &settings) != 0) {
        throw_system_error("设置串口配置");
    }

    // 丢弃打开串口前残留的输入输出数据，确保本次测试从新请求开始。
    if (::ioctl(fd, TCFLSH, TCIOFLUSH) != 0) {
        throw_system_error("清空串口缓存");
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "用法: " << argv[0] << " <joint_id>\n";
        std::cerr << "例如: " << argv[0] << " 5\n";
        return 1;
    }
    int joint_id = 0;
    try {
        const std::string text = argv[1];
        std::size_t position = 0;

        joint_id = std::stoi(text, &position);

        if (position != text.size()) {
            throw std::invalid_argument("joint_id 不是完整整数");
        }
    } catch (const std::exception&) {
        std::cerr << "joint_id 必须是 1～6 的整数\n";
        return 1;
    }

    constexpr const char* port = "/dev/ttyUSB0";

    int fd = -1;

    try {
        // O_NONBLOCK 与 protocol.cpp 中的 poll/read 配合使用。
        fd = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            throw_system_error("打开串口");
        }

        configure_serial_port(fd);
        // serial_protocol::pro_j1(fd);
        
        // 使能测试
        // const bool enabled = robot_reader::is_joint_enabled(fd, 1);
        // std::cout << "使能状态："
        //     << (enabled ? "已使能" : "未使能")
        //     << '\n';

        // 角度获取测试
        // const auto joint_angle = robot_reader::read_joint_angle(fd, joint_id);
        // std::cout << "关节电机角度为："
        //     << joint_angle << "°"
        //     << '\n';

        // 运行测试 
        // const auto running_flag = robot_reader::is_joint_running(fd, joint_id);
        // std::cout << "电机运行状态："
        //     << running_flag 
        //     << '\n';

        // 使能测试
        // const auto enable_flag = robot_reader::writeEnable(fd, joint_id);
        // std::cout << "写使能确认:"
        //     << enable_flag 
        //     << '\n';
        const auto disable_flag = robot_reader::writeDisable(fd, joint_id);
        std::cout << "写失能确认:"
            << disable_flag 
            << '\n';
        

        ::close(fd);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "串口测试失败：" << error.what() << '\n';
        if (fd >= 0) {
            ::close(fd);
        }
        return 1;
    }
}
