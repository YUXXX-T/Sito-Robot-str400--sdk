#include "protocol.hpp"

#include <asm/termbits.h>
#include <array>
#include <cerrno>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <system_error>
#include <sys/ioctl.h>
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

void print_frame(const char* label, const std::array<std::uint8_t, 10>& frame) {
    std::cout << label << ':';
    for (const auto byte : frame) {
        std::cout << ' '
                  << std::uppercase << std::hex
                  << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned>(byte);
    }
    std::cout << std::dec << '\n';
}

void write_all(int fd, const std::array<std::uint8_t, 10>& frame) {
    std::size_t offset = 0;
    while (offset < frame.size()) {
        const auto n = ::write(
            fd,
            frame.data() + offset,
            frame.size() - offset
        );
        if (n < 0) {
            if (errno == EINTR) continue;
            throw_system_error("发送故障查询报文");
        }
        if (n == 0) {
            throw std::runtime_error("故障查询报文未发送任何字节");
        }
        offset += static_cast<std::size_t>(n);
    }
}

}  // namespace

int main() {
    constexpr const char* port = "/dev/ttyUSB0";
    constexpr std::array<std::uint8_t, 10> request{
        0x01, 0x03,
        0x00, 0x1B,
        0x00, 0x00, 0x00, 0x01,
        0x05, 0x17
    };

    int fd = -1;
    try {
        fd = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            throw_system_error("打开串口");
        }

        configure_serial_port(fd);
        print_frame("TX", request);
        write_all(fd, request);

        const auto response = serial_protocol::receive_frame(fd, request);
        print_frame("RX", response);

        const auto value =
            (static_cast<std::uint32_t>(response[4]) << 24)
            | (static_cast<std::uint32_t>(response[5]) << 16)
            | (static_cast<std::uint32_t>(response[6]) << 8)
            | static_cast<std::uint32_t>(response[7]);

        std::cout << "J1 故障码: " << value << '\n';
        ::close(fd);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "J1 故障查询失败：" << error.what() << '\n';
        if (fd >= 0) {
            ::close(fd);
        }
        return 1;
    }
}
