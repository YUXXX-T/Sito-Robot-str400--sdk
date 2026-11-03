#include "protocol.hpp"
#include "robot_reader.hpp"

#include <cerrno>
#include <stdexcept>
#include <unistd.h>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace {

void print_frame(const std::array<std::uint8_t, 10>& frame, const char* name) {
    std::cout << name << ':';

    for (const auto byte : frame) {
        std::cout << ' '
                  << std::uppercase
                  << std::hex
                  << std::setw(2)
                  << std::setfill('0')
                  << static_cast<unsigned>(byte);
    }

    std::cout << std::dec << '\n';
} // print_frame 可以对 response 也可以对 request

std::uint32_t read_joint_register(
    int fd,
    int joint_id,
    std::uint16_t register_address,
    std::uint8_t read_count
) {
    if (joint_id < 1 || joint_id > 6) {
        throw std::invalid_argument("joint_id must be 1..6");
    }

    if (read_count != 1 && read_count != 2) {
        throw std::invalid_argument("read_count must be 1 or 2");
    }

    const auto joint = static_cast<std::uint8_t>(joint_id);

    // NORMAL-RS485 读取请求：固定 10 字节。
    std::array<std::uint8_t, 10> request{
        joint,
        0x03, // 读取保持寄存器
        static_cast<std::uint8_t>(register_address >> 8),
        static_cast<std::uint8_t>(register_address & 0xFF),
        0x00, 0x00, 0x00, read_count,
        0x00, 0x00
    };

    const auto crc = serial_protocol::crc16(request.data(), 8);
    request[8] = static_cast<std::uint8_t>(crc >> 8);
    request[9] = static_cast<std::uint8_t>(crc & 0xFF);

    // receive_frame() 只负责接收，所以这里先发送请求。
    std::size_t offset = 0;
    while (offset < request.size()) {
        const auto sent = ::write(
            fd,
            request.data() + offset,
            request.size() - offset
        );

        if (sent < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error("请求发送失败");
        }

        if (sent == 0) {
            throw std::runtime_error("请求未发送任何数据");
        }

        offset += static_cast<std::size_t>(sent);
    }

    std::array<std::uint8_t, 10> response{};
    try {
        response = serial_protocol::receive_frame(fd, request);
    } catch (const std::exception& error) {
        std::ostringstream context;
        context << "J" << joint_id
                << " 读取寄存器 0x"
                << std::uppercase << std::hex << std::setw(4)
                << std::setfill('0') << register_address
                << " 失败：" << error.what();
        throw std::runtime_error(context.str());
    }

    // 回复数据区 response[4]～response[7] 按大端序组成 32 位数值。
    return (static_cast<std::uint32_t>(response[4]) << 24)
        | (static_cast<std::uint32_t>(response[5]) << 16)
        | (static_cast<std::uint32_t>(response[6]) << 8)
        | static_cast<std::uint32_t>(response[7]);
}

std::array<std::uint8_t, 10> make_write_request(
    std::uint8_t joint_id,
    std::uint16_t reg,
    std::int32_t value
) {
    if (joint_id < 1 || joint_id > 6) throw std::invalid_argument("关节地址必须为1～6");

    const auto data = static_cast<std::uint32_t>(value);

    std::array<std::uint8_t, 10>  frame{
        joint_id,
        0x06, // 写单个保持寄存器; 写多个保持寄存器为0x10
        static_cast<std::uint8_t>(reg >> 8),
        static_cast<std::uint8_t>(reg & 0x00FF),
        static_cast<std::uint8_t>(data>>24),
        static_cast<std::uint8_t>(data>>16),
        static_cast<std::uint8_t>(data>>8),
        static_cast<std::uint8_t>(data & 0x000000FF),
        0, 0
    };

    const std::uint16_t crc = serial_protocol::crc16(frame.data(), 8);
    frame[8] = static_cast<std::uint8_t>(crc >> 8);
    frame[9] = static_cast<std::uint8_t>(crc & 0x00FF);
    
    return frame;

} // make_write_request



std::uint8_t check_joint_id(int joint_id) {
    if (joint_id < 1 || joint_id > 6) {
        throw std::invalid_argument("joint_id must be 1..6");
    }

    return static_cast<std::uint8_t>(joint_id);
} // check_joint_id


}  // namespace

namespace robot_reader {
    
bool send_write_request(int fd, std::uint8_t joint_id, std::uint16_t reg, std::int32_t value) {
    const auto frame = make_write_request(joint_id, reg, value);
    print_frame(frame, "TX");

    // 1. 完整发送请求
    std::size_t offset = 0;
    while(offset<frame.size()){
        const auto n = ::write(fd, frame.data()+offset, frame.size()-offset);

        if (n<0) {
            if (errno == EINTR) continue;
            throw std::system_error(errno, std::generic_category(), "发送写请求失败");
        }
        if (n == 0) throw std::runtime_error("写请求未发送任何数据");
        offset += static_cast<std::size_t>(n);
    }

    // 2. 写回复检查前 8 字节完整回显和 CRC；读回复仅匹配前 4 字节。
    const auto response = serial_protocol::receive_frame(fd, frame);
    // 3. 输出response原值
    print_frame(response, "RX");


    return true;
} // send_write_request

bool is_joint_enabled(int fd, int joint_id) {
    const auto value = read_joint_register(fd, joint_id, 0x0010, 1);

    if (value != 0 && value != 1) {
        throw std::runtime_error("使能状态无效");
    }

    return value == 1;
} // is_joint_enabled

bool is_joint_running(int fd, int joint_id) {
    const auto value = read_joint_register(fd, joint_id, 0x001A, 1);

    if (value != 0 && value != 1) {
        throw std::runtime_error("运行状态无效");
    }

    return value == 1;
} // is_joint_running

std::int32_t read_joint_error_code(int fd, int joint_id) {
    const auto value = read_joint_register(fd, joint_id, 0x001B, 1);
    return static_cast<std::int32_t>(value);
} // read_joint_error_code

std::int32_t read_joint_encoder(int fd, int joint_id) {
    const auto value = read_joint_register(fd, joint_id, 0x0015, 2);

    if (value > 32767) {
        throw std::runtime_error("角度编码器值超出范围");
    }

    return static_cast<std::int32_t>(value);
} // read_joint_encoder

double read_joint_angle(int fd, int joint_id) {
    const auto value = read_joint_encoder(fd, joint_id);

    // 按设备寄存器定义换算设备角 m；模型零位/方向转换由 joint_mapping 单独处理。
    double angle =
        static_cast<double>(value) / 32767.0 * 360.0;

    // 仅把角度归一到 [-180°, 180°)，不改变设备定义的零位或正方向。
    if (angle >= 180.0) {
        angle -= 360.0;
    }

    return angle;
} // read_joint_angle

bool writeEnable(int fd, int joint_id) {
    std::uint8_t joint = check_joint_id(joint_id);
    // send_write_request(int fd, std::uint8_t joint_id, std::uint16_t reg, std::int32_t value)
    // reg 寄存器地址, value 是写入值
    send_write_request(
        fd, joint, 0x0010, 1
    );

    // 再读取设备实际报告的使能状态
    const bool enabled = is_joint_enabled(fd, joint_id);
    if (!enabled) {
        throw std::runtime_error("写使能回复有效，但设备仍报告未使能");
    }

    return true;
} // writeEnable

bool writeDisable(int fd, int joint_id){
    std::uint8_t joint = check_joint_id(joint_id);
    send_write_request(
        fd, joint, 0x0010, 0
    );

    const bool enabled = is_joint_enabled(fd, joint_id);
    if (enabled) {
        throw std::runtime_error(
            "写失能回复有效，但设备仍报告已使能"
        );
    }

    return true;

} // writeDisable

bool writeStop(int fd, int joint_id){
    const auto joint = check_joint_id(joint_id);

    // Studio 的 NORMAL-RS485 实现使用每关节寄存器 0x0033，写入 1 请求停止。
    // send_write_request 会发送请求并等待、校验设备的写回复。
    return send_write_request(fd, joint, 0x0033, 1);
} // writeStop
}  // namespace robot_reader

