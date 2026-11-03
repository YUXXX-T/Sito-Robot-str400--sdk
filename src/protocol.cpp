#include "protocol.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <vector>

namespace serial_protocol {

namespace {

std::string format_frame(const std::uint8_t* data, std::size_t size) {
    std::ostringstream output;
    output << std::uppercase << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < size; ++i) {
        if (i != 0) output << ' ';
        output << std::setw(2) << static_cast<unsigned>(data[i]);
    }
    return output.str();
}

std::string format_u16(std::uint16_t value) {
    std::ostringstream output;
    output << "0x" << std::uppercase << std::hex << std::setfill('0')
           << std::setw(4) << value;
    return output.str();
}

} // namespace

std::uint16_t crc16(const std::uint8_t* data, std::size_t size){    // 函数只计算 CRC 数值

    // crc 寄存器
    std::uint16_t crc = 0xffff;

    for (std::size_t i = 0; i<size; ++i){
        // 首次 XOR 运算
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit){
            if (crc& 0x0001) { // 判断右移一位的最低位是否为1, & 与运算只有在同时为1时为1，其余为0
                // static_cast<std::uint16_t> 表示显式转换为 16 位无符号整数，保持 crc 状态始终只保留 16 位
                crc = static_cast<std::uint16_t>(
                    (crc >> 1) ^ 0xA001);
            } else {
                crc = static_cast<std::uint16_t>(crc >> 1);
            }
        }
    }

    return crc;
}

std::array<std::uint8_t, 10> receive_frame(int fd, const  std::array<std::uint8_t, 10>& request){
    // 接收并校验一帧固定长度的 NORMAL-RS485 回复。
    // 支持分段到达，最长等待 500 ms；成功返回 10 字节完整帧，失败抛出异常。

    std::array<std::uint8_t, 10> respond{};
    std::vector<std::uint8_t> window;
    window.reserve(respond.size());
    std::array<std::uint8_t, 10> last_candidate{};
    bool have_bad_candidate = false;
    // 读请求的 [4..7] 是读取参数，读回复的 [4..7] 是寄存器值，不能要求相等。
    // 写回复继续校验整个地址/功能码/寄存器/写入值回显。
    const std::size_t header_size = request[1] == 0x03 ? 4 : 8;
    std::size_t received_bytes = 0;
    constexpr std::size_t kMaxDiagnosticBytes = 64;
    std::vector<std::uint8_t> diagnostic_bytes;
    diagnostic_bytes.reserve(kMaxDiagnosticBytes);

    using Clock = std::chrono::steady_clock;
    const auto deadline = Clock::now() + std::chrono::milliseconds(500);
    while (Clock::now() < deadline) {
        const auto remaining = std::chrono::ceil<std::chrono::milliseconds>(deadline - Clock::now());

        if (remaining.count() <= 0) break;
        
        pollfd pfd{fd, POLLIN, 0}; // 等待这个文件描述符 fd 是否变得可读, events想等待的事件为 POLLIN, pfd.revents=0表示暂时还没有填入检测结果
        const int result = ::poll(&pfd, 1, static_cast<int>(remaining.count())); // pollfd 结构体地址, 要监听 1 个文件描述符, 最多等待多少毫秒

        // result < 0 表示调用失败，需要检查 errno
        if (result < 0) {
            if (errno == EINTR) continue;
            throw std::runtime_error("poll 失败"); // 中断 while
        }
        // result 为0表示没有一个文件描述符发生了事件
        if (result == 0) {
            break; // 统一在循环外报告是否收到字节、CRC 或帧匹配错误。
        }
        // result > 0, 串口文件描述符可以读取调用，但是串口异常，获取不到数据
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) { // 串口错误、断开，或者 fd 无效
            throw std::runtime_error("串口发生错误或已断开");
        }
        if (!(pfd.revents & POLLIN)) {
            continue;
        }

        // 每次只读取一个字节，避免把下一帧的一部分一起吞掉；
        // 同时可以在残留字节或 CRC 错位时重新寻找当前请求的帧头。
        std::uint8_t byte = 0;
        const auto n = ::read(fd, &byte, 1);
        if (n<0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            throw std::runtime_error("read 失败");
        }
        if (n==0) continue;

        ++received_bytes;
        if (diagnostic_bytes.size() < kMaxDiagnosticBytes) diagnostic_bytes.push_back(byte);
        window.push_back(byte);
        if (window.size() != respond.size()) continue;

        // 固定 10 字节滑动窗口：只有匹配头部且 CRC 正确才接受。
        // 不按读取参数匹配数据区，也不丢弃可能包含下一帧的重叠后缀。
        if (!std::equal(window.begin(), window.begin() + header_size, request.begin())) {
            window.erase(window.begin());
            continue;
        }

        std::copy(window.begin(), window.end(), respond.begin());
        const auto received_crc =
            (static_cast<std::uint16_t>(respond[8]) << 8)
            | static_cast<std::uint16_t>(respond[9]);
        const auto calculated_crc = crc16(respond.data(), 8);
        if (received_crc == calculated_crc) {
            return respond;
        }

        last_candidate = respond;
        have_bad_candidate = true;
        // 当前 10 字节 CRC 不正确；删除首字节后继续检查后缀，
        // 防止坏帧前面夹着下一帧的起始字节。
        window.erase(window.begin());
    }

    if (have_bad_candidate) {
        const auto received_crc =
            (static_cast<std::uint16_t>(last_candidate[8]) << 8)
            | static_cast<std::uint16_t>(last_candidate[9]);
        const auto calculated_crc = crc16(last_candidate.data(), 8);
        throw std::runtime_error(
            "接收超时，未找到有效回复；最后候选帧 CRC 错误，RX="
            + format_frame(last_candidate.data(), last_candidate.size())
            + "，收到CRC=" + format_u16(received_crc)
            + "，计算CRC=" + format_u16(calculated_crc)
        );
    }

    if (received_bytes == 0) {
        throw std::runtime_error("接收回复超时：500 ms 内未收到任何字节");
    }
    throw std::runtime_error(
        "接收回复超时：收到 " + std::to_string(received_bytes)
        + " 字节，但未找到头部匹配且 CRC 正确的完整帧；RX="
        + format_frame(diagnostic_bytes.data(), diagnostic_bytes.size())
        + (received_bytes > diagnostic_bytes.size() ? "（仅保留前 64 字节）" : "")
    );
}


void pro_j1(int fd){
    // 实测 NORMAL-RS485 协议
    const std::array<std::uint8_t, 10> request{
        0x01, // 设备地址
        0x03, // 功能码
        0x00, 0x10, // 寄存器地址
        0x00, 0x00, 0x00, 0x01, // 读取参数
        0xC4, 0xB2 // CRC校验
    };

    // 官方文档推荐设定
    // const std::uint8_t request[]={
    //     0x80,             // 整机控制器地址
    //     0x03,             // 读寄存器
    //     0x10, 0x10,       // J1 伺服状态寄存器
    //     0x00, 0x01,       // 读取一个寄存器
    //     0x79, 0xF8        // CRC，低字节在前
    // };
    
    // NORMAL 读取角度
    // const std::array<std::uint8_t, 10> request{
    //     0x03, 
    //     0x03,
    //     0x00, 0x15,
    //     0x00, 0x00, 0x00, 0x02,
    //     0x1c, 0xBF
    // };

    // NORMAL 读取状态 未完成版
    // const std::uint8_t request[]={
    //     0x01, 
    //     0x03,
    //     0x00, 0x15,
    //     0x00, 0x00, 0x00, 0x02,
    //     0xc5, 0x3e
    // };

    ssize_t sent;
    do {
        sent = ::write(fd, request.data(), request.size());
    } while (sent<0 && errno == EINTR);

    if (sent!=static_cast<ssize_t>(request.size())){
        throw std::runtime_error("请求未完整发送");
    }

//     using Clock = std::chrono::steady_clock;
//     const auto deadline = Clock::now() + std::chrono::milliseconds(500);
//     std::size_t total = 0;
//     // 只发送一次，然后在 500 ms 内收集并打印收到的所有字节
//     while(Clock::now() < deadline) {
//         const auto remaining = 
//             std::chrono::ceil<std::chrono::milliseconds>(
//                 deadline - Clock::now()).count();
//         if (remaining <= 0) break;

//         pollfd pfd{fd, POLLIN, 0};
//         const int result = ::poll(&pfd, 1, static_cast<int>(remaining));

//         if (result < 0) {
//             if (errno == EINTR) continue;
//             throw std::runtime_error("poll 失败");
//         }
//         if (result == 0) break;

//         if (pfd.revents & POLLIN) {
//             std::uint8_t buffer[256];
//             const auto n = ::read(fd, buffer, sizeof(buffer));

//         if (n < 0) {
//                 if (errno == EINTR || errno == EAGAIN) continue;
//                 throw std::runtime_error("read 失败");
//             }

//             if (n > 0) {
//                 std::printf("RX (%zd bytes): ", n);
//                 for (ssize_t i = 0; i < n; ++i) {
//                     std::printf("%02X ", static_cast<unsigned>(buffer[i]));
//                 }
//                 std::printf("\n");
//                 total += static_cast<std::size_t>(n);
//             }

//         }
//         if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
//             throw std::runtime_error("串口断开或发生错误");
//         }
//     }
//     std::printf("共收到 %zu 字节\n", total);
// }

    // receive_frame 代替
    const auto response = receive_frame(fd, request);
    std::printf("有效回复：");
    for (auto byte : response) {
        std::printf("%02X ", static_cast<unsigned>(byte));
    }
    std::printf("\n");

}

}  // namespace serial_protocol
