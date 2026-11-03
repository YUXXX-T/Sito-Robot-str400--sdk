#pragma once

#include <cerrno>     // 获取系统调用失败后的错误码 errno
#include <chrono>     // 表示时间、计算超时、休眠时间等
#include <cstdint>    // 提供固定宽度整数类型，如 uint8_t、uint32_t
#include <cstdio>     // C 风格输入输出，如 FILE*、fopen、printf
#include <stdexcept>  // 提供标准异常类型，如 std::runtime_error
#include <poll.h>     // Linux/POSIX 下等待文件描述符事件，如串口、Socket 可读
#include <unistd.h>   // POSIX 系统调用，如 read、write、close、usleep


#include <array>
#include <cstddef>


namespace serial_protocol {
void pro_j1(int fd); // 测试接口
std::uint16_t crc16(const std::uint8_t* data, std::size_t size); // 函数只计算 CRC 数值
// 接收固定 10 字节 NORMAL-RS485 回复，验证 CRC。
// 0x03 读回复匹配前 4 字节，[4..7] 为返回值；写回复匹配前 8 字节完整回显。
std::array<std::uint8_t, 10> receive_frame(int fd, const  std::array<std::uint8_t, 10>& request);




}
