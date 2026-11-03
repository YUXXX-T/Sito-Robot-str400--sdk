#pragma once

#include <array>
#include <cstddef>
#include <stdexcept>
#include <cstdint>
#include <iostream>

namespace robot_reader{ // 读取状态、发送通用写寄存器请求
bool send_write_request(int fd, std::uint8_t joint_id, std::uint16_t reg, std::int32_t value); // 发送写请求并等待、校验设备回复
bool is_joint_enabled(int fd, int joint_id);          // 读取关节使能状态
bool is_joint_running(int fd, int joint_id);          // 读取关节运行状态
std::int32_t read_joint_error_code(int fd, int joint_id); // 读取关节故障码
std::int32_t read_joint_encoder(int fd, int joint_id);    // 读取关节原始编码器值
// 将编码器寄存器值换算为设备报告的关节实际角，并折到 [-180°, 180°)。
// 返回设备角 m；调用运动学前用 joint_mapping::device_to_model() 转为模型角 q。
double read_joint_angle(int fd, int joint_id);

bool writeEnable(int fd, int joint_id);             // 请求关节使能
bool writeDisable(int fd, int joint_id);             // 请求关节失能
bool writeStop(int fd, int joint_id);               // 请求关节停止并验证写回复；不代表机械臂已物理停稳


}
