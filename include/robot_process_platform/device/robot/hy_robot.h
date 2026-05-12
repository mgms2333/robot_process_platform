#pragma once

#include <map>
#include <string>
#include <vector>

#include "robot_process_platform/device/interfaces/robot_interface.h"

namespace robot_process_platform::device
{

struct HyRobotConfig
{
    bool enable_sdk = false;
    bool auto_connect_controller = true;
    bool auto_electrify = false;
    unsigned int box_id = 0;
    unsigned int robot_id = 0;
    unsigned short port = 10003;
    int motion_done_timeout_ms = 30000;
    int io_poll_interval_ms = 50;
    double default_velocity = 100.0;
    double default_acceleration = 100.0;
    double default_radius = 0.0;
    std::string host_name = "127.0.0.1";
    std::string tcp_name = "TCP";
    std::string ucs_name = "Base";
    std::map<std::string, int> box_di_bits;
    std::map<std::string, int> box_do_bits;
};

// HyRobot 是当前阶段的占位机器人实现。
// 现在它仍然使用 mock 风格行为验证执行链。
// 当 enable_sdk=true 且 CMake 打开 hy SDK 支持后，
// HyRobot 可切换到真实的华沿机器人 SDK 调用。
// 设备连接由平台/设备管理流程显式触发，
// runtime 不负责在执行动作时隐式建连。
class HyRobot : public IRobot
{
public:
    HyRobot();
    explicit HyRobot(const HyRobotConfig& config_value);
    ~HyRobot() override;

    RobotCommandResult MoveJoint(
        const std::map<std::string, std::string>& action_parameters) override;

    RobotCommandResult MoveLinear(
        const std::map<std::string, std::string>& action_parameters) override;

    RobotCommandResult SetDigitalOutput(
        const std::map<std::string, std::string>& action_parameters) override;

    RobotCommandResult WaitDigitalInput(
        const std::map<std::string, std::string>& action_parameters,
        int timeout_ms) override;

    RobotCommandResult Delay(
        const std::map<std::string, std::string>& action_parameters,
        int timeout_ms) override;

    bool Connect();
    void Disconnect();
    bool IsConnected() const;

    const std::vector<std::string>& GetExecutionLog() const;
    const HyRobotConfig& GetConfig() const;

private:
    RobotCommandResult ExecuteMockCommand(
        const std::string& command_name,
        const std::map<std::string, std::string>& action_parameters,
        int timeout_ms);

    std::string BuildLogEntry(
        const std::string& command_name,
        const std::map<std::string, std::string>& action_parameters,
        int timeout_ms) const;

    HyRobotConfig config;
    bool sdk_connected = false;
    std::vector<std::string> execution_log;
};

}  // namespace robot_process_platform::device
