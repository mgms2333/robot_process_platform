#pragma once

#include <vector>

#include "robot_process_platform/device/interfaces/robot_interface.h"

namespace robot_process_platform::device
{

// HyRobot 是当前阶段的占位机器人实现。
// 现在它仍然使用 mock 风格行为验证执行链，
// 后续可在此基础上逐步替换成真实的 hy 机器人对接代码。
class HyRobot : public IRobot
{
public:
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

    const std::vector<std::string>& GetExecutionLog() const;

private:
    std::string BuildLogEntry(
        const std::string& command_name,
        const std::map<std::string, std::string>& action_parameters,
        int timeout_ms) const;

    std::vector<std::string> execution_log;
};

}  // namespace robot_process_platform::device
