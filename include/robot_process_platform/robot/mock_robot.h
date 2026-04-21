#pragma once

#include <vector>

#include "robot_process_platform/robot/robot_interface.h"

namespace robot_process_platform::robot
{

// MockRobot 用于在没有真实机器人 SDK 时验证任务执行链。
// 它只记录收到的命令，并始终返回成功结果。
class MockRobot : public IRobot
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

}  // namespace robot_process_platform::robot
