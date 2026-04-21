#pragma once

#include <map>
#include <string>

namespace robot_process_platform::robot
{

// RobotCommandResult 表示一次机器人或 IO 命令执行结果。
struct RobotCommandResult
{
    RobotCommandResult(bool success_value, const std::string& message_value);

    bool success = false;
    std::string message;
};

// IRobot 定义运行时执行器依赖的最小机器人与 IO 抽象。
// 当前阶段只覆盖 TaskActionType 已经使用到的几类基础能力。
class IRobot
{
public:
    virtual ~IRobot() = default;

    virtual RobotCommandResult MoveJoint(
        const std::map<std::string, std::string>& action_parameters) = 0;

    virtual RobotCommandResult MoveLinear(
        const std::map<std::string, std::string>& action_parameters) = 0;

    virtual RobotCommandResult SetDigitalOutput(
        const std::map<std::string, std::string>& action_parameters) = 0;

    virtual RobotCommandResult WaitDigitalInput(
        const std::map<std::string, std::string>& action_parameters,
        int timeout_ms) = 0;

    virtual RobotCommandResult Delay(
        const std::map<std::string, std::string>& action_parameters,
        int timeout_ms) = 0;
};

}  // namespace robot_process_platform::robot
