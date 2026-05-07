#include "robot_process_platform/device/robot/hy_robot.h"

#include <sstream>

namespace robot_process_platform::device
{

RobotCommandResult HyRobot::MoveJoint(
    const std::map<std::string, std::string>& action_parameters)
{
    execution_log.push_back(BuildLogEntry("MoveJoint", action_parameters, 0));
    return RobotCommandResult(true, "MoveJoint executed.");
}

RobotCommandResult HyRobot::MoveLinear(
    const std::map<std::string, std::string>& action_parameters)
{
    execution_log.push_back(BuildLogEntry("MoveLinear", action_parameters, 0));
    return RobotCommandResult(true, "MoveLinear executed.");
}

RobotCommandResult HyRobot::SetDigitalOutput(
    const std::map<std::string, std::string>& action_parameters)
{
    execution_log.push_back(BuildLogEntry("SetDigitalOutput", action_parameters, 0));
    return RobotCommandResult(true, "SetDigitalOutput executed.");
}

RobotCommandResult HyRobot::WaitDigitalInput(
    const std::map<std::string, std::string>& action_parameters,
    int timeout_ms)
{
    execution_log.push_back(BuildLogEntry("WaitDigitalInput", action_parameters, timeout_ms));
    return RobotCommandResult(true, "WaitDigitalInput executed.");
}

RobotCommandResult HyRobot::Delay(
    const std::map<std::string, std::string>& action_parameters,
    int timeout_ms)
{
    execution_log.push_back(BuildLogEntry("Delay", action_parameters, timeout_ms));
    return RobotCommandResult(true, "Delay executed.");
}

const std::vector<std::string>& HyRobot::GetExecutionLog() const
{
    return execution_log;
}

std::string HyRobot::BuildLogEntry(
    const std::string& command_name,
    const std::map<std::string, std::string>& action_parameters,
    int timeout_ms) const
{
    std::ostringstream output_stream;
    output_stream << command_name;

    if (timeout_ms > 0)
    {
        output_stream << "(timeout_ms=" << timeout_ms << ")";
    }

    if (!action_parameters.empty())
    {
        output_stream << " ";
        std::size_t parameter_index = 0;
        for (const auto& parameter_entry : action_parameters)
        {
            if (parameter_index > 0)
            {
                output_stream << ", ";
            }

            output_stream << parameter_entry.first << "=" << parameter_entry.second;
            ++parameter_index;
        }
    }

    return output_stream.str();
}

}  // namespace robot_process_platform::device
