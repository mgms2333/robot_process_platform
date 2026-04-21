#include "robot_process_platform/robot/robot_interface.h"

namespace robot_process_platform::robot
{

RobotCommandResult::RobotCommandResult(bool success_value, const std::string& message_value)
    :
    success(success_value),
    message(message_value) {}

}  // namespace robot_process_platform::robot
