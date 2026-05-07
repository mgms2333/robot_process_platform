#include "robot_process_platform/device/interfaces/robot_interface.h"

namespace robot_process_platform::device
{

RobotCommandResult::RobotCommandResult(bool success_value, const std::string& message_value)
    :
    success(success_value),
    message(message_value) {}

}  // namespace robot_process_platform::device
