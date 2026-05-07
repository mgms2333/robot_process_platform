#include "robot_process_platform/device/interfaces/camera_interface.h"

namespace robot_process_platform::device
{

CameraCommandResult::CameraCommandResult(
    bool success_value,
    const std::string& message_value,
    const std::map<std::string, std::string>& result_parameters_value)
    :
    success(success_value),
    message(message_value),
    result_parameters(result_parameters_value) {}

}  // namespace robot_process_platform::device
