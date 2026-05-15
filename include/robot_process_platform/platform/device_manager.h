#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "robot_process_platform/core/error_code.h"
#include "robot_process_platform/device/interfaces/camera_interface.h"
#include "robot_process_platform/device/interfaces/robot_interface.h"

namespace robot_process_platform::platform
{

class DeviceManager
{
public:
    using RobotPtr = std::unique_ptr<device::IRobot>;
    using CameraPtr = std::unique_ptr<device::ICamera>;

    DeviceManager() = default;
    ~DeviceManager() = default;

    core::Status RegisterRobot(const std::string& robot_name, RobotPtr robot);
    core::Status UnregisterRobot(const std::string& robot_name);
    core::Result<device::IRobot*> GetRobot(const std::string& robot_name) const;
    core::Status SetActiveRobot(const std::string& robot_name);
    core::Result<device::IRobot*> GetActiveRobot() const;

    core::Status RegisterCamera(const std::string& camera_name, CameraPtr camera);
    core::Status UnregisterCamera(const std::string& camera_name);
    core::Result<device::ICamera*> GetCamera(const std::string& camera_name) const;
    core::Status SetActiveCamera(const std::string& camera_name);
    core::Result<device::ICamera*> GetActiveCamera() const;

private:
    std::unordered_map<std::string, RobotPtr> robots;
    std::unordered_map<std::string, CameraPtr> cameras;
    std::string active_robot_name;
    std::string active_camera_name;
};

}  // namespace robot_process_platform::platform
