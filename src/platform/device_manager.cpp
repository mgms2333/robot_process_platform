#include "robot_process_platform/platform/device_manager.h"

#include <utility>

#include "robot_process_platform/platform/logger.h"

namespace robot_process_platform::platform
{

core::Status DeviceManager::RegisterRobot(const std::string& robot_name, RobotPtr robot)
{
    if (robot_name.empty() || robot == nullptr)
    {
        Logger::GetInstance().LogE("DeviceManager", "RegisterRobot failed because robot_name is empty or robot is null.");
        return core::MakeErrorStatus(core::ErrorCode::InvalidArgument,
                                     "robot_name_is_empty_or_robot_is_null");
    }

    if (robots.find(robot_name) != robots.end())
    {
        Logger::GetInstance().LogW("DeviceManager", "RegisterRobot failed because robot already exists: " + robot_name);
        return core::MakeErrorStatus(core::ErrorCode::DeviceAlreadyRegistered,
                                     "robot_already_registered: " + robot_name);
    }

    robots.emplace(robot_name, std::move(robot));
    Logger::GetInstance().LogI("DeviceManager", "Robot registered: " + robot_name);
    return core::MakeSuccessStatus();
}

core::Status DeviceManager::UnregisterRobot(const std::string& robot_name)
{
    const auto robot_iterator = robots.find(robot_name);
    if (robot_iterator == robots.end())
    {
        Logger::GetInstance().LogW("DeviceManager", "UnregisterRobot failed because robot was not found: " + robot_name);
        return core::MakeErrorStatus(core::ErrorCode::DeviceNotFound,
                                     "robot_not_found: " + robot_name);
    }

    robots.erase(robot_iterator);
    if (active_robot_name == robot_name)
    {
        active_robot_name.clear();
    }

    Logger::GetInstance().LogI("DeviceManager", "Robot unregistered: " + robot_name);
    return core::MakeSuccessStatus();
}

core::Result<device::IRobot*> DeviceManager::GetRobot(const std::string& robot_name) const
{
    const auto robot_iterator = robots.find(robot_name);
    if (robot_iterator == robots.end())
    {
        Logger::GetInstance().LogW("DeviceManager", "GetRobot failed because robot was not found: " + robot_name);
        return core::MakeErrorResult<device::IRobot*>(core::ErrorCode::DeviceNotFound,
                                                      "robot_not_found: " + robot_name);
    }

    return core::MakeSuccessResult<device::IRobot*>(robot_iterator->second.get());
}

core::Status DeviceManager::SetActiveRobot(const std::string& robot_name)
{
    const auto robot_iterator = robots.find(robot_name);
    if (robot_iterator == robots.end())
    {
        Logger::GetInstance().LogW("DeviceManager", "SetActiveRobot failed because robot was not found: " + robot_name);
        return core::MakeErrorStatus(core::ErrorCode::DeviceNotFound,
                                     "robot_not_found: " + robot_name);
    }

    active_robot_name = robot_name;
    Logger::GetInstance().LogI("DeviceManager", "Active robot selected: " + robot_name);
    return core::MakeSuccessStatus();
}

core::Result<device::IRobot*> DeviceManager::GetActiveRobot() const
{
    if (active_robot_name.empty())
    {
        Logger::GetInstance().LogW("DeviceManager", "GetActiveRobot failed because no active robot is selected.");
        return core::MakeErrorResult<device::IRobot*>(core::ErrorCode::DeviceActiveRobotNotSet,
                                                      "active_robot_is_not_set");
    }

    return GetRobot(active_robot_name);
}

core::Status DeviceManager::RegisterCamera(const std::string& camera_name, CameraPtr camera)
{
    if (camera_name.empty() || camera == nullptr)
    {
        Logger::GetInstance().LogE("DeviceManager", "RegisterCamera failed because camera_name is empty or camera is null.");
        return core::MakeErrorStatus(core::ErrorCode::InvalidArgument,
                                     "camera_name_is_empty_or_camera_is_null");
    }

    if (cameras.find(camera_name) != cameras.end())
    {
        Logger::GetInstance().LogW("DeviceManager", "RegisterCamera failed because camera already exists: " + camera_name);
        return core::MakeErrorStatus(core::ErrorCode::DeviceAlreadyRegistered,
                                     "camera_already_registered: " + camera_name);
    }

    cameras.emplace(camera_name, std::move(camera));
    Logger::GetInstance().LogI("DeviceManager", "Camera registered: " + camera_name);
    return core::MakeSuccessStatus();
}

core::Status DeviceManager::UnregisterCamera(const std::string& camera_name)
{
    const auto camera_iterator = cameras.find(camera_name);
    if (camera_iterator == cameras.end())
    {
        Logger::GetInstance().LogW("DeviceManager", "UnregisterCamera failed because camera was not found: " + camera_name);
        return core::MakeErrorStatus(core::ErrorCode::DeviceNotFound,
                                     "camera_not_found: " + camera_name);
    }

    cameras.erase(camera_iterator);
    if (active_camera_name == camera_name)
    {
        active_camera_name.clear();
    }

    Logger::GetInstance().LogI("DeviceManager", "Camera unregistered: " + camera_name);
    return core::MakeSuccessStatus();
}

core::Result<device::ICamera*> DeviceManager::GetCamera(const std::string& camera_name) const
{
    const auto camera_iterator = cameras.find(camera_name);
    if (camera_iterator == cameras.end())
    {
        Logger::GetInstance().LogW("DeviceManager", "GetCamera failed because camera was not found: " + camera_name);
        return core::MakeErrorResult<device::ICamera*>(core::ErrorCode::DeviceNotFound,
                                                       "camera_not_found: " + camera_name);
    }

    return core::MakeSuccessResult<device::ICamera*>(camera_iterator->second.get());
}

core::Status DeviceManager::SetActiveCamera(const std::string& camera_name)
{
    const auto camera_iterator = cameras.find(camera_name);
    if (camera_iterator == cameras.end())
    {
        Logger::GetInstance().LogW("DeviceManager", "SetActiveCamera failed because camera was not found: " + camera_name);
        return core::MakeErrorStatus(core::ErrorCode::DeviceNotFound,
                                     "camera_not_found: " + camera_name);
    }

    active_camera_name = camera_name;
    Logger::GetInstance().LogI("DeviceManager", "Active camera selected: " + camera_name);
    return core::MakeSuccessStatus();
}

core::Result<device::ICamera*> DeviceManager::GetActiveCamera() const
{
    if (active_camera_name.empty())
    {
        Logger::GetInstance().LogW("DeviceManager", "GetActiveCamera failed because no active camera is selected.");
        return core::MakeErrorResult<device::ICamera*>(core::ErrorCode::DeviceActiveCameraNotSet,
                                                       "active_camera_is_not_set");
    }

    return GetCamera(active_camera_name);
}

}  // namespace robot_process_platform::platform
