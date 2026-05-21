#pragma once

#include <optional>
#include <string>
#include <vector>

#include "robot_process_platform/core/error_code.h"
#include "robot_process_platform/device/robot/hy_robot.h"

namespace robot_process_platform::platform
{

struct ConfiguredRobotConfig
{
    std::string robot_name;
    std::string robot_type;
    bool is_default = false;
    std::optional<device::HyRobotConfig> hy_robot_config;
};

struct SystemConfig
{
    std::vector<ConfiguredRobotConfig> configured_robots;
    std::string default_robot_name;
    std::string selected_robot_name;
};

core::Result<SystemConfig> LoadSystemConfig(const std::string& config_file_path);

}  // namespace robot_process_platform::platform
