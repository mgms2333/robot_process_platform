#include "robot_process_platform/platform/system_config.h"

#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

#include "robot_process_platform/core/json_utils.h"

namespace robot_process_platform::platform
{

namespace
{

constexpr const char* kDefaultHyRobotName = "hy_robot";

std::string ReadAllText(const std::string& file_path)
{
    std::ifstream input_file(file_path);
    if (!input_file.is_open())
    {
        throw std::runtime_error("failed_to_open_file: " + file_path);
    }

    std::ostringstream text_stream;
    text_stream << input_file.rdbuf();
    return text_stream.str();
}

bool HasObjectField(const core::json::JsonValue& object_value, const std::string& field_name)
{
    return object_value.type == core::json::JsonValueType::Object &&
           object_value.object_value.find(field_name) != object_value.object_value.end();
}

std::string GetOptionalStringField(const core::json::JsonValue& object_value,
                                   const std::string& field_name,
                                   const std::string& default_value)
{
    if (!HasObjectField(object_value, field_name))
    {
        return default_value;
    }

    return core::json::GetStringField(object_value, field_name);
}

bool GetOptionalBoolField(const core::json::JsonValue& object_value,
                          const std::string& field_name,
                          bool default_value)
{
    if (!HasObjectField(object_value, field_name))
    {
        return default_value;
    }

    const core::json::JsonValue& field_value = core::json::GetObjectField(object_value, field_name);
    if (field_value.type != core::json::JsonValueType::Bool)
    {
        throw std::runtime_error("json_field_is_not_bool: " + field_name);
    }

    return field_value.bool_value;
}

int GetOptionalIntField(const core::json::JsonValue& object_value,
                        const std::string& field_name,
                        int default_value)
{
    if (!HasObjectField(object_value, field_name))
    {
        return default_value;
    }

    return core::json::GetIntField(object_value, field_name);
}

double GetOptionalDoubleField(const core::json::JsonValue& object_value,
                              const std::string& field_name,
                              double default_value)
{
    if (!HasObjectField(object_value, field_name))
    {
        return default_value;
    }

    return std::stod(core::json::GetNumberFieldAsString(object_value, field_name));
}

std::map<std::string, int> GetOptionalIntMapField(const core::json::JsonValue& object_value,
                                                  const std::string& field_name)
{
    std::map<std::string, int> output_map;
    if (!HasObjectField(object_value, field_name))
    {
        return output_map;
    }

    const core::json::JsonValue& field_value = core::json::GetObjectField(object_value, field_name);
    if (field_value.type != core::json::JsonValueType::Object)
    {
        throw std::runtime_error("json_field_is_not_object: " + field_name);
    }

    for (const auto& field_entry : field_value.object_value)
    {
        if (field_entry.second.type != core::json::JsonValueType::Number)
        {
            throw std::runtime_error("json_map_value_is_not_number: " + field_name);
        }

        output_map[field_entry.first] = std::stoi(field_entry.second.number_value);
    }

    return output_map;
}

device::HyRobotConfig BuildHyRobotConfigFromJson(const core::json::JsonValue& config_value)
{
    device::HyRobotConfig hy_robot_config;
    hy_robot_config.enable_sdk =
        GetOptionalBoolField(config_value, "enable_sdk", hy_robot_config.enable_sdk);
    hy_robot_config.auto_connect_controller =
        GetOptionalBoolField(config_value,
                             "auto_connect_controller",
                             hy_robot_config.auto_connect_controller);
    hy_robot_config.auto_electrify =
        GetOptionalBoolField(config_value, "auto_electrify", hy_robot_config.auto_electrify);
    hy_robot_config.box_id =
        static_cast<unsigned int>(GetOptionalIntField(config_value, "box_id", hy_robot_config.box_id));
    hy_robot_config.robot_id =
        static_cast<unsigned int>(GetOptionalIntField(config_value, "robot_id", hy_robot_config.robot_id));
    hy_robot_config.port =
        static_cast<unsigned short>(GetOptionalIntField(config_value, "port", hy_robot_config.port));
    hy_robot_config.motion_done_timeout_ms =
        GetOptionalIntField(config_value,
                            "motion_done_timeout_ms",
                            hy_robot_config.motion_done_timeout_ms);
    hy_robot_config.io_poll_interval_ms =
        GetOptionalIntField(config_value,
                            "io_poll_interval_ms",
                            hy_robot_config.io_poll_interval_ms);
    hy_robot_config.default_velocity =
        GetOptionalDoubleField(config_value, "default_velocity", hy_robot_config.default_velocity);
    hy_robot_config.default_acceleration =
        GetOptionalDoubleField(config_value,
                               "default_acceleration",
                               hy_robot_config.default_acceleration);
    hy_robot_config.default_radius =
        GetOptionalDoubleField(config_value, "default_radius", hy_robot_config.default_radius);
    hy_robot_config.host_name =
        GetOptionalStringField(config_value, "host_name", hy_robot_config.host_name);
    hy_robot_config.tcp_name =
        GetOptionalStringField(config_value, "tcp_name", hy_robot_config.tcp_name);
    hy_robot_config.ucs_name =
        GetOptionalStringField(config_value, "ucs_name", hy_robot_config.ucs_name);
    hy_robot_config.box_di_bits = GetOptionalIntMapField(config_value, "box_di_bits");
    hy_robot_config.box_do_bits = GetOptionalIntMapField(config_value, "box_do_bits");
    return hy_robot_config;
}

}  // namespace

core::Result<SystemConfig> LoadSystemConfig(const std::string& config_file_path)
{
    SystemConfig system_config;

    if (!std::filesystem::exists(config_file_path))
    {
        ConfiguredRobotConfig default_robot;
        default_robot.robot_name = kDefaultHyRobotName;
        default_robot.robot_type = "hy_robot";
        default_robot.is_default = true;
        default_robot.hy_robot_config = device::HyRobotConfig();
        system_config.configured_robots.push_back(default_robot);
        system_config.default_robot_name = kDefaultHyRobotName;
        system_config.selected_robot_name = kDefaultHyRobotName;
        return core::MakeSuccessResult<SystemConfig>(system_config);
    }

    try
    {
        const core::json::JsonValue root_value =
            core::json::JsonParser(ReadAllText(config_file_path)).Parse();
        const core::json::JsonValue& devices_value =
            core::json::GetObjectField(root_value, "devices");
        const std::string active_robot_name =
            core::json::GetStringField(devices_value, "active_robot");
        const std::vector<core::json::JsonValue>& robot_values =
            core::json::GetArrayField(devices_value, "robots");

        bool active_robot_found = false;
        for (const core::json::JsonValue& robot_value : robot_values)
        {
            ConfiguredRobotConfig configured_robot;
            configured_robot.robot_name = core::json::GetStringField(robot_value, "name");
            configured_robot.robot_type = core::json::GetStringField(robot_value, "type");
            configured_robot.is_default =
                GetOptionalBoolField(robot_value,
                                     "default",
                                     configured_robot.robot_name == active_robot_name);

            if (configured_robot.robot_type == "hy_robot")
            {
                configured_robot.hy_robot_config =
                    BuildHyRobotConfigFromJson(core::json::GetObjectField(robot_value, "config"));
            }

            system_config.configured_robots.push_back(configured_robot);

            if (configured_robot.is_default && system_config.default_robot_name.empty())
            {
                system_config.default_robot_name = configured_robot.robot_name;
            }

            if (configured_robot.robot_name == active_robot_name)
            {
                active_robot_found = true;
            }
        }

        if (!active_robot_found)
        {
            return core::MakeErrorResult<SystemConfig>(
                core::ErrorCode::DeviceNotFound,
                "active_robot_not_found_in_config: " + active_robot_name);
        }

        if (!system_config.default_robot_name.empty())
        {
            system_config.selected_robot_name = system_config.default_robot_name;
        }
        else if (!system_config.configured_robots.empty())
        {
            system_config.selected_robot_name = system_config.configured_robots.front().robot_name;
        }

        return core::MakeSuccessResult<SystemConfig>(system_config);
    }
    catch (const std::exception& exception)
    {
        return core::MakeErrorResult<SystemConfig>(
            core::ErrorCode::InvalidArgument,
            "failed_to_load_system_config: " + std::string(exception.what()));
    }
}

}  // namespace robot_process_platform::platform
