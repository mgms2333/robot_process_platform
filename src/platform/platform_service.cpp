#include "robot_process_platform/platform/platform_service.h"

#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "robot_process_platform/core/json_utils.h"
#include "robot_process_platform/platform/logger.h"

namespace robot_process_platform::platform
{

namespace
{

constexpr const char* kDefaultHyRobotName = "hy_robot";
constexpr const char* kDefaultPlatformConfigFilePath = "./config/platform_config.json";

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

PlatformService::PlatformService()
    : task_manager(template_manager)
{
}

PlatformService::~PlatformService()
{
    Shutdown();
}

core::Status PlatformService::Initialize(bool auto_connect_default_robot)
{
    return Initialize(kDefaultPlatformConfigFilePath, auto_connect_default_robot);
}

core::Status PlatformService::Initialize(const std::string& config_file_path,
                                         bool auto_connect_default_robot)
{
    if (initialized)
    {
        return core::MakeSuccessStatus();
    }

    Logger::GetInstance().LogI("PlatformService", "Initializing platform service.");

    const core::Status register_configured_robot_status =
        RegisterConfiguredHyRobot(config_file_path);
    if (!register_configured_robot_status.Ok())
    {
        return register_configured_robot_status;
    }

    const core::Status rebuild_runtime_status = RebuildRuntimeService();
    if (!rebuild_runtime_status.Ok())
    {
        return rebuild_runtime_status;
    }

    if (auto_connect_default_robot)
    {
        const core::Status connect_default_robot_status = ConnectActiveRobot();
        if (!connect_default_robot_status.Ok())
        {
            return connect_default_robot_status;
        }
    }

    initialized = true;
    Logger::GetInstance().LogI("PlatformService", "Platform service initialized.");
    return core::MakeSuccessStatus();
}

void PlatformService::Shutdown()
{
    if (!initialized)
    {
        return;
    }

    Logger::GetInstance().LogI("PlatformService", "Shutting down platform service.");

    if (runtime_service != nullptr)
    {
        runtime_service->StopService();
        runtime_service.reset();
    }

    initialized = false;
}

PlatformState PlatformService::GetPlatformState() const
{
    PlatformState platform_state;
    platform_state.initialized = initialized;

    if (runtime_service != nullptr)
    {
        platform_state.runtime_service_running = runtime_service->IsServiceRunning();
    }

    platform_state.active_robot_name = device_manager.GetActiveRobotName();
    platform_state.active_camera_name = device_manager.GetActiveCameraName();

    return platform_state;
}

core::Status PlatformService::LoadTemplateLibrary(const std::string& shared_library_path)
{
    return template_manager.LoadTemplateLibrary(shared_library_path);
}

core::Status PlatformService::UnloadTemplate(const std::string& template_name)
{
    return template_manager.UnloadTemplate(template_name);
}

std::vector<TemplateManager::TemplateInfo> PlatformService::GetLoadedTemplates() const
{
    return template_manager.GetLoadedTemplates();
}

core::Status PlatformService::RegisterRobot(const std::string& robot_name,
                                            std::unique_ptr<device::IRobot> robot)
{
    return device_manager.RegisterRobot(robot_name, std::move(robot));
}

core::Status PlatformService::RegisterCamera(const std::string& camera_name,
                                             std::unique_ptr<device::ICamera> camera)
{
    return device_manager.RegisterCamera(camera_name, std::move(camera));
}

core::Status PlatformService::SetActiveRobot(const std::string& robot_name)
{
    const core::Status set_active_robot_status = device_manager.SetActiveRobot(robot_name);
    if (!set_active_robot_status.Ok())
    {
        return set_active_robot_status;
    }

    return RebuildRuntimeService();
}

core::Status PlatformService::SetActiveCamera(const std::string& camera_name)
{
    return device_manager.SetActiveCamera(camera_name);
}

core::Result<device::IRobot*> PlatformService::GetActiveRobot() const
{
    return device_manager.GetActiveRobot();
}

core::Result<device::ICamera*> PlatformService::GetActiveCamera() const
{
    return device_manager.GetActiveCamera();
}

core::Status PlatformService::ConnectActiveRobot()
{
    const auto active_hy_robot_result = GetActiveHyRobot();
    if (!active_hy_robot_result.Ok())
    {
        return core::MakeErrorStatus(active_hy_robot_result.code, active_hy_robot_result.message);
    }

    if (active_hy_robot_result.value.value()->Connect())
    {
        return core::MakeSuccessStatus();
    }

    return core::MakeErrorStatus(core::ErrorCode::InternalError,
                                 "failed_to_connect_active_robot");
}

core::Status PlatformService::DisconnectActiveRobot()
{
    const auto active_hy_robot_result = GetActiveHyRobot();
    if (!active_hy_robot_result.Ok())
    {
        return core::MakeErrorStatus(active_hy_robot_result.code, active_hy_robot_result.message);
    }

    active_hy_robot_result.value.value()->Disconnect();
    return core::MakeSuccessStatus();
}

core::Result<core::Task> PlatformService::CreateTask(const std::string& template_name,
                                                     const std::string& task_context_json,
                                                     const std::string& local_directory_path) const
{
    return task_manager.CreateAndSaveTask(template_name,
                                          task_context_json,
                                          local_directory_path);
}

core::Result<core::Task> PlatformService::LoadTask(const std::string& task_id,
                                                   const std::string& local_directory_path) const
{
    return task_manager.LoadTask(task_id, local_directory_path);
}

core::Status PlatformService::DeleteTask(const std::string& task_id,
                                         const std::string& local_directory_path) const
{
    return task_manager.DeleteTask(task_id, local_directory_path);
}

core::Status PlatformService::StartTask(const std::string& task_id,
                                        const std::string& local_directory_path,
                                        int start_block_index,
                                        int start_action_index)
{
    const core::Result<core::Task> load_task_result = LoadTask(task_id, local_directory_path);
    if (!load_task_result.Ok())
    {
        return core::MakeErrorStatus(load_task_result.code, load_task_result.message);
    }

    const core::Status submit_load_task_status =
        runtime_service->SubmitLoadTask(load_task_result.value.value(),
                                        start_block_index,
                                        start_action_index);
    if (!submit_load_task_status.Ok())
    {
        return submit_load_task_status;
    }

    return runtime_service->SubmitStartTask();
}

core::Status PlatformService::PauseTask()
{
    return runtime_service->SubmitPauseTask();
}

core::Status PlatformService::ResumeTask()
{
    return runtime_service->SubmitResumeTask();
}

core::Status PlatformService::StopTask()
{
    return runtime_service->SubmitStopTask();
}

core::Status PlatformService::EmergencyStopTask()
{
    return runtime_service->SubmitEmergencyStop();
}

core::Status PlatformService::ResetFault()
{
    return runtime_service->SubmitResetFault();
}

runtime::ExecutionContext PlatformService::GetRuntimeSnapshot() const
{
    if (runtime_service == nullptr)
    {
        return runtime::ExecutionContext();
    }

    return runtime_service->GetContextSnapshot();
}

core::Status PlatformService::RegisterConfiguredHyRobot(const std::string& config_file_path)
{
    std::string robot_name = kDefaultHyRobotName;
    device::HyRobotConfig hy_robot_config;

    if (std::filesystem::exists(config_file_path))
    {
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
                const std::string configured_robot_name =
                    core::json::GetStringField(robot_value, "name");
                if (configured_robot_name != active_robot_name)
                {
                    continue;
                }

                const std::string robot_type =
                    core::json::GetStringField(robot_value, "type");
                if (robot_type != "hy_robot")
                {
                    return core::MakeErrorStatus(core::ErrorCode::InvalidArgument,
                                                 "unsupported_active_robot_type: " + robot_type);
                }

                robot_name = configured_robot_name;
                hy_robot_config =
                    BuildHyRobotConfigFromJson(core::json::GetObjectField(robot_value, "config"));
                active_robot_found = true;
                break;
            }

            if (!active_robot_found)
            {
                return core::MakeErrorStatus(core::ErrorCode::DeviceNotFound,
                                             "active_robot_not_found_in_config: " + active_robot_name);
            }

            Logger::GetInstance().LogI("PlatformService",
                                       "Loaded platform device config: " + config_file_path);
        }
        catch (const std::exception& exception)
        {
            return core::MakeErrorStatus(core::ErrorCode::InvalidArgument,
                                         "failed_to_load_platform_config: " +
                                             std::string(exception.what()));
        }
    }
    else
    {
        Logger::GetInstance().LogW("PlatformService",
                                   "Platform config file not found, using built-in mock hy robot defaults: " +
                                       config_file_path);
    }

    const core::Status register_robot_status =
        device_manager.RegisterRobot(robot_name,
                                     std::make_unique<device::HyRobot>(hy_robot_config));
    if (!register_robot_status.Ok() &&
        register_robot_status.code != core::ErrorCode::DeviceAlreadyRegistered)
    {
        return register_robot_status;
    }

    return device_manager.SetActiveRobot(robot_name);
}

core::Status PlatformService::RebuildRuntimeService()
{
    if (runtime_service != nullptr)
    {
        runtime_service->StopService();
        runtime_service.reset();
    }

    const auto active_robot_result = device_manager.GetActiveRobot();
    if (!active_robot_result.Ok())
    {
        return core::MakeErrorStatus(active_robot_result.code,
                                     active_robot_result.message);
    }

    runtime_service =
        std::make_unique<runtime::RuntimeService>(*active_robot_result.value.value());
    return runtime_service->StartService();
}

core::Result<device::HyRobot*> PlatformService::GetActiveHyRobot() const
{
    const auto active_robot_result = device_manager.GetActiveRobot();
    if (!active_robot_result.Ok())
    {
        return core::MakeErrorResult<device::HyRobot*>(active_robot_result.code,
                                                       active_robot_result.message);
    }

    device::HyRobot* hy_robot =
        dynamic_cast<device::HyRobot*>(active_robot_result.value.value());
    if (hy_robot == nullptr)
    {
        return core::MakeErrorResult<device::HyRobot*>(core::ErrorCode::InternalError,
                                                       "active_robot_is_not_hy_robot");
    }

    return core::MakeSuccessResult<device::HyRobot*>(hy_robot);
}

}  // namespace robot_process_platform::platform
