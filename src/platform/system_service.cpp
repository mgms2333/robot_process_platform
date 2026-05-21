#include "robot_process_platform/platform/system_service.h"

#include <utility>

#include "robot_process_platform/platform/logger.h"
#include "robot_process_platform/platform/system_config.h"

namespace robot_process_platform::platform
{

namespace
{

constexpr const char* kDefaultSystemConfigFilePath = "./config/system_config.json";

}  // namespace

SystemService::SystemService()
    : task_manager(template_manager)
{
}

SystemService::~SystemService()
{
    Shutdown();
}

core::Status SystemService::Initialize(bool auto_connect_default_robot)
{
    return Initialize(kDefaultSystemConfigFilePath, auto_connect_default_robot);
}

core::Status SystemService::Initialize(const std::string& config_file_path,
                                       bool auto_connect_default_robot)
{
    if (initialized)
    {
        return core::MakeSuccessStatus();
    }

    Logger::GetInstance().LogI("SystemService", "Initializing system service.");

    const core::Status load_configured_robots_status =
        LoadConfiguredRobots(config_file_path);
    if (!load_configured_robots_status.Ok())
    {
        return load_configured_robots_status;
    }

    if (auto_connect_default_robot)
    {
        const core::Status connect_default_robot_status = ConnectSelectedRobot();
        if (!connect_default_robot_status.Ok())
        {
            return connect_default_robot_status;
        }
    }

    initialized = true;
    Logger::GetInstance().LogI("SystemService", "System service initialized.");
    return core::MakeSuccessStatus();
}

void SystemService::Shutdown()
{
    if (!initialized)
    {
        return;
    }

    Logger::GetInstance().LogI("SystemService", "Shutting down system service.");

    if (runtime_service != nullptr)
    {
        runtime_service->StopService();
        runtime_service.reset();
    }

    initialized = false;
}

SystemState SystemService::GetSystemState() const
{
    SystemState system_state;
    system_state.initialized = initialized;

    if (runtime_service != nullptr)
    {
        system_state.runtime_service_running = runtime_service->IsServiceRunning();
    }

    system_state.active_robot_name = device_manager.GetActiveRobotName();
    system_state.active_camera_name = device_manager.GetActiveCameraName();

    return system_state;
}

std::vector<ConfiguredRobotInfo> SystemService::GetConfiguredRobots() const
{
    return configured_robots;
}

std::string SystemService::GetDefaultRobotName() const
{
    return default_robot_name;
}

std::string SystemService::GetSelectedRobotName() const
{
    return selected_robot_name;
}

core::Status SystemService::LoadTemplateLibrary(const std::string& shared_library_path)
{
    return template_manager.LoadTemplateLibrary(shared_library_path);
}

core::Status SystemService::UnloadTemplate(const std::string& template_name)
{
    return template_manager.UnloadTemplate(template_name);
}

std::vector<TemplateManager::TemplateInfo> SystemService::GetLoadedTemplates() const
{
    return template_manager.GetLoadedTemplates();
}

core::Status SystemService::RegisterRobot(const std::string& robot_name,
                                          std::unique_ptr<device::IRobot> robot)
{
    return device_manager.RegisterRobot(robot_name, std::move(robot));
}

core::Status SystemService::RegisterCamera(const std::string& camera_name,
                                           std::unique_ptr<device::ICamera> camera)
{
    return device_manager.RegisterCamera(camera_name, std::move(camera));
}

core::Status SystemService::SelectConfiguredRobot(const std::string& robot_name)
{
    for (const ConfiguredRobotInfo& configured_robot : configured_robots)
    {
        if (configured_robot.robot_name == robot_name)
        {
            selected_robot_name = robot_name;
            return core::MakeSuccessStatus();
        }
    }

    return core::MakeErrorStatus(core::ErrorCode::DeviceNotFound,
                                 "configured_robot_not_found: " + robot_name);
}

core::Status SystemService::ConnectSelectedRobot()
{
    if (selected_robot_name.empty())
    {
        return core::MakeErrorStatus(core::ErrorCode::DeviceNotFound,
                                     "selected_robot_is_not_set");
    }

    std::optional<ConfiguredRobotInfo> selected_robot_info;
    for (const ConfiguredRobotInfo& configured_robot : configured_robots)
    {
        if (configured_robot.robot_name == selected_robot_name)
        {
            selected_robot_info = configured_robot;
            break;
        }
    }

    if (!selected_robot_info.has_value())
    {
        return core::MakeErrorStatus(core::ErrorCode::DeviceNotFound,
                                     "selected_robot_not_found: " + selected_robot_name);
    }

    if (runtime_service != nullptr)
    {
        runtime_service->StopService();
        runtime_service.reset();
    }

    if (selected_robot_info->robot_type != "hy_robot")
    {
        return core::MakeErrorStatus(core::ErrorCode::InvalidArgument,
                                     "unsupported_selected_robot_type: " +
                                         selected_robot_info->robot_type);
    }

    const auto configured_hy_robot_config_iterator =
        configured_hy_robot_configs.find(selected_robot_name);
    if (configured_hy_robot_config_iterator == configured_hy_robot_configs.end())
    {
        return core::MakeErrorStatus(core::ErrorCode::DeviceNotFound,
                                     "selected_hy_robot_config_not_found: " + selected_robot_name);
    }

    const auto existing_robot_result = device_manager.GetRobot(selected_robot_name);
    if (existing_robot_result.Ok())
    {
        const core::Status unregister_robot_status =
            device_manager.UnregisterRobot(selected_robot_name);
        if (!unregister_robot_status.Ok())
        {
            return unregister_robot_status;
        }
    }

    std::unique_ptr<device::HyRobot> hy_robot =
        std::make_unique<device::HyRobot>(configured_hy_robot_config_iterator->second);
    if (!hy_robot->Connect())
    {
        return core::MakeErrorStatus(core::ErrorCode::InternalError,
                                     "failed_to_connect_selected_robot: " + selected_robot_name);
    }

    const core::Status register_robot_status =
        device_manager.RegisterRobot(selected_robot_name, std::move(hy_robot));
    if (!register_robot_status.Ok())
    {
        return register_robot_status;
    }

    const core::Status set_active_robot_status =
        device_manager.SetActiveRobot(selected_robot_name);
    if (!set_active_robot_status.Ok())
    {
        return set_active_robot_status;
    }

    return RebuildRuntimeService();
}

core::Status SystemService::SetActiveRobot(const std::string& robot_name)
{
    const core::Status set_active_robot_status = device_manager.SetActiveRobot(robot_name);
    if (!set_active_robot_status.Ok())
    {
        return set_active_robot_status;
    }

    return RebuildRuntimeService();
}

core::Status SystemService::SetActiveCamera(const std::string& camera_name)
{
    return device_manager.SetActiveCamera(camera_name);
}

core::Result<device::IRobot*> SystemService::GetActiveRobot() const
{
    return device_manager.GetActiveRobot();
}

core::Result<device::ICamera*> SystemService::GetActiveCamera() const
{
    return device_manager.GetActiveCamera();
}

core::Status SystemService::ConnectActiveRobot()
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

core::Status SystemService::DisconnectActiveRobot()
{
    const auto active_hy_robot_result = GetActiveHyRobot();
    if (!active_hy_robot_result.Ok())
    {
        return core::MakeErrorStatus(active_hy_robot_result.code, active_hy_robot_result.message);
    }

    active_hy_robot_result.value.value()->Disconnect();
    return core::MakeSuccessStatus();
}

core::Result<core::Task> SystemService::CreateTask(const std::string& template_name,
                                                   const std::string& task_context_json,
                                                   const std::string& local_directory_path) const
{
    return task_manager.CreateAndSaveTask(template_name,
                                          task_context_json,
                                          local_directory_path);
}

core::Result<core::Task> SystemService::LoadTask(const std::string& task_id,
                                                 const std::string& local_directory_path) const
{
    return task_manager.LoadTask(task_id, local_directory_path);
}

core::Status SystemService::DeleteTask(const std::string& task_id,
                                       const std::string& local_directory_path) const
{
    return task_manager.DeleteTask(task_id, local_directory_path);
}

core::Status SystemService::StartTask(const std::string& task_id,
                                      const std::string& local_directory_path,
                                      int start_block_index,
                                      int start_action_index)
{
    if (runtime_service == nullptr)
    {
        return core::MakeErrorStatus(core::ErrorCode::RuntimeServiceNotRunning,
                                     "runtime_service_is_not_running");
    }

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

core::Status SystemService::PauseTask()
{
    if (runtime_service == nullptr)
    {
        return core::MakeErrorStatus(core::ErrorCode::RuntimeServiceNotRunning,
                                     "runtime_service_is_not_running");
    }

    return runtime_service->SubmitPauseTask();
}

core::Status SystemService::ResumeTask()
{
    if (runtime_service == nullptr)
    {
        return core::MakeErrorStatus(core::ErrorCode::RuntimeServiceNotRunning,
                                     "runtime_service_is_not_running");
    }

    return runtime_service->SubmitResumeTask();
}

core::Status SystemService::StopTask()
{
    if (runtime_service == nullptr)
    {
        return core::MakeErrorStatus(core::ErrorCode::RuntimeServiceNotRunning,
                                     "runtime_service_is_not_running");
    }

    return runtime_service->SubmitStopTask();
}

core::Status SystemService::EmergencyStopTask()
{
    if (runtime_service == nullptr)
    {
        return core::MakeErrorStatus(core::ErrorCode::RuntimeServiceNotRunning,
                                     "runtime_service_is_not_running");
    }

    return runtime_service->SubmitEmergencyStop();
}

core::Status SystemService::ResetFault()
{
    if (runtime_service == nullptr)
    {
        return core::MakeErrorStatus(core::ErrorCode::RuntimeServiceNotRunning,
                                     "runtime_service_is_not_running");
    }

    return runtime_service->SubmitResetFault();
}

runtime::ExecutionContext SystemService::GetRuntimeSnapshot() const
{
    if (runtime_service == nullptr)
    {
        return runtime::ExecutionContext();
    }

    return runtime_service->GetContextSnapshot();
}

void SystemService::SetRuntimeFactCallback(RuntimeFactCallback runtime_fact_callback_value)
{
    {
        std::lock_guard<std::mutex> runtime_fact_callback_guard(runtime_fact_callback_lock);
        runtime_fact_callback = std::move(runtime_fact_callback_value);
    }

    if (runtime_service != nullptr)
    {
        runtime_service->SetRuntimeFactCallback(
            [this](const runtime::RuntimeFact& runtime_fact)
            {
                HandleRuntimeFact(runtime_fact);
            });
    }
}

void SystemService::HandleRuntimeFact(const runtime::RuntimeFact& runtime_fact) const
{
    Logger::GetInstance().LogI("SystemService",
                               "Runtime fact received: type=" + runtime::ToString(runtime_fact.type) +
                                   " state_before=" + runtime::ToString(runtime_fact.state_before) +
                                   " state_after=" + runtime::ToString(runtime_fact.state_after));

    RuntimeFactCallback current_runtime_fact_callback;
    {
        std::lock_guard<std::mutex> runtime_fact_callback_guard(runtime_fact_callback_lock);
        current_runtime_fact_callback = runtime_fact_callback;
    }

    if (current_runtime_fact_callback)
    {
        current_runtime_fact_callback(runtime_fact);
    }
}

core::Status SystemService::LoadConfiguredRobots(const std::string& config_file_path)
{
    const core::Result<SystemConfig> load_system_config_result =
        LoadSystemConfig(config_file_path);
    if (!load_system_config_result.Ok())
    {
        return core::MakeErrorStatus(load_system_config_result.code,
                                     load_system_config_result.message);
    }

    configured_robots.clear();
    configured_hy_robot_configs.clear();
    default_robot_name = load_system_config_result.value->default_robot_name;
    selected_robot_name = load_system_config_result.value->selected_robot_name;

    for (const ConfiguredRobotConfig& configured_robot : load_system_config_result.value->configured_robots)
    {
        configured_robots.push_back(
            ConfiguredRobotInfo{configured_robot.robot_name,
                                configured_robot.robot_type,
                                configured_robot.is_default});

        if (configured_robot.robot_type == "hy_robot" && configured_robot.hy_robot_config.has_value())
        {
            configured_hy_robot_configs[configured_robot.robot_name] =
                configured_robot.hy_robot_config.value();
        }
    }

    Logger::GetInstance().LogI("SystemService",
                               "Loaded system device config: " + config_file_path);
    return core::MakeSuccessStatus();
}

core::Status SystemService::RebuildRuntimeService()
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
    runtime_service->SetRuntimeFactCallback(
        [this](const runtime::RuntimeFact& runtime_fact)
        {
            HandleRuntimeFact(runtime_fact);
        });
    return runtime_service->StartService();
}

core::Result<device::HyRobot*> SystemService::GetActiveHyRobot() const
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
