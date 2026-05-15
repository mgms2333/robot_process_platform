#include "robot_process_platform/platform/platform_service.h"

#include <utility>

#include "robot_process_platform/platform/logger.h"

namespace robot_process_platform::platform
{

namespace
{

constexpr const char* kDefaultHyRobotName = "hy_robot";
constexpr const char* kDefaultHyRobotHostName = "10.20.0.209";

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
    if (initialized)
    {
        return core::MakeSuccessStatus();
    }

    Logger::GetInstance().LogI("PlatformService", "Initializing platform service.");

    const core::Status register_default_robot_status = RegisterDefaultHyRobot();
    if (!register_default_robot_status.Ok())
    {
        return register_default_robot_status;
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

    const auto active_robot_result = device_manager.GetActiveRobot();
    if (active_robot_result.Ok())
    {
        platform_state.active_robot_name = kDefaultHyRobotName;
    }

    const auto active_camera_result = device_manager.GetActiveCamera();
    if (active_camera_result.Ok())
    {
        platform_state.active_camera_name = "active_camera";
    }

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

core::Status PlatformService::StartTask(const std::string& template_name,
                                        const std::string& task_context_json,
                                        const std::string& local_directory_path)
{
    const core::Result<core::Task> create_task_result =
        CreateTask(template_name, task_context_json, local_directory_path);
    if (!create_task_result.Ok())
    {
        return core::MakeErrorStatus(create_task_result.code, create_task_result.message);
    }

    return runtime_service->SubmitLoadTask(create_task_result.value.value());
}

core::Status PlatformService::StartTaskById(const std::string& task_id,
                                            const std::string& local_directory_path)
{
    const core::Result<core::Task> load_task_result = LoadTask(task_id, local_directory_path);
    if (!load_task_result.Ok())
    {
        return core::MakeErrorStatus(load_task_result.code, load_task_result.message);
    }

    const core::Status submit_load_task_status =
        runtime_service->SubmitLoadTask(load_task_result.value.value());
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

core::Status PlatformService::RegisterDefaultHyRobot()
{
    device::HyRobotConfig hy_robot_config;
    hy_robot_config.host_name = kDefaultHyRobotHostName;

    const core::Status register_robot_status =
        device_manager.RegisterRobot(kDefaultHyRobotName,
                                     std::make_unique<device::HyRobot>(hy_robot_config));
    if (!register_robot_status.Ok() &&
        register_robot_status.code != core::ErrorCode::DeviceAlreadyRegistered)
    {
        return register_robot_status;
    }

    return device_manager.SetActiveRobot(kDefaultHyRobotName);
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
