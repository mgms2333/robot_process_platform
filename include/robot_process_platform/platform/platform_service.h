#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "robot_process_platform/core/error_code.h"
#include "robot_process_platform/core/task_types.h"
#include "robot_process_platform/device/interfaces/camera_interface.h"
#include "robot_process_platform/device/interfaces/robot_interface.h"
#include "robot_process_platform/device/robot/hy_robot.h"
#include "robot_process_platform/platform/device_manager.h"
#include "robot_process_platform/platform/task_manager.h"
#include "robot_process_platform/platform/template_manager.h"
#include "robot_process_platform/runtime/execution_context.h"
#include "robot_process_platform/runtime/runtime_service.h"

namespace robot_process_platform::platform
{

struct PlatformState
{
    bool initialized = false;
    bool runtime_service_running = false;
    std::string active_robot_name;
    std::string active_camera_name;
};

class PlatformService
{
public:
    PlatformService();
    ~PlatformService();

    core::Status Initialize(bool auto_connect_default_robot = false);
    core::Status Initialize(const std::string& config_file_path,
                            bool auto_connect_default_robot = false);
    void Shutdown();

    PlatformState GetPlatformState() const;

    core::Status LoadTemplateLibrary(const std::string& shared_library_path);
    core::Status UnloadTemplate(const std::string& template_name);
    std::vector<TemplateManager::TemplateInfo> GetLoadedTemplates() const;

    core::Status RegisterRobot(const std::string& robot_name,
                               std::unique_ptr<device::IRobot> robot);
    core::Status RegisterCamera(const std::string& camera_name,
                                std::unique_ptr<device::ICamera> camera);
    core::Status SetActiveRobot(const std::string& robot_name);
    core::Status SetActiveCamera(const std::string& camera_name);
    core::Result<device::IRobot*> GetActiveRobot() const;
    core::Result<device::ICamera*> GetActiveCamera() const;
    core::Status ConnectActiveRobot();
    core::Status DisconnectActiveRobot();

    core::Result<core::Task> CreateTask(const std::string& template_name,
                                        const std::string& task_context_json,
                                        const std::string& local_directory_path) const;
    core::Result<core::Task> LoadTask(const std::string& task_id,
                                      const std::string& local_directory_path) const;
    core::Status DeleteTask(const std::string& task_id,
                            const std::string& local_directory_path) const;

    core::Status StartTask(const std::string& task_id,
                           const std::string& local_directory_path,
                           int start_block_index = 0,
                           int start_action_index = 0);
    core::Status PauseTask();
    core::Status ResumeTask();
    core::Status StopTask();
    core::Status EmergencyStopTask();
    core::Status ResetFault();
    runtime::ExecutionContext GetRuntimeSnapshot() const;

private:
    core::Status RegisterConfiguredHyRobot(const std::string& config_file_path);
    core::Status RebuildRuntimeService();
    core::Result<device::HyRobot*> GetActiveHyRobot() const;

    TemplateManager template_manager;
    TaskManager task_manager;
    DeviceManager device_manager;
    std::unique_ptr<runtime::RuntimeService> runtime_service;
    bool initialized = false;
};

}  // namespace robot_process_platform::platform
