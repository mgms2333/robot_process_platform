#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
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

struct SystemState
{
    bool initialized = false;
    bool runtime_service_running = false;
    std::string active_robot_name;
    std::string active_camera_name;
};

struct ConfiguredRobotInfo
{
    std::string robot_name;
    std::string robot_type;
    bool is_default = false;
};

using RuntimeFactCallback = std::function<void(const runtime::RuntimeFact&)>;

class SystemService
{
public:
    SystemService();
    ~SystemService();

    core::Status Initialize(bool auto_connect_default_robot = false);
    core::Status Initialize(const std::string& config_file_path,
                            bool auto_connect_default_robot = false);
    void Shutdown();

    SystemState GetSystemState() const;
    std::vector<ConfiguredRobotInfo> GetConfiguredRobots() const;
    std::string GetDefaultRobotName() const;
    std::string GetSelectedRobotName() const;

    core::Status LoadTemplateLibrary(const std::string& shared_library_path);
    core::Status UnloadTemplate(const std::string& template_name);
    std::vector<TemplateManager::TemplateInfo> GetLoadedTemplates() const;

    core::Status RegisterRobot(const std::string& robot_name,
                               std::unique_ptr<device::IRobot> robot);
    core::Status RegisterCamera(const std::string& camera_name,
                                std::unique_ptr<device::ICamera> camera);
    core::Status SelectConfiguredRobot(const std::string& robot_name);
    core::Status ConnectSelectedRobot();
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
    void SetRuntimeFactCallback(RuntimeFactCallback runtime_fact_callback_value);

private:
    void HandleRuntimeFact(const runtime::RuntimeFact& runtime_fact) const;
    core::Status LoadConfiguredRobots(const std::string& config_file_path);
    core::Status RebuildRuntimeService();
    core::Result<device::HyRobot*> GetActiveHyRobot() const;

    TemplateManager template_manager;
    TaskManager task_manager;
    DeviceManager device_manager;
    std::unique_ptr<runtime::RuntimeService> runtime_service;
    mutable std::mutex runtime_fact_callback_lock;
    RuntimeFactCallback runtime_fact_callback;
    std::vector<ConfiguredRobotInfo> configured_robots;
    std::unordered_map<std::string, device::HyRobotConfig> configured_hy_robot_configs;
    std::string default_robot_name;
    std::string selected_robot_name;
    bool initialized = false;
};

}  // namespace robot_process_platform::platform
