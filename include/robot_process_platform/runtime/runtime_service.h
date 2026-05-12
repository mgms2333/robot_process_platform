#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>

#include "robot_process_platform/core/error_code.h"
#include "robot_process_platform/core/task_types.h"
#include "robot_process_platform/device/interfaces/robot_interface.h"
#include "robot_process_platform/runtime/execution_context.h"
#include "robot_process_platform/runtime/task_runner.h"

namespace robot_process_platform::runtime
{

enum class RuntimeCommandType
{
    LoadTask,
    StartTask,
    PauseTask,
    ResumeTask,
    StopTask,
    EmergencyStop,
    ResetFault,
    Shutdown
};

struct RuntimeCommand
{
    RuntimeCommandType type = RuntimeCommandType::StartTask;
    std::optional<core::Task> task;
    int start_block_index = 0;
    int start_action_index = 0;
};

class RuntimeService
{
public:
    explicit RuntimeService(device::IRobot& robot_instance);
    ~RuntimeService();

    core::Status StartService();
    void StopService();

    core::Status SubmitLoadTask(const core::Task& task,
                                int start_block_index = 0,
                                int start_action_index = 0);
    core::Status SubmitStartTask();
    core::Status SubmitPauseTask();
    core::Status SubmitResumeTask();
    core::Status SubmitStopTask();
    core::Status SubmitEmergencyStop();
    core::Status SubmitResetFault();

    ExecutionContext GetContextSnapshot() const;
    bool IsServiceRunning() const;

private:
    core::Status SubmitCommand(const RuntimeCommand& command);
    void RuntimeLoop();
    void ProcessPendingCommands();
    void ProcessRuntimeCommand(const RuntimeCommand& command);
    void TickIfRunning();

    TaskRunner task_runner;
    ExecutionContext execution_context;
    std::optional<core::Task> loaded_task_storage;

    std::queue<RuntimeCommand> command_queue;
    mutable std::mutex command_lock;
    mutable std::mutex context_lock;
    std::condition_variable command_condition;
    std::thread runtime_thread;

    bool service_running = false;
    bool stop_requested = false;
};

}  // namespace robot_process_platform::runtime
