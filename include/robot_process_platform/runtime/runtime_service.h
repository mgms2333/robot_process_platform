#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

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

enum class RuntimeFactType
{
    StateChanged,
    TaskLoaded,
    TaskStarted,
    TaskPaused,
    TaskResumed,
    TaskStopped,
    TaskCompleted,
    TaskFailed,
    TaskEmergencyStopped,
    FaultReset
};

struct RuntimeFact
{
    RuntimeFactType type = RuntimeFactType::StateChanged;
    RuntimeState state_before = RuntimeState::Idle;
    RuntimeState state_after = RuntimeState::Idle;
    ExecutionContext context;
};

std::string ToString(RuntimeFactType runtime_fact_type);

using RuntimeFactCallback = std::function<void(const RuntimeFact&)>;

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

    void SetRuntimeFactCallback(RuntimeFactCallback runtime_fact_callback_value);
    ExecutionContext GetContextSnapshot() const;
    bool IsServiceRunning() const;

private:
    core::Status SubmitCommand(const RuntimeCommand& command);
    void RuntimeLoop();
    void ProcessPendingCommands();
    void ProcessRuntimeCommand(const RuntimeCommand& command);
    void TickIfRunning();
    void EmitRuntimeFacts(const std::vector<RuntimeFact>& runtime_facts) const;

    TaskRunner task_runner;
    ExecutionContext execution_context;
    std::optional<core::Task> loaded_task_storage;

    std::queue<RuntimeCommand> command_queue;
    mutable std::mutex command_lock;
    mutable std::mutex context_lock;
    mutable std::mutex runtime_fact_callback_lock;
    std::condition_variable command_condition;
    std::thread runtime_thread;
    RuntimeFactCallback runtime_fact_callback;

    bool service_running = false;
    bool stop_requested = false;
};

}  // namespace robot_process_platform::runtime
