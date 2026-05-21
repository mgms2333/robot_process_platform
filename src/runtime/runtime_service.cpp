#include "robot_process_platform/runtime/runtime_service.h"

#include <chrono>
#include <utility>

#include "robot_process_platform/platform/logger.h"

namespace robot_process_platform::runtime
{

namespace
{

constexpr auto kRuntimeIdleWait = std::chrono::milliseconds(10);

RuntimeCommand BuildCommand(RuntimeCommandType type)
{
    RuntimeCommand command;
    command.type = type;
    return command;
}

std::vector<RuntimeFact> BuildRuntimeFacts(const ExecutionContext& context_before,
                                          const ExecutionContext& context_after,
                                          std::optional<RuntimeFactType> specific_fact_type)
{
    std::vector<RuntimeFact> runtime_facts;

    if (context_before.runtime_state == context_after.runtime_state)
    {
        return runtime_facts;
    }

    if (specific_fact_type.has_value())
    {
        RuntimeFact specific_fact;
        specific_fact.type = specific_fact_type.value();
        specific_fact.state_before = context_before.runtime_state;
        specific_fact.state_after = context_after.runtime_state;
        specific_fact.context = context_after;
        runtime_facts.push_back(specific_fact);
    }

    RuntimeFact state_changed_fact;
    state_changed_fact.type = RuntimeFactType::StateChanged;
    state_changed_fact.state_before = context_before.runtime_state;
    state_changed_fact.state_after = context_after.runtime_state;
    state_changed_fact.context = context_after;
    runtime_facts.push_back(state_changed_fact);

    return runtime_facts;
}

}  // namespace

RuntimeService::RuntimeService(device::IRobot& robot_instance)
    : task_runner(robot_instance) {}

RuntimeService::~RuntimeService()
{
    StopService();
}

core::Status RuntimeService::StartService()
{
    std::lock_guard<std::mutex> command_guard(command_lock);

    if (service_running)
    {
        return core::MakeSuccessStatus();
    }

    platform::Logger::GetInstance().LogI("RuntimeService", "Starting runtime service thread.");
    stop_requested = false;
    service_running = true;
    runtime_thread = std::thread(&RuntimeService::RuntimeLoop, this);
    return core::MakeSuccessStatus();
}

void RuntimeService::StopService()
{
    {
        std::lock_guard<std::mutex> command_guard(command_lock);
        if (!service_running)
        {
            return;
        }

        platform::Logger::GetInstance().LogI("RuntimeService", "Stopping runtime service thread.");
        stop_requested = true;
        command_queue.push(BuildCommand(RuntimeCommandType::Shutdown));
    }

    command_condition.notify_all();

    if (runtime_thread.joinable())
    {
        runtime_thread.join();
    }

    {
        std::lock_guard<std::mutex> command_guard(command_lock);
        service_running = false;
    }
}

core::Status RuntimeService::SubmitLoadTask(const core::Task& task,
                                            int start_block_index,
                                            int start_action_index)
{
    RuntimeCommand command;
    command.type = RuntimeCommandType::LoadTask;
    command.task = task;
    command.start_block_index = start_block_index;
    command.start_action_index = start_action_index;
    return SubmitCommand(command);
}

core::Status RuntimeService::SubmitStartTask()
{
    return SubmitCommand(BuildCommand(RuntimeCommandType::StartTask));
}

core::Status RuntimeService::SubmitPauseTask()
{
    return SubmitCommand(BuildCommand(RuntimeCommandType::PauseTask));
}

core::Status RuntimeService::SubmitResumeTask()
{
    return SubmitCommand(BuildCommand(RuntimeCommandType::ResumeTask));
}

core::Status RuntimeService::SubmitStopTask()
{
    return SubmitCommand(BuildCommand(RuntimeCommandType::StopTask));
}

core::Status RuntimeService::SubmitEmergencyStop()
{
    return SubmitCommand(BuildCommand(RuntimeCommandType::EmergencyStop));
}

core::Status RuntimeService::SubmitResetFault()
{
    return SubmitCommand(BuildCommand(RuntimeCommandType::ResetFault));
}

void RuntimeService::SetRuntimeFactCallback(RuntimeFactCallback runtime_fact_callback_value)
{
    std::lock_guard<std::mutex> runtime_fact_callback_guard(runtime_fact_callback_lock);
    runtime_fact_callback = std::move(runtime_fact_callback_value);
}

ExecutionContext RuntimeService::GetContextSnapshot() const
{
    std::lock_guard<std::mutex> context_guard(context_lock);
    return execution_context;
}

bool RuntimeService::IsServiceRunning() const
{
    std::lock_guard<std::mutex> command_guard(command_lock);
    return service_running;
}

core::Status RuntimeService::SubmitCommand(const RuntimeCommand& command)
{
    {
        std::lock_guard<std::mutex> command_guard(command_lock);
        if (!service_running)
        {
            platform::Logger::GetInstance().LogE("RuntimeService", "SubmitCommand rejected because service is not running.");
            return core::MakeErrorStatus(core::ErrorCode::RuntimeServiceNotRunning,
                                         "runtime_service_is_not_running");
        }

        command_queue.push(command);
    }

    platform::Logger::GetInstance().LogD("RuntimeService", "Runtime command queued.");
    command_condition.notify_one();
    return core::MakeSuccessStatus();
}

void RuntimeService::RuntimeLoop()
{
    platform::Logger::GetInstance().LogI("RuntimeService", "Runtime service loop entered.");

    while (true)
    {
        ProcessPendingCommands();

        {
            std::lock_guard<std::mutex> command_guard(command_lock);
            if (stop_requested)
            {
                break;
            }
        }

        TickIfRunning();

        std::unique_lock<std::mutex> command_guard(command_lock);
        command_condition.wait_for(command_guard,
                                   kRuntimeIdleWait,
                                   [this]()
                                   {
                                       return stop_requested || !command_queue.empty();
                                   });
    }

    platform::Logger::GetInstance().LogI("RuntimeService", "Runtime service loop exited.");
}

void RuntimeService::ProcessPendingCommands()
{
    while (true)
    {
        RuntimeCommand command;

        {
            std::lock_guard<std::mutex> command_guard(command_lock);
            if (command_queue.empty())
            {
                return;
            }

            command = std::move(command_queue.front());
            command_queue.pop();
        }

        ProcessRuntimeCommand(command);
    }
}

void RuntimeService::ProcessRuntimeCommand(const RuntimeCommand& command)
{
    std::vector<RuntimeFact> runtime_facts;

    {
        std::lock_guard<std::mutex> context_guard(context_lock);
        const ExecutionContext context_before = execution_context;

        switch (command.type)
        {
            case RuntimeCommandType::LoadTask:
                if (!command.task.has_value())
                {
                    execution_context.last_error_code = core::ErrorCode::RuntimeCommandMissingTask;
                    execution_context.last_error = "load_task_command_missing_task";
                    platform::Logger::GetInstance().LogE("RuntimeService",
                                                         "LoadTask command rejected because task payload is missing.");
                    return;
                }

                platform::Logger::GetInstance().LogI("RuntimeService",
                                                     "Processing LoadTask command: task_id=" + command.task->task_id);
                loaded_task_storage = command.task;
                task_runner.LoadTask(*loaded_task_storage,
                                     execution_context,
                                     command.start_block_index,
                                     command.start_action_index);
                runtime_facts =
                    BuildRuntimeFacts(context_before, execution_context, RuntimeFactType::TaskLoaded);
                break;
            case RuntimeCommandType::StartTask:
                platform::Logger::GetInstance().LogI("RuntimeService", "Processing StartTask command.");
                task_runner.Start(execution_context);
                runtime_facts =
                    BuildRuntimeFacts(context_before, execution_context, RuntimeFactType::TaskStarted);
                break;
            case RuntimeCommandType::PauseTask:
                platform::Logger::GetInstance().LogI("RuntimeService", "Processing PauseTask command.");
                task_runner.Pause(execution_context);
                runtime_facts =
                    BuildRuntimeFacts(context_before, execution_context, RuntimeFactType::TaskPaused);
                break;
            case RuntimeCommandType::ResumeTask:
                platform::Logger::GetInstance().LogI("RuntimeService", "Processing ResumeTask command.");
                task_runner.Resume(execution_context);
                runtime_facts =
                    BuildRuntimeFacts(context_before, execution_context, RuntimeFactType::TaskResumed);
                break;
            case RuntimeCommandType::StopTask:
                platform::Logger::GetInstance().LogI("RuntimeService", "Processing StopTask command.");
                task_runner.Stop(execution_context);
                loaded_task_storage.reset();
                runtime_facts =
                    BuildRuntimeFacts(context_before, execution_context, RuntimeFactType::TaskStopped);
                break;
            case RuntimeCommandType::EmergencyStop:
                platform::Logger::GetInstance().LogW("RuntimeService", "Processing EmergencyStop command.");
                task_runner.EmergencyStop(execution_context);
                runtime_facts = BuildRuntimeFacts(context_before,
                                                  execution_context,
                                                  RuntimeFactType::TaskEmergencyStopped);
                break;
            case RuntimeCommandType::ResetFault:
                platform::Logger::GetInstance().LogI("RuntimeService", "Processing ResetFault command.");
                task_runner.ResetFault(execution_context);
                runtime_facts =
                    BuildRuntimeFacts(context_before, execution_context, RuntimeFactType::FaultReset);
                break;
            case RuntimeCommandType::Shutdown:
                platform::Logger::GetInstance().LogI("RuntimeService", "Processing Shutdown command.");
                loaded_task_storage.reset();
                break;
        }
    }

    EmitRuntimeFacts(runtime_facts);
}

void RuntimeService::TickIfRunning()
{
    std::vector<RuntimeFact> runtime_facts;

    {
        std::lock_guard<std::mutex> context_guard(context_lock);
        if (execution_context.runtime_state != RuntimeState::Running)
        {
            return;
        }

        const ExecutionContext context_before = execution_context;
        task_runner.Tick(execution_context);

        std::optional<RuntimeFactType> specific_fact_type;
        switch (execution_context.runtime_state)
        {
            case RuntimeState::Completed:
                specific_fact_type = RuntimeFactType::TaskCompleted;
                break;
            case RuntimeState::Fault:
            case RuntimeState::Failed:
                specific_fact_type = RuntimeFactType::TaskFailed;
                break;
            case RuntimeState::EmergencyStop:
                specific_fact_type = RuntimeFactType::TaskEmergencyStopped;
                break;
            default:
                break;
        }

        runtime_facts = BuildRuntimeFacts(context_before, execution_context, specific_fact_type);
    }

    EmitRuntimeFacts(runtime_facts);
}

void RuntimeService::EmitRuntimeFacts(const std::vector<RuntimeFact>& runtime_facts) const
{
    if (runtime_facts.empty())
    {
        return;
    }

    RuntimeFactCallback current_runtime_fact_callback;
    {
        std::lock_guard<std::mutex> runtime_fact_callback_guard(runtime_fact_callback_lock);
        current_runtime_fact_callback = runtime_fact_callback;
    }

    if (!current_runtime_fact_callback)
    {
        return;
    }

    for (const RuntimeFact& runtime_fact : runtime_facts)
    {
        current_runtime_fact_callback(runtime_fact);
    }
}

std::string ToString(RuntimeFactType runtime_fact_type)
{
    switch (runtime_fact_type)
    {
        case RuntimeFactType::StateChanged:
            return "StateChanged";
        case RuntimeFactType::TaskLoaded:
            return "TaskLoaded";
        case RuntimeFactType::TaskStarted:
            return "TaskStarted";
        case RuntimeFactType::TaskPaused:
            return "TaskPaused";
        case RuntimeFactType::TaskResumed:
            return "TaskResumed";
        case RuntimeFactType::TaskStopped:
            return "TaskStopped";
        case RuntimeFactType::TaskCompleted:
            return "TaskCompleted";
        case RuntimeFactType::TaskFailed:
            return "TaskFailed";
        case RuntimeFactType::TaskEmergencyStopped:
            return "TaskEmergencyStopped";
        case RuntimeFactType::FaultReset:
            return "FaultReset";
    }

    return "Unknown";
}

}  // namespace robot_process_platform::runtime
