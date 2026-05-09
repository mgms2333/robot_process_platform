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
    std::lock_guard<std::mutex> context_guard(context_lock);

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
            task_runner.LoadTask(*command.task,
                                 execution_context,
                                 command.start_block_index,
                                 command.start_action_index);
            return;
        case RuntimeCommandType::StartTask:
            platform::Logger::GetInstance().LogI("RuntimeService", "Processing StartTask command.");
            task_runner.Start(execution_context);
            return;
        case RuntimeCommandType::PauseTask:
            platform::Logger::GetInstance().LogI("RuntimeService", "Processing PauseTask command.");
            task_runner.Pause(execution_context);
            return;
        case RuntimeCommandType::ResumeTask:
            platform::Logger::GetInstance().LogI("RuntimeService", "Processing ResumeTask command.");
            task_runner.Resume(execution_context);
            return;
        case RuntimeCommandType::StopTask:
            platform::Logger::GetInstance().LogI("RuntimeService", "Processing StopTask command.");
            task_runner.Stop(execution_context);
            return;
        case RuntimeCommandType::EmergencyStop:
            platform::Logger::GetInstance().LogW("RuntimeService", "Processing EmergencyStop command.");
            task_runner.EmergencyStop(execution_context);
            return;
        case RuntimeCommandType::ResetFault:
            platform::Logger::GetInstance().LogI("RuntimeService", "Processing ResetFault command.");
            task_runner.ResetFault(execution_context);
            return;
        case RuntimeCommandType::Shutdown:
            platform::Logger::GetInstance().LogI("RuntimeService", "Processing Shutdown command.");
            return;
    }
}

void RuntimeService::TickIfRunning()
{
    std::lock_guard<std::mutex> context_guard(context_lock);
    if (execution_context.runtime_state != RuntimeState::Running)
    {
        return;
    }

    task_runner.Tick(execution_context);
}

}  // namespace robot_process_platform::runtime
