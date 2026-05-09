#include "robot_process_platform/runtime/task_runner.h"

#include "robot_process_platform/platform/logger.h"

namespace robot_process_platform::runtime
{

namespace
{

void SetRuntimeError(ExecutionContext& execution_context,
                     core::ErrorCode error_code,
                     const std::string& error_message)
{
    execution_context.last_error_code = error_code;
    execution_context.last_error = error_message;
}

void LogRuntimeFailure(const std::string& source,
                       const ExecutionContext& execution_context)
{
    platform::Logger::GetInstance().LogE(source,
                                         "Runtime failure: error_code=" +
                                             core::ToString(execution_context.last_error_code) +
                                             " error=" + execution_context.last_error);
}

}  // namespace

TaskRunner::TaskRunner(device::IRobot& robot_instance)
    : block_executor(robot_instance) {}

bool TaskRunner::RunTask(const core::Task& task, ExecutionContext& execution_context)
{
    platform::Logger::GetInstance().LogI("TaskRunner", "RunTask requested from default start position.");
    return RunTask(task, execution_context, 0, 0);
}

bool TaskRunner::RunTask(const core::Task& task,
                         ExecutionContext& execution_context,
                         int start_block_index,
                         int start_action_index)
{
    platform::Logger::GetInstance().LogI("TaskRunner",
                                         "RunTask requested: task_id=" + task.task_id +
                                             " start_block_index=" + std::to_string(start_block_index) +
                                             " start_action_index=" + std::to_string(start_action_index));
    if (!LoadTask(task, execution_context, start_block_index, start_action_index))
    {
        LogRuntimeFailure("TaskRunner", execution_context);
        return false;
    }

    if (!Start(execution_context))
    {
        LogRuntimeFailure("TaskRunner", execution_context);
        return false;
    }

    while (execution_context.runtime_state == RuntimeState::Running)
    {
        if (!Tick(execution_context))
        {
            LogRuntimeFailure("TaskRunner", execution_context);
            return false;
        }
    }

    platform::Logger::GetInstance().LogI("TaskRunner",
                                         "RunTask finished with runtime_state=" + ToString(execution_context.runtime_state));
    return execution_context.runtime_state == RuntimeState::Completed;
}

bool TaskRunner::LoadTask(const core::Task& task,
                          ExecutionContext& execution_context,
                          int start_block_index,
                          int start_action_index)
{
    platform::Logger::GetInstance().LogI("TaskRunner",
                                         "Loading task: " + task.task_id +
                                             " start_block_index=" + std::to_string(start_block_index) +
                                             " start_action_index=" + std::to_string(start_action_index));

    block_queue.Clear();
    loaded_task = &task;
    loaded_start_action_index = start_action_index;
    should_apply_start_action_index = true;

    execution_context.task_id = task.task_id;
    execution_context.template_name = task.template_name;
    execution_context.current_block_index = start_block_index - 1;
    execution_context.current_action_index = -1;
    execution_context.last_error_code = core::ErrorCode::Ok;
    execution_context.last_error.clear();
    execution_context.runtime_state = RuntimeState::Idle;

    if (!ValidateStartPosition(task, start_block_index, start_action_index, execution_context))
    {
        execution_context.runtime_state = RuntimeState::Failed;
        loaded_task = nullptr;
        LogRuntimeFailure("TaskRunner", execution_context);
        return false;
    }

    return HandleStateTransition(StateEvent::TaskLoaded, execution_context, "TaskRunner");
}

bool TaskRunner::Start(ExecutionContext& execution_context)
{
    platform::Logger::GetInstance().LogI("TaskRunner", "Start requested.");
    return HandleStateTransition(StateEvent::StartRequested, execution_context, "TaskRunner");
}

bool TaskRunner::Pause(ExecutionContext& execution_context)
{
    platform::Logger::GetInstance().LogI("TaskRunner", "Pause requested.");
    return HandleStateTransition(StateEvent::PauseRequested, execution_context, "TaskRunner");
}

bool TaskRunner::Resume(ExecutionContext& execution_context)
{
    platform::Logger::GetInstance().LogI("TaskRunner", "Resume requested.");
    return HandleStateTransition(StateEvent::ResumeRequested, execution_context, "TaskRunner");
}

bool TaskRunner::Stop(ExecutionContext& execution_context)
{
    platform::Logger::GetInstance().LogI("TaskRunner", "Stop requested.");
    block_queue.Clear();
    loaded_task = nullptr;
    return HandleStateTransition(StateEvent::StopRequested, execution_context, "TaskRunner");
}

bool TaskRunner::EmergencyStop(ExecutionContext& execution_context)
{
    platform::Logger::GetInstance().LogW("TaskRunner", "EmergencyStop requested.");
    block_queue.Clear();
    return HandleStateTransition(StateEvent::EmergencyStopTriggered, execution_context, "TaskRunner");
}

bool TaskRunner::ResetFault(ExecutionContext& execution_context)
{
    platform::Logger::GetInstance().LogI("TaskRunner", "ResetFault requested.");
    block_queue.Clear();
    return HandleStateTransition(StateEvent::FaultResetRequested, execution_context, "TaskRunner");
}

// Tick 是 runtime 的一次调度周期。
// 当前粒度为：一次 Tick 最多执行一个 Action。
bool TaskRunner::Tick(ExecutionContext& execution_context)
{
    if (loaded_task == nullptr)
    {
        SetRuntimeError(execution_context, core::ErrorCode::RuntimeNoLoadedTask, "no_loaded_task");
        execution_context.runtime_state = RuntimeState::Failed;
        LogRuntimeFailure("TaskRunner", execution_context);
        return false;
    }

    if (execution_context.runtime_state != RuntimeState::Running)
    {
        return true;
    }

    if (!block_queue.HasBlock() &&
        !PushNextBlockIfAllowed(*loaded_task, execution_context))
    {
        if (!HandleStateTransition(StateEvent::TaskCompleted, execution_context, "TaskRunner"))
        {
            SetRuntimeError(execution_context,
                            core::ErrorCode::RuntimeFailedToEnterCompletedState,
                            "failed_to_enter_completed_state");
            execution_context.runtime_state = RuntimeState::Failed;
            LogRuntimeFailure("TaskRunner", execution_context);
            return false;
        }

        execution_context.current_block_index = static_cast<int>(loaded_task->execution_blocks.size());
        execution_context.current_action_index = -1;
        platform::Logger::GetInstance().LogI("TaskRunner", "Task execution completed: " + loaded_task->task_id);
        return true;
    }

    const QueuedBlock* queued_block = block_queue.PeekBlock();
    if (queued_block == nullptr)
    {
        SetRuntimeError(execution_context, core::ErrorCode::RuntimeQueuePeekFailed, "failed_to_peek_block_from_queue");
        execution_context.runtime_state = RuntimeState::Failed;
        LogRuntimeFailure("TaskRunner", execution_context);
        return false;
    }

    int action_index = 0;
    if (should_apply_start_action_index)
    {
        action_index = loaded_start_action_index;
        should_apply_start_action_index = false;
    }
    else if (execution_context.current_block_index == queued_block->block_index)
    {
        action_index = execution_context.current_action_index + 1;
    }

    if (action_index >= static_cast<int>(queued_block->execution_block.actions.size()))
    {
        execution_context.current_block_index = queued_block->block_index;
        execution_context.current_action_index = -1;
        block_queue.Clear();
        if (!HandleStateTransition(StateEvent::BlockCompleted, execution_context, "TaskRunner"))
        {
            SetRuntimeError(execution_context,
                            core::ErrorCode::RuntimeFailedToAcceptBlockCompleted,
                            "failed_to_accept_block_completed_event");
            execution_context.runtime_state = RuntimeState::Failed;
            LogRuntimeFailure("TaskRunner", execution_context);
            return false;
        }

        return true;
    }

    if (!block_executor.ExecuteOneAction(*queued_block, action_index, execution_context))
    {
        HandleStateTransition(StateEvent::BlockFailed, execution_context, "TaskRunner");
        execution_context.runtime_state = RuntimeState::Failed;
        if (execution_context.last_error_code == core::ErrorCode::Ok)
        {
            execution_context.last_error_code = core::ErrorCode::RuntimeBlockExecutionFailed;
        }
        platform::Logger::GetInstance().LogE("TaskRunner",
                                             "Task execution failed: " + loaded_task->task_id +
                                                 " error=" + execution_context.last_error);
        return false;
    }

    const bool block_completed =
        execution_context.current_action_index + 1 >=
        static_cast<int>(queued_block->execution_block.actions.size());
    if (block_completed)
    {
        block_queue.Clear();
        if (!HandleStateTransition(StateEvent::BlockCompleted, execution_context, "TaskRunner"))
        {
            SetRuntimeError(execution_context,
                            core::ErrorCode::RuntimeFailedToAcceptBlockCompleted,
                            "failed_to_accept_block_completed_event");
            execution_context.runtime_state = RuntimeState::Failed;
            LogRuntimeFailure("TaskRunner", execution_context);
            return false;
        }
    }

    platform::Logger::GetInstance().LogI("TaskRunner",
                                         "Task tick completed: task_id=" + loaded_task->task_id +
                                             " block_index=" + std::to_string(execution_context.current_block_index) +
                                             " action_index=" + std::to_string(execution_context.current_action_index));
    return true;
}

bool TaskRunner::ValidateStartPosition(const core::Task& task,
                                       int start_block_index,
                                       int start_action_index,
                                       ExecutionContext& execution_context) const
{
    if (start_block_index < 0 ||
        start_block_index > static_cast<int>(task.execution_blocks.size()))
    {
        SetRuntimeError(execution_context, core::ErrorCode::RuntimeInvalidStartBlockIndex, "invalid_start_block_index");
        return false;
    }

    if (start_block_index == static_cast<int>(task.execution_blocks.size()))
    {
        if (start_action_index != 0)
        {
            SetRuntimeError(execution_context,
                            core::ErrorCode::RuntimeInvalidCompletedTaskStartAction,
                            "invalid_start_action_index_for_completed_task");
            return false;
        }

        return true;
    }

    const core::ExecutionBlock& start_block =
        task.execution_blocks[static_cast<std::size_t>(start_block_index)];
    if (start_action_index < 0 ||
        start_action_index >= static_cast<int>(start_block.actions.size()))
    {
        SetRuntimeError(execution_context, core::ErrorCode::RuntimeInvalidStartActionIndex, "invalid_start_action_index");
        return false;
    }

    return true;
}

bool TaskRunner::PushNextBlockIfAllowed(const core::Task& task, ExecutionContext& execution_context)
{
    if (!state_machine.CanPushBlockToQueue(execution_context))
    {
        return false;
    }

    if (block_queue.HasBlock())
    {
        SetRuntimeError(execution_context, core::ErrorCode::RuntimeQueueShouldBeEmpty, "block_queue_should_be_empty_before_push");
        return false;
    }

    const std::optional<QueuedBlock> next_queued_block =
        block_scheduler.BuildNextQueuedBlock(task, execution_context);
    if (!next_queued_block.has_value())
    {
        return false;
    }

    if (!block_queue.PushBlock(next_queued_block.value()))
    {
        SetRuntimeError(execution_context, core::ErrorCode::RuntimeQueuePushFailed, "failed_to_push_block_into_queue");
        return false;
    }

    platform::Logger::GetInstance().LogI("TaskRunner",
                                         "Queued block index=" + std::to_string(next_queued_block->block_index) +
                                             " name=" + next_queued_block->execution_block.block_name);
    return true;
}

bool TaskRunner::HandleStateTransition(StateEvent state_event,
                                       ExecutionContext& execution_context,
                                       const std::string& event_source)
{
    const RuntimeState state_before = state_machine.CurrentState(execution_context);
    const bool handled = state_machine.HandleEvent(state_event, execution_context);
    const RuntimeState state_after = state_machine.CurrentState(execution_context);

    if (!handled)
    {
        execution_context.last_error_code = core::ErrorCode::RuntimeStateEventRejected;
        platform::Logger::GetInstance().LogW(event_source,
                                             "State event rejected: event=" + ToString(state_event) +
                                                 " state_before=" + ToString(state_before));
        return false;
    }

    platform::Logger::GetInstance().LogI(event_source,
                                         "State transition handled: event=" + ToString(state_event) +
                                             " state_before=" + ToString(state_before) +
                                             " state_after=" + ToString(state_after));
    return true;
}

}  // namespace robot_process_platform::runtime
