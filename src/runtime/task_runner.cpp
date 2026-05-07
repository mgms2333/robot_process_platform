#include "robot_process_platform/runtime/task_runner.h"

#include "robot_process_platform/platform/logger.h"

namespace robot_process_platform::runtime
{

TaskRunner::TaskRunner(device::IRobot& robot_instance)
    : block_executor(robot_instance) {}

bool TaskRunner::RunTask(const core::Task& task, ExecutionContext& execution_context)
{
    return RunTask(task, execution_context, 0, 0);
}

bool TaskRunner::RunTask(const core::Task& task,
                         ExecutionContext& execution_context,
                         int start_block_index,
                         int start_action_index)
{
    if (!LoadTask(task, execution_context, start_block_index, start_action_index))
    {
        return false;
    }

    if (!Start(execution_context))
    {
        return false;
    }

    while (execution_context.runtime_state == RuntimeState::Running)
    {
        if (!Tick(execution_context))
        {
            return false;
        }
    }

    return execution_context.runtime_state == RuntimeState::Completed;
}

bool TaskRunner::LoadTask(const core::Task& task,
                          ExecutionContext& execution_context,
                          int start_block_index,
                          int start_action_index)
{
    platform::Logger::GetInstance().LogI(
        "TaskRunner",
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
    execution_context.last_error.clear();
    execution_context.runtime_state = RuntimeState::Idle;

    if (!ValidateStartPosition(task, start_block_index, start_action_index, execution_context))
    {
        execution_context.runtime_state = RuntimeState::Failed;
        loaded_task = nullptr;
        return false;
    }

    return HandleStateTransition(StateEvent::TaskLoaded, execution_context, "TaskRunner");
}

bool TaskRunner::Start(ExecutionContext& execution_context)
{
    return HandleStateTransition(StateEvent::StartRequested, execution_context, "TaskRunner");
}

bool TaskRunner::Pause(ExecutionContext& execution_context)
{
    return HandleStateTransition(StateEvent::PauseRequested, execution_context, "TaskRunner");
}

bool TaskRunner::Resume(ExecutionContext& execution_context)
{
    return HandleStateTransition(StateEvent::ResumeRequested, execution_context, "TaskRunner");
}

bool TaskRunner::Stop(ExecutionContext& execution_context)
{
    block_queue.Clear();
    loaded_task = nullptr;
    return HandleStateTransition(StateEvent::StopRequested, execution_context, "TaskRunner");
}

bool TaskRunner::EmergencyStop(ExecutionContext& execution_context)
{
    block_queue.Clear();
    return HandleStateTransition(StateEvent::EmergencyStopTriggered, execution_context, "TaskRunner");
}

bool TaskRunner::ResetFault(ExecutionContext& execution_context)
{
    block_queue.Clear();
    return HandleStateTransition(StateEvent::FaultResetRequested, execution_context, "TaskRunner");
}

// Tick 是 runtime 的一次调度周期。
// 当前粒度为：一次 Tick 最多执行一个 Action。
bool TaskRunner::Tick(ExecutionContext& execution_context)
{
    if (loaded_task == nullptr)
    {
        execution_context.last_error = "no_loaded_task";
        execution_context.runtime_state = RuntimeState::Failed;
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
            execution_context.last_error = "failed_to_enter_completed_state";
            execution_context.runtime_state = RuntimeState::Failed;
            return false;
        }

        execution_context.current_block_index = static_cast<int>(loaded_task->execution_blocks.size());
        execution_context.current_action_index = -1;
        platform::Logger::GetInstance().LogI(
            "TaskRunner",
            "Task execution completed: " + loaded_task->task_id);
        return true;
    }

    const QueuedBlock* queued_block = block_queue.PeekBlock();
    if (queued_block == nullptr)
    {
        execution_context.last_error = "failed_to_peek_block_from_queue";
        execution_context.runtime_state = RuntimeState::Failed;
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
            execution_context.last_error = "failed_to_accept_block_completed_event";
            execution_context.runtime_state = RuntimeState::Failed;
            return false;
        }

        return true;
    }

    if (!block_executor.ExecuteOneAction(*queued_block, action_index, execution_context))
    {
        HandleStateTransition(StateEvent::BlockFailed, execution_context, "TaskRunner");
        execution_context.runtime_state = RuntimeState::Failed;
        platform::Logger::GetInstance().LogE(
            "TaskRunner",
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
            execution_context.last_error = "failed_to_accept_block_completed_event";
            execution_context.runtime_state = RuntimeState::Failed;
            return false;
        }
    }

    platform::Logger::GetInstance().LogI(
        "TaskRunner",
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
        execution_context.last_error = "invalid_start_block_index";
        return false;
    }

    if (start_block_index == static_cast<int>(task.execution_blocks.size()))
    {
        if (start_action_index != 0)
        {
            execution_context.last_error = "invalid_start_action_index_for_completed_task";
            return false;
        }

        return true;
    }

    const core::ExecutionBlock& start_block =
        task.execution_blocks[static_cast<std::size_t>(start_block_index)];
    if (start_action_index < 0 ||
        start_action_index >= static_cast<int>(start_block.actions.size()))
    {
        execution_context.last_error = "invalid_start_action_index";
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
        execution_context.last_error = "block_queue_should_be_empty_before_push";
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
        execution_context.last_error = "failed_to_push_block_into_queue";
        return false;
    }

    platform::Logger::GetInstance().LogI(
        "TaskRunner",
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
        platform::Logger::GetInstance().LogW(
            event_source,
            "State event rejected: event=" + ToString(state_event) +
                " state_before=" + ToString(state_before));
        return false;
    }

    platform::Logger::GetInstance().LogI(
        event_source,
        "State transition handled: event=" + ToString(state_event) +
            " state_before=" + ToString(state_before) +
            " state_after=" + ToString(state_after));
    return true;
}

}  // namespace robot_process_platform::runtime
