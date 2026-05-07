#include "robot_process_platform/runtime/block_executor.h"

#include <sstream>

#include "robot_process_platform/platform/logger.h"

namespace robot_process_platform::runtime
{

BlockExecutor::BlockExecutor(device::IRobot& robot_instance)
    : robot(robot_instance) {}

bool BlockExecutor::ExecuteQueuedBlock(const QueuedBlock& queued_block,
                                       ExecutionContext& execution_context)
{
    return ExecuteQueuedBlock(queued_block, 0, execution_context);
}

bool BlockExecutor::ExecuteQueuedBlock(const QueuedBlock& queued_block,
                                       int start_action_index,
                                       ExecutionContext& execution_context)
{
    execution_context.current_block_index = queued_block.block_index;
    execution_context.current_action_index = start_action_index - 1;

    if (start_action_index < 0 ||
        start_action_index >= static_cast<int>(queued_block.execution_block.actions.size()))
    {
        execution_context.last_error = "invalid_start_action_index";
        return false;
    }

    std::ostringstream block_log_stream;
    block_log_stream << "Executing queued block index=" << queued_block.block_index
                     << " start_action_index=" << start_action_index
                     << " name=" << queued_block.execution_block.block_name
                     << " semantic_type=" << queued_block.execution_block.semantic_type
                     << " semantic_index=" << queued_block.execution_block.semantic_index
                     << " process_block_name=" << queued_block.execution_block.process_block_name;
    platform::Logger::GetInstance().LogI("BlockExecutor", block_log_stream.str());

    for (std::size_t action_index = static_cast<std::size_t>(start_action_index);
         action_index < queued_block.execution_block.actions.size();
         ++action_index)
    {
        if (!ExecuteOneAction(queued_block,
                              static_cast<int>(action_index),
                              execution_context))
        {
            return false;
        }
    }

    return true;
}

bool BlockExecutor::ExecuteOneAction(const QueuedBlock& queued_block,
                                     int action_index,
                                     ExecutionContext& execution_context)
{
    if (action_index < 0 ||
        action_index >= static_cast<int>(queued_block.execution_block.actions.size()))
    {
        execution_context.last_error = "invalid_action_index";
        return false;
    }

    return ExecuteAction(queued_block.execution_block.actions[static_cast<std::size_t>(action_index)],
                         queued_block.block_index,
                         action_index,
                         execution_context);
}

bool BlockExecutor::ExecuteAction(const core::Action& action,
                                  int block_index,
                                  int action_index,
                                  ExecutionContext& execution_context)
{
    execution_context.current_block_index = block_index;
    execution_context.current_action_index = action_index;

    platform::Logger::GetInstance().LogD(
        "BlockExecutor",
        "Running action index=" + std::to_string(action_index) +
            " name=" + action.action_name +
            " type=" + core::ToString(action.action_type));

    device::RobotCommandResult command_result(false, "Unsupported action.");

    switch (action.action_type)
    {
        case core::TaskActionType::MoveJoint:
            command_result = robot.MoveJoint(action.action_parameters);
            break;
        case core::TaskActionType::MoveLinear:
            command_result = robot.MoveLinear(action.action_parameters);
            break;
        case core::TaskActionType::SetDigitalOutput:
            command_result = robot.SetDigitalOutput(action.action_parameters);
            break;
        case core::TaskActionType::WaitDigitalInput:
            command_result = robot.WaitDigitalInput(action.action_parameters, action.timeout_ms);
            break;
        case core::TaskActionType::Delay:
            command_result = robot.Delay(action.action_parameters, action.timeout_ms);
            break;
    }

    if (!command_result.success)
    {
        execution_context.last_error = command_result.message;
        platform::Logger::GetInstance().LogE(
            "BlockExecutor",
            "Action failed: " + action.action_name + " error=" + command_result.message);
        return false;
    }

    platform::Logger::GetInstance().LogD(
        "BlockExecutor",
        "Action completed: " + action.action_name);
    return true;
}

}  // namespace robot_process_platform::runtime
