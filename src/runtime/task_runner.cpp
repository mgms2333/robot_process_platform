#include "robot_process_platform/runtime/task_runner.h"

#include <sstream>

#include "robot_process_platform/platform/logger.h"

namespace robot_process_platform::runtime
{

TaskRunner::TaskRunner(robot::IRobot& robot_instance)
    :
    robot(robot_instance) {}

bool TaskRunner::RunTask(const core::Task& task, ExecutionContext& execution_context)
{
    platform::Logger::GetInstance().LogI(
        "TaskRunner",
        "Starting task execution: " + task.task_id);

    execution_context.task_id = task.task_id;
    execution_context.template_name = task.template_name;
    execution_context.status = ExecutionStatus::Running;
    execution_context.current_block_index = -1;
    execution_context.current_action_index = -1;
    execution_context.last_error.clear();

    for (std::size_t block_index = 0; block_index < task.execution_blocks.size(); ++block_index)
    {
        if (!RunExecutionBlock(task.execution_blocks[block_index],
                               static_cast<int>(block_index),
                               execution_context))
        {
            execution_context.status = ExecutionStatus::Failed;
            platform::Logger::GetInstance().LogE(
                "TaskRunner",
                "Task execution failed: " + task.task_id + " error=" + execution_context.last_error);
            return false;
        }
    }

    execution_context.status = ExecutionStatus::Completed;
    execution_context.current_block_index = static_cast<int>(task.execution_blocks.size());
    execution_context.current_action_index = -1;
    platform::Logger::GetInstance().LogI(
        "TaskRunner",
        "Task execution completed: " + task.task_id);
    return true;
}

bool TaskRunner::RunExecutionBlock(const core::ExecutionBlock& execution_block,
                                   int block_index,
                                   ExecutionContext& execution_context)
{
    execution_context.current_block_index = block_index;
    execution_context.current_action_index = -1;

    std::ostringstream block_log_stream;
    block_log_stream << "Running execution block index=" << block_index
                     << " name=" << execution_block.block_name
                     << " semantic_type=" << execution_block.semantic_type
                     << " semantic_index=" << execution_block.semantic_index
                     << " process_block_name=" << execution_block.process_block_name;
    platform::Logger::GetInstance().LogI("TaskRunner", block_log_stream.str());

    for (std::size_t action_index = 0; action_index < execution_block.actions.size(); ++action_index)
    {
        if (!RunAction(execution_block.actions[action_index],
                       block_index,
                       static_cast<int>(action_index),
                       execution_context))
        {
            return false;
        }
    }

    return true;
}

bool TaskRunner::RunAction(const core::Action& action,
                           int block_index,
                           int action_index,
                           ExecutionContext& execution_context)
{
    (void)block_index;

    execution_context.current_action_index = action_index;

    platform::Logger::GetInstance().LogD(
        "TaskRunner",
        "Running action index=" + std::to_string(action_index) +
            " name=" + action.action_name +
            " type=" + core::ToString(action.action_type));

    robot::RobotCommandResult command_result(false, "Unsupported action.");

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
            "TaskRunner",
            "Action failed: " + action.action_name + " error=" + command_result.message);
        return false;
    }

    platform::Logger::GetInstance().LogD(
        "TaskRunner",
        "Action completed: " + action.action_name);
    return true;
}

}  // namespace robot_process_platform::runtime
