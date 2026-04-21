#pragma once

#include "robot_process_platform/core/task_types.h"
#include "robot_process_platform/robot/robot_interface.h"
#include "robot_process_platform/runtime/execution_context.h"

namespace robot_process_platform::runtime
{

// TaskRunner 负责按顺序执行 Task / ExecutionBlock / Action。
// 当前阶段先提供最小顺序执行能力，不引入调度器和状态机。
class TaskRunner
{
public:
    explicit TaskRunner(robot::IRobot& robot_instance);

    bool RunTask(const core::Task& task, ExecutionContext& execution_context);

private:
    bool RunExecutionBlock(const core::ExecutionBlock& execution_block,
                           int block_index,
                           ExecutionContext& execution_context);

    bool RunAction(const core::Action& action,
                   int block_index,
                   int action_index,
                   ExecutionContext& execution_context);

    robot::IRobot& robot;
};

}  // namespace robot_process_platform::runtime
