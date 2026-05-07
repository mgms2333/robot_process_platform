#pragma once

#include "robot_process_platform/runtime/block_executor.h"
#include "robot_process_platform/runtime/block_queue.h"
#include "robot_process_platform/runtime/block_scheduler.h"
#include "robot_process_platform/core/task_types.h"
#include "robot_process_platform/device/interfaces/robot_interface.h"
#include "robot_process_platform/runtime/execution_context.h"
#include "robot_process_platform/runtime/state_machine.h"

namespace robot_process_platform::runtime
{

// TaskRunner 是当前阶段的最小运行时编排器。
// 它负责把：
// 1. Scheduler
// 2. StateMachine
// 3. BlockQueue
// 4. BlockExecutor
// 串成一条最小可控执行链。
class TaskRunner
{
public:
    explicit TaskRunner(device::IRobot& robot_instance);

    bool RunTask(const core::Task& task, ExecutionContext& execution_context);
    bool RunTask(const core::Task& task,
                 ExecutionContext& execution_context,
                 int start_block_index,
                 int start_action_index);
    bool LoadTask(const core::Task& task,
                  ExecutionContext& execution_context,
                  int start_block_index = 0,
                  int start_action_index = 0);
    bool Start(ExecutionContext& execution_context);
    bool Pause(ExecutionContext& execution_context);
    bool Resume(ExecutionContext& execution_context);
    bool Stop(ExecutionContext& execution_context);
    bool EmergencyStop(ExecutionContext& execution_context);
    bool ResetFault(ExecutionContext& execution_context);
    bool Tick(ExecutionContext& execution_context);

private:
    bool ValidateStartPosition(const core::Task& task,
                               int start_block_index,
                               int start_action_index,
                               ExecutionContext& execution_context) const;
    bool PushNextBlockIfAllowed(const core::Task& task, ExecutionContext& execution_context);
    bool HandleStateTransition(StateEvent state_event,
                               ExecutionContext& execution_context,
                               const std::string& event_source);

    StateMachine state_machine;
    BlockScheduler block_scheduler;
    BlockQueue block_queue;
    BlockExecutor block_executor;
    const core::Task* loaded_task = nullptr;
    int loaded_start_action_index = 0;
    bool should_apply_start_action_index = false;
};

}  // namespace robot_process_platform::runtime
