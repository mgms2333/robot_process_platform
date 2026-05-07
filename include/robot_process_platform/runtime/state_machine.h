#pragma once

#include "robot_process_platform/runtime/execution_context.h"

namespace robot_process_platform::runtime
{

enum class StateEvent
{
    TaskLoaded,
    StartRequested,
    PauseRequested,
    ResumeRequested,
    StopRequested,
    BlockCompleted,
    BlockFailed,
    TaskCompleted,
    EmergencyStopTriggered,
    FaultResetRequested
};

// StateMachine 负责维护运行时当前状态，并判断状态迁移是否合法。
// 当前阶段它先只做最小控制：
// 1. 任务装载后进入 Ready
// 2. Start 后进入 Running
// 3. block 失败进入 Fault
// 4. 任务完成进入 Completed
class StateMachine
{
public:
    StateMachine();

    bool HandleEvent(StateEvent event, ExecutionContext& execution_context);

    bool CanPushBlockToQueue(const ExecutionContext& execution_context) const;

    RuntimeState CurrentState(const ExecutionContext& execution_context) const;
};

std::string ToString(StateEvent state_event);

}  // namespace robot_process_platform::runtime
