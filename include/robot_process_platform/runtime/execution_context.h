#pragma once

#include <string>

namespace robot_process_platform::runtime
{

enum class RuntimeState
{
    Idle,
    Ready,
    Running,
    Paused,
    Fault,
    EmergencyStop,
    Stopped,
    Completed,
    Failed
};

// ExecutionContext 保存一次任务执行过程中的最小运行时状态。
// 当前阶段先记录：
// 1. 任务执行到哪个 block / action
// 2. 运行时当前状态
// 3. 最后错误信息
struct ExecutionContext
{
    ExecutionContext();

    RuntimeState runtime_state = RuntimeState::Idle;
    std::string task_id;
    std::string template_name;
    int current_block_index = -1;
    int current_action_index = -1;
    std::string last_error;
};

std::string ToString(RuntimeState runtime_state);

}  // namespace robot_process_platform::runtime
