#pragma once

#include <string>

namespace robot_process_platform::runtime
{

enum class ExecutionStatus
{
    Idle,
    Running,
    Completed,
    Failed
};

// ExecutionContext 保存一次任务执行过程中的最小运行时状态。
// 当前阶段先记录执行位置和最后错误，后续可继续扩展为恢复点。
struct ExecutionContext
{
    ExecutionContext();

    ExecutionStatus status = ExecutionStatus::Idle;
    std::string task_id;
    std::string template_name;
    int current_block_index = -1;
    int current_action_index = -1;
    std::string last_error;
};

std::string ToString(ExecutionStatus execution_status);

}  // namespace robot_process_platform::runtime
