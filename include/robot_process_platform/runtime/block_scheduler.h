#pragma once

#include <optional>

#include "robot_process_platform/core/task_types.h"
#include "robot_process_platform/runtime/block_queue.h"
#include "robot_process_platform/runtime/execution_context.h"

namespace robot_process_platform::runtime
{

// BlockScheduler 负责从 Task 中找到下一个应该推进的 block。
// 它不直接执行 block，只负责把 task 结构映射为运行时待执行条目。
class BlockScheduler
{
public:
    BlockScheduler();

    std::optional<QueuedBlock> BuildNextQueuedBlock(
        const core::Task& task,
        const ExecutionContext& execution_context) const;
};

}  // namespace robot_process_platform::runtime
