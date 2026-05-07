#include "robot_process_platform/runtime/block_scheduler.h"

namespace robot_process_platform::runtime
{

BlockScheduler::BlockScheduler() = default;

std::optional<QueuedBlock> BlockScheduler::BuildNextQueuedBlock(
    const core::Task& task,
    const ExecutionContext& execution_context) const
{
    const int next_block_index = execution_context.current_block_index + 1;
    if (next_block_index < 0 ||
        next_block_index >= static_cast<int>(task.execution_blocks.size()))
    {
        return std::nullopt;
    }

    return QueuedBlock(task.execution_blocks[next_block_index], next_block_index);
}

}  // namespace robot_process_platform::runtime
