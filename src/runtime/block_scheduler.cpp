#include "robot_process_platform/runtime/block_scheduler.h"

#include "robot_process_platform/platform/logger.h"

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
        platform::Logger::GetInstance().LogD("BlockScheduler",
                                             "No next block available: current_block_index=" +
                                                 std::to_string(execution_context.current_block_index) +
                                                 " task_block_count=" + std::to_string(task.execution_blocks.size()));
        return std::nullopt;
    }

    platform::Logger::GetInstance().LogD("BlockScheduler",
                                         "Selected next block: block_index=" + std::to_string(next_block_index) +
                                             " block_name=" + task.execution_blocks[next_block_index].block_name);
    return QueuedBlock(task.execution_blocks[next_block_index], next_block_index);
}

}  // namespace robot_process_platform::runtime
