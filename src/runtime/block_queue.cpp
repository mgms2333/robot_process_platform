#include "robot_process_platform/runtime/block_queue.h"

#include "robot_process_platform/platform/logger.h"

namespace robot_process_platform::runtime
{

QueuedBlock::QueuedBlock(const core::ExecutionBlock& execution_block_value,
                         int block_index_value)
    : execution_block(execution_block_value),
      block_index(block_index_value) {}

BlockQueue::BlockQueue() = default;

bool BlockQueue::HasBlock() const
{
    return queued_block.has_value();
}

bool BlockQueue::PushBlock(const QueuedBlock& queued_block_value)
{
    if (queued_block.has_value())
    {
        platform::Logger::GetInstance().LogW("BlockQueue",
                                             "PushBlock rejected because queue already has block: existing_block_index=" +
                                                 std::to_string(queued_block->block_index) +
                                                 " new_block_index=" + std::to_string(queued_block_value.block_index));
        return false;
    }

    queued_block = queued_block_value;
    platform::Logger::GetInstance().LogD("BlockQueue",
                                         "Block pushed: block_index=" + std::to_string(queued_block_value.block_index) +
                                             " block_name=" + queued_block_value.execution_block.block_name);
    return true;
}

const QueuedBlock* BlockQueue::PeekBlock() const
{
    if (!queued_block.has_value())
    {
        return nullptr;
    }

    return &queued_block.value();
}

std::optional<QueuedBlock> BlockQueue::PopBlock()
{
    if (!queued_block.has_value())
    {
        platform::Logger::GetInstance().LogW("BlockQueue", "PopBlock rejected because queue is empty.");
        return std::nullopt;
    }

    std::optional<QueuedBlock> popped_block = queued_block;
    platform::Logger::GetInstance().LogD("BlockQueue",
                                         "Block popped: block_index=" + std::to_string(popped_block->block_index) +
                                             " block_name=" + popped_block->execution_block.block_name);
    queued_block.reset();
    return popped_block;
}

void BlockQueue::Clear()
{
    if (queued_block.has_value())
    {
        platform::Logger::GetInstance().LogD("BlockQueue",
                                             "Clearing active block: block_index=" + std::to_string(queued_block->block_index) +
                                                 " block_name=" + queued_block->execution_block.block_name);
    }
    queued_block.reset();
}

}  // namespace robot_process_platform::runtime
