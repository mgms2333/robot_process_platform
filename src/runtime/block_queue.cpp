#include "robot_process_platform/runtime/block_queue.h"

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
        return false;
    }

    queued_block = queued_block_value;
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
        return std::nullopt;
    }

    std::optional<QueuedBlock> popped_block = queued_block;
    queued_block.reset();
    return popped_block;
}

void BlockQueue::Clear()
{
    queued_block.reset();
}

}  // namespace robot_process_platform::runtime
