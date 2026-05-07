#pragma once

#include <optional>

#include "robot_process_platform/core/task_types.h"

namespace robot_process_platform::runtime
{

// QueuedBlock 是单槽队列里的运行时条目。
// 当前阶段先保留 block 与它在 task 中的索引。
struct QueuedBlock
{
    QueuedBlock(const core::ExecutionBlock& execution_block_value,
                int block_index_value);

    core::ExecutionBlock execution_block;
    int block_index = -1;
};

// BlockQueue 是最小单槽执行块队列。
// 当前阶段先不做多 block 预取，只负责：
// 1. 是否已经有 block 在队列里
// 2. 放入一个 block
// 3. 取出这个 block 交给执行器
class BlockQueue
{
public:
    BlockQueue();

    bool HasBlock() const;
    bool PushBlock(const QueuedBlock& queued_block);
    const QueuedBlock* PeekBlock() const;
    std::optional<QueuedBlock> PopBlock();
    void Clear();

private:
    std::optional<QueuedBlock> queued_block;
};

}  // namespace robot_process_platform::runtime
