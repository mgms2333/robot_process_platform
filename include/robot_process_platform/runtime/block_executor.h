#pragma once

#include "robot_process_platform/device/interfaces/robot_interface.h"
#include "robot_process_platform/runtime/block_queue.h"
#include "robot_process_platform/runtime/execution_context.h"

namespace robot_process_platform::runtime
{

// BlockExecutor 只负责消费已经进入队列的 block。
// 它不负责决定下一个 block 是什么，也不负责状态迁移。
class BlockExecutor
{
public:
    explicit BlockExecutor(device::IRobot& robot_instance);

    bool ExecuteQueuedBlock(const QueuedBlock& queued_block,
                            ExecutionContext& execution_context);
    bool ExecuteQueuedBlock(const QueuedBlock& queued_block,
                            int start_action_index,
                            ExecutionContext& execution_context);
    bool ExecuteOneAction(const QueuedBlock& queued_block,
                          int action_index,
                          ExecutionContext& execution_context);

private:
    bool ExecuteAction(const core::Action& action,
                       int block_index,
                       int action_index,
                       ExecutionContext& execution_context);

    device::IRobot& robot;
};

}  // namespace robot_process_platform::runtime
