#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace robot_process_platform::core
{

// 本文件定义执行端最基础的静态任务模型。
//
// 当前执行端对象层级如下：
// 1. Task 表示一次完整任务实例
// 2. Task 内包含多个 ExecutionBlock
// 3. ExecutionBlock 内包含多个 Action
//
// 可以理解为：
// Task = 一次完整执行任务
// ExecutionBlock = 一组应连续执行的动作块
// Action = 执行器直接消费的原子动作
//
// 模板层的规划对象与执行层对象并不相同：
// 1. 模板层更关注 BoxPlan / ProcessBlockPlan 等工艺语义
// 2. Build 阶段负责把工艺语义对象展开为 ExecutionBlock / Action
// 3. 执行层只消费统一的 Task / ExecutionBlock / Action

// TaskActionType 定义运行时可识别的基础动作类型。
// 模板层最终必须把工艺语义展开为这些统一动作，供执行链消费。
enum class TaskActionType
{
    MoveJoint,
    MoveLinear,
    SetDigitalOutput,
    WaitDigitalInput,
    Delay
};

// Action 表示最小可执行单元。
// 一个 Action 不再继续拆分，由执行器直接交给机器人或设备适配层处理。
// Action 不直接描述完整工艺语义，它只描述“当前要执行的一个基础动作”。
struct Action
{
    Action(const std::string& action_name_value,
           TaskActionType action_type_value,
           const std::map<std::string, std::string>& action_parameters_value,
           std::int32_t timeout_ms_value);

    // action_name 用于日志、调试和界面展示。
    std::string action_name;

    // action_type 决定执行器应该调用哪一类底层能力。
    TaskActionType action_type;

    // action_parameters 保存动作参数，MVP 阶段先使用键值对保持灵活性。
    std::map<std::string, std::string> action_parameters;

    // timeout_ms 表示动作最大允许执行时长，单位为毫秒。
    // 该值必须由模板显式给出，避免“未设置”和“无限等待”混淆。
    std::int32_t timeout_ms;
};

// Step 是更细的可读执行步骤占位对象。
// 当前阶段 Task 不再直接消费 Step，而是直接消费 ExecutionBlock。
// 保留该结构是为了后续在 block 内继续表达细粒度步骤时有稳定落点。
struct Step
{
    explicit Step(const std::string& step_name_value);

    // step_name 用于标识当前工艺步骤名称。
    std::string step_name;

    // actions 保存该步骤下需要顺序执行的动作列表。
    std::vector<Action> actions;

    // priority 为后续优先级队列预留，当前阶段先保留字段。
    int priority = 0;
};

// ExecutionBlock 表示执行层的最小调度块。
// 一个 block 内部的动作应连续执行，避免每个 Action 单独调度造成明显停顿。
struct ExecutionBlock
{
    ExecutionBlock(const std::string& block_name_value,
                   const std::string& semantic_type_value,
                   int semantic_index_value);

    // block_name 用于日志、调试和界面展示。
    std::string block_name;

    // semantic_type 表示该 block 对应的模板语义类型，例如 box / seam / unit。
    std::string semantic_type;

    // semantic_index 表示该语义对象在模板规划结果中的序号。
    int semantic_index;

    // process_block_name 记录该 block 来源于哪个模板工艺块，例如 pick_block。
    std::string process_block_name;

    // actions 保存该 block 下需要连续执行的动作列表。
    std::vector<Action> actions;
};

// Task 表示一次完整执行任务实例。
// 模板 Build 的输出结果最终必须落到 Task / ExecutionBlock / Action 统一模型上。
// 一个 Task 会按顺序组织多个 ExecutionBlock，从而描述完整执行流程。
struct Task
{
    Task(const std::string& task_id_value, const std::string& template_name_value);

    // task_id 是任务实例唯一标识。
    std::string task_id;

    // template_name 记录任务由哪个模板生成。
    std::string template_name;

    // execution_blocks 保存完整的执行块序列。
    std::vector<ExecutionBlock> execution_blocks;
};

std::string ToString(TaskActionType action_type);

}  // namespace robot_process_platform::core
