#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace robot_process_platform::core
{

// 本文件定义运行时最基础的静态任务模型。
//
// 三者之间的包含关系如下：
// 1. Task 表示一次完整任务实例
// 2. Task 内包含多个 Step
// 3. Step 内包含多个 Action
//
// 可以理解为：
// Task = 一次完整工艺
// Step = 工艺中的语义步骤
// Action = 步骤中的原子执行动作
//
// 运行时推进时，通常按以下粒度工作：
// 1. 调度器以 Step 作为工艺语义边界推进
// 2. 执行器以 Action 作为最小执行单元处理
// 3. 模板层最终必须把具体工艺展开成 Task / Step / Action 统一模型

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

// Step 表示具有工艺语义的一组动作，例如 Pick、Place、Lift。
// 调度器推进时以 Step 作为语义边界，以 Action 作为执行边界。
// 一个 Step 会包含多个按顺序执行的 Action。
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

// Task 表示一次完整工艺任务实例。
// 模板的输出结果最终必须落到 Task / Step / Action 统一模型上。
// 一个 Task 会按顺序组织多个 Step，从而描述完整执行流程。
struct Task
{
    Task(const std::string& task_id_value, const std::string& template_name_value);

    // task_id 是任务实例唯一标识。
    std::string task_id;

    // template_name 记录任务由哪个模板生成。
    std::string template_name;

    // steps 保存完整的任务步骤序列。
    std::vector<Step> steps;
};

std::string ToString(TaskActionType action_type);

}  // namespace robot_process_platform::core
