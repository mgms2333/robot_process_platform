#pragma once

#include <map>
#include <string>
#include <vector>

#include "robot_process_platform/core/task_types.h"

namespace robot_process_platform::plugin
{

// ProcessBlockPlan 表示规划阶段中的一个工艺处理块。
// 它只描述工艺块级信息，不直接描述底层 Action。
struct ProcessBlockPlan
{
    explicit ProcessBlockPlan(const std::string& block_name_value);

    // block_name 表示工艺块名称。
    std::string block_name;

    // block_parameters 保存该工艺块的规划参数。
    std::map<std::string, std::string> block_parameters;
};

// BoxPlan 表示码垛模板中的一个箱子级规划对象。
// 这类对象属于模板层工艺语义，不属于平台执行层的通用概念。
struct BoxPlan
{
    explicit BoxPlan(int box_index_value);

    // box_index 表示当前箱子在整垛任务中的顺序编号。
    int box_index;

    // process_blocks 保存该箱子对应的工艺块规划结果。
    std::vector<ProcessBlockPlan> process_blocks;
};

// ProcessPlan 表示模板规划阶段输出的完整中间计划。
// Builder 阶段会基于 ProcessPlan 展开出统一 Task。
struct ProcessPlan
{
    ProcessPlan(const std::string& task_id_value,
                const std::string& template_name_value,
                const std::string& source_context_json_value);

    // task_id 是本次规划对应的任务标识。
    std::string task_id;

    // template_name 表示该计划由哪个模板生成。
    std::string template_name;

    // source_context_json 记录模板创建任务时收到的原始上下文 JSON。
    std::string source_context_json;

    // box_plans 保存所有箱子级规划对象。
    std::vector<BoxPlan> box_plans;
};

// ITemplate 定义所有工艺模板必须遵守的统一接口。
class ITemplate
{
public:
    virtual ~ITemplate() = default;

    // TemplateName 返回模板的稳定标识名。
    virtual std::string TemplateName() const = 0;

    // CreateTask 负责接收原始上下文 JSON，并返回模板生成出的统一 Task。
    virtual core::Task CreateTask(const std::string& task_context_json) const = 0;
};

}  // namespace robot_process_platform::plugin
