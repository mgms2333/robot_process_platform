#pragma once

#include <map>
#include <string>
#include <vector>

#include "robot_process_platform/core/task_types.h"

namespace robot_process_platform::plugin
{

// ProcessUnitPlan 表示规划阶段中的一个工艺处理单元。
// 它只描述工艺单元级信息，不直接描述底层 Action。
struct ProcessUnitPlan
{
    explicit ProcessUnitPlan(const std::string& unit_name_value);

    // unit_name 表示工艺单元名称。
    std::string unit_name;

    // unit_parameters 保存该工艺单元的规划参数。
    std::map<std::string, std::string> unit_parameters;
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

    // process_units 保存所有规划出的工艺单元。
    std::vector<ProcessUnitPlan> process_units;
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
