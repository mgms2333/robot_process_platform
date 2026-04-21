#pragma once

#include "robot_process_platform/plugin/template_types.h"

namespace palletizing
{

// PalletizingTaskParameters 是码垛模板内部使用的强类型参数对象。
// 当前已经对接 tests/97582da7-be0d-4139-81cc-8d0775bdd49e.json 中的核心字段，
// 并保存模板内部需要的规划结果。
struct PalletizingTaskParameters
{
    PalletizingTaskParameters(const std::string& task_uuid_value,
                              const std::string& pallet_direction_value,
                              const std::string& left_point_models_json_value,
                              const std::string& box_length_mm_value,
                              const std::string& box_width_mm_value,
                              const std::string& box_height_mm_value);

    // task_uuid 表示该码垛任务在业务侧的唯一标识。
    std::string task_uuid;

    // pallet_direction 表示托盘放置方向。
    std::string pallet_direction;

    // left_point_models_json 保存左托盘按箱子组织的点位数组原文。
    std::string left_point_models_json;

    // box_length_mm 表示箱体长度，单位毫米。
    std::string box_length_mm;

    // box_width_mm 表示箱体宽度，单位毫米。
    std::string box_width_mm;

    // box_height_mm 表示箱体高度，单位毫米。
    std::string box_height_mm;
};

// PalletizingTemplate 是码垛模板的最小参考实现。
// 它用于固定模板的主流程边界：
// CreateTask -> ParseTaskParameters -> Plan -> BuildTask
class PalletizingTemplate : public robot_process_platform::plugin::ITemplate
{
public:
    std::string TemplateName() const override;

    // CreateTask 是模板对外开放的创建入口。
    // 当前版本会解析 tests/97582da7-be0d-4139-81cc-8d0775bdd49e.json 这一类输入，
    // 并生成统一 Task。
    robot_process_platform::core::Task CreateTask(const std::string& task_context_json) const override;

private:
    // ParseTaskParametersFromJson 负责把原始 JSON 收敛为模板专属参数对象。
    PalletizingTaskParameters ParseTaskParametersFromJson(const std::string& task_context_json) const;

    // BuildProcessPlan 只负责生成箱子级规划对象及其工艺块。
    // 它不负责展开执行端的 ExecutionBlock 与 Action。
    robot_process_platform::plugin::ProcessPlan BuildProcessPlan(
        const std::string& task_context_json,
        const PalletizingTaskParameters& task_parameters) const;

    // BuildTaskFromPlan 只负责创建执行端 Task，并收集各工艺块展开出的 ExecutionBlock。
    robot_process_platform::core::Task BuildTaskFromPlan(
        const robot_process_platform::plugin::ProcessPlan& process_plan) const;

    // BuildExecutionBlockFromProcessBlock 负责把一个工艺块展开成一个执行块。
    robot_process_platform::core::ExecutionBlock BuildExecutionBlockFromProcessBlock(
        const robot_process_platform::plugin::ProcessBlockPlan& process_block_plan) const;

    // BuildExecutionBlockForPickBlock 负责把取料工艺块展开成一个连续执行块。
    robot_process_platform::core::ExecutionBlock BuildExecutionBlockForPickBlock(
        const robot_process_platform::plugin::ProcessBlockPlan& process_block_plan) const;

    // BuildExecutionBlockForScanBlock 负责把扫码工艺块展开成一个连续执行块。
    robot_process_platform::core::ExecutionBlock BuildExecutionBlockForScanBlock(
        const robot_process_platform::plugin::ProcessBlockPlan& process_block_plan) const;

    // BuildExecutionBlockForTransferBlock 负责把过渡工艺块展开成一个连续执行块。
    robot_process_platform::core::ExecutionBlock BuildExecutionBlockForTransferBlock(
        const robot_process_platform::plugin::ProcessBlockPlan& process_block_plan) const;

    // BuildExecutionBlockForPlaceBlock 负责把放料工艺块展开成一个连续执行块。
    robot_process_platform::core::ExecutionBlock BuildExecutionBlockForPlaceBlock(
        const robot_process_platform::plugin::ProcessBlockPlan& process_block_plan) const;
};

}  // namespace palletizing
