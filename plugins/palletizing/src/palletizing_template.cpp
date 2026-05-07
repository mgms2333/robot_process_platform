#include "palletizing/palletizing_template.h"

#include <array>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "robot_process_platform/core/json_utils.h"

/*
当前文件主流程说明

输入：std::string task_context_json
    |
    | ParseTaskParametersFromJson()
    | 输入：const std::string&
    v
PalletizingTaskParameters task_parameters
    |
    | BuildProcessPlan()
    | 输入：const std::string& + const PalletizingTaskParameters&
    v
robot_process_platform::plugin::ProcessPlan process_plan
    |
    | 内部按箱子展开：
    | for each box in leftPointModels
    |     -> robot_process_platform::plugin::BoxPlan box_plan
    |     -> box_plan.process_blocks
    v
std::vector<robot_process_platform::plugin::BoxPlan> box_plans
    |
    | BuildTaskFromPlan()
    | 输入：const robot_process_platform::plugin::ProcessPlan&
    v
robot_process_platform::core::Task task
    |
    | BuildExecutionBlockFromProcessBlock()
    | 输入：const robot_process_platform::plugin::ProcessBlockPlan&
    v
robot_process_platform::core::ExecutionBlock execution_block
    |
    | BuildExecutionBlockForPickBlock()
    | BuildExecutionBlockForScanBlock()
    | BuildExecutionBlockForTransferBlock()
    | BuildExecutionBlockForPlaceBlock()
    v
std::vector<robot_process_platform::core::Action> actions
*/

namespace palletizing
{
std::string BuildUnitRuntimeJson(const std::vector<std::pair<std::string, std::string>>& fields)
{
    std::ostringstream output_stream;
    output_stream << "{";

    for (std::size_t field_index = 0; field_index < fields.size(); ++field_index)
    {
        if (field_index > 0)
        {
            output_stream << ", ";
        }

        output_stream << "\"" << fields[field_index].first << "\": "
                      << "\"" << fields[field_index].second << "\"";
    }

    output_stream << "}";
    return output_stream.str();
}

PalletizingTaskParameters::PalletizingTaskParameters(const std::string& task_uuid_value,
                                                     const std::string& pallet_direction_value,
                                                     const std::string& left_point_models_json_value,
                                                     const std::string& box_length_mm_value,
                                                     const std::string& box_width_mm_value,
                                                     const std::string& box_height_mm_value)
    :
    task_uuid(task_uuid_value),
    pallet_direction(pallet_direction_value),
    left_point_models_json(left_point_models_json_value),
    box_length_mm(box_length_mm_value),
    box_width_mm(box_width_mm_value),
    box_height_mm(box_height_mm_value) {}

std::string PalletizingTemplate::TemplateName() const
{
    return "palletizing";
}

robot_process_platform::core::Task PalletizingTemplate::CreateTask(const std::string& task_context_json) const
{
    const PalletizingTaskParameters task_parameters = ParseTaskParametersFromJson(task_context_json);
    const robot_process_platform::plugin::ProcessPlan process_plan =
        BuildProcessPlan(task_context_json, task_parameters);
    return BuildTaskFromPlan(process_plan);
}

robot_process_platform::plugin::ProcessPlan PalletizingTemplate::BuildProcessPlan(
    const std::string& task_context_json,
    const PalletizingTaskParameters& task_parameters) const
{
    robot_process_platform::plugin::ProcessPlan process_plan(
        task_parameters.task_uuid,
        TemplateName(),
        task_context_json);

    const auto extract_number_value =
        [&](const std::string& text, const std::string& key) -> std::string
        {
            const std::string key_token = "\"" + key + "\"";
            const std::size_t key_position = text.find(key_token);
            if (key_position == std::string::npos)
            {
                throw std::runtime_error("Missing number key: " + key);
            }

            const std::size_t colon_position = text.find(':', key_position);
            if (colon_position == std::string::npos)
            {
                throw std::runtime_error("Invalid number value for key: " + key);
            }

            std::size_t value_begin = colon_position + 1;
            while (value_begin < text.size() && std::isspace(static_cast<unsigned char>(text[value_begin])))
            {
                ++value_begin;
            }

            std::size_t value_end = value_begin;
            while (value_end < text.size())
            {
                const char current_character = text[value_end];
                if ((current_character >= '0' && current_character <= '9') ||
                    current_character == '-' || current_character == '.')
                {
                    ++value_end;
                    continue;
                }

                break;
            }

            if (value_begin == value_end)
            {
                throw std::runtime_error("Empty number value for key: " + key);
            }

            return text.substr(value_begin, value_end - value_begin);
        };

    const auto extract_inner_arrays =
        [&](const std::string& arrays_text) -> std::vector<std::string>
        {
            std::vector<std::string> inner_arrays;
            int depth = 0;
            std::size_t current_begin = std::string::npos;

            for (std::size_t index = 0; index < arrays_text.size(); ++index)
            {
                if (arrays_text[index] == '[')
                {
                    ++depth;
                    if (depth == 2)
                    {
                        current_begin = index;
                    }
                }
                else if (arrays_text[index] == ']')
                {
                    if (depth == 2 && current_begin != std::string::npos)
                    {
                        inner_arrays.push_back(arrays_text.substr(current_begin, index - current_begin + 1));
                        current_begin = std::string::npos;
                    }

                    --depth;
                }
            }

            return inner_arrays;
        };

    const auto extract_object_blocks =
        [&](const std::string& array_text) -> std::vector<std::string>
        {
            std::vector<std::string> object_blocks;
            int depth = 0;
            std::size_t current_begin = std::string::npos;

            for (std::size_t index = 0; index < array_text.size(); ++index)
            {
                if (array_text[index] == '{')
                {
                    if (depth == 0)
                    {
                        current_begin = index;
                    }

                    ++depth;
                }
                else if (array_text[index] == '}')
                {
                    --depth;
                    if (depth == 0 && current_begin != std::string::npos)
                    {
                        object_blocks.push_back(array_text.substr(current_begin, index - current_begin + 1));
                        current_begin = std::string::npos;
                    }
                }
            }

            return object_blocks;
        };

    const auto format_pose_from_object =
        [&](const std::string& point_object_text) -> std::string
        {
            std::ostringstream output_stream;
            output_stream << extract_number_value(point_object_text, "x") << ","
                          << extract_number_value(point_object_text, "y") << ","
                          << extract_number_value(point_object_text, "z") << ","
                          << extract_number_value(point_object_text, "rx") << ","
                          << extract_number_value(point_object_text, "ry") << ","
                          << extract_number_value(point_object_text, "rz");
            return output_stream.str();
        };

    const std::vector<std::string> box_point_groups =
        extract_inner_arrays(task_parameters.left_point_models_json);

    // 这里按箱子展开：leftPointModels 里的每个子数组都代表一个箱子的点位集合。
    for (std::size_t box_index = 0; box_index < box_point_groups.size(); ++box_index)
    {
        const std::vector<std::string> point_objects = extract_object_blocks(box_point_groups[box_index]);

        std::string pick_pose;
        std::string scan_pose;
        std::string pick_lift_pose;
        std::string transfer_pose;
        std::string place_pose;
        std::string place_lift_pose;

        for (const auto& point_object_text : point_objects)
        {
            const std::string handle_value = extract_number_value(point_object_text, "handle");

            if (handle_value == "3")
            {
                pick_pose = format_pose_from_object(point_object_text);
            }
            else if (handle_value == "6")
            {
                scan_pose = format_pose_from_object(point_object_text);
            }
            else if (handle_value == "7")
            {
                pick_lift_pose = format_pose_from_object(point_object_text);
            }
            else if (handle_value == "8")
            {
                transfer_pose = format_pose_from_object(point_object_text);
            }
            else if (handle_value == "4")
            {
                place_pose = format_pose_from_object(point_object_text);
            }
            else if (handle_value == "9")
            {
                place_lift_pose = format_pose_from_object(point_object_text);
            }
        }

        if (pick_pose.empty() || scan_pose.empty() || pick_lift_pose.empty() ||
            transfer_pose.empty() || place_pose.empty() || place_lift_pose.empty())
        {
            throw std::runtime_error("Box point group is missing required handle points.");
        }

        robot_process_platform::plugin::BoxPlan box_plan(static_cast<int>(box_index));

        robot_process_platform::plugin::ProcessBlockPlan pick_block_plan("pick_block");
        pick_block_plan.block_parameters["box_index"] = std::to_string(box_index);
        pick_block_plan.block_parameters["source_pose"] = pick_pose;
        pick_block_plan.block_parameters["pick_lift_pose"] = pick_lift_pose;
        pick_block_plan.block_parameters["box_length_mm"] = task_parameters.box_length_mm;
        pick_block_plan.block_parameters["box_width_mm"] = task_parameters.box_width_mm;
        pick_block_plan.block_parameters["box_height_mm"] = task_parameters.box_height_mm;
        pick_block_plan.block_parameters["pick_runtime_json"] = BuildUnitRuntimeJson(
            {{"box_index", std::to_string(box_index)},
             {"pick_confirm_timeout_ms", "1000"}});

        robot_process_platform::plugin::ProcessBlockPlan scan_block_plan("scan_block");
        scan_block_plan.block_parameters["box_index"] = std::to_string(box_index);
        scan_block_plan.block_parameters["scan_pose"] = scan_pose;
        scan_block_plan.block_parameters["scan_runtime_json"] = BuildUnitRuntimeJson(
            {{"box_index", std::to_string(box_index)},
             {"pallet_direction", task_parameters.pallet_direction}});

        robot_process_platform::plugin::ProcessBlockPlan transfer_block_plan("transfer_block");
        transfer_block_plan.block_parameters["box_index"] = std::to_string(box_index);
        transfer_block_plan.block_parameters["transfer_pose"] = transfer_pose;
        transfer_block_plan.block_parameters["transfer_runtime_json"] = BuildUnitRuntimeJson(
            {{"box_index", std::to_string(box_index)},
             {"transfer_pose", transfer_pose}});

        robot_process_platform::plugin::ProcessBlockPlan place_block_plan("place_block");
        place_block_plan.block_parameters["box_index"] = std::to_string(box_index);
        place_block_plan.block_parameters["target_pose"] = place_pose;
        place_block_plan.block_parameters["place_lift_pose"] = place_lift_pose;
        place_block_plan.block_parameters["box_length_mm"] = task_parameters.box_length_mm;
        place_block_plan.block_parameters["box_width_mm"] = task_parameters.box_width_mm;
        place_block_plan.block_parameters["box_height_mm"] = task_parameters.box_height_mm;
        place_block_plan.block_parameters["place_runtime_json"] = BuildUnitRuntimeJson(
            {{"box_index", std::to_string(box_index)},
             {"place_release_timeout_ms", "1000"}});

        box_plan.process_blocks.push_back(pick_block_plan);
        box_plan.process_blocks.push_back(scan_block_plan);
        box_plan.process_blocks.push_back(transfer_block_plan);
        box_plan.process_blocks.push_back(place_block_plan);
        process_plan.box_plans.push_back(box_plan);
    }

    return process_plan;
}

robot_process_platform::core::Task PalletizingTemplate::BuildTaskFromPlan(
    const robot_process_platform::plugin::ProcessPlan& process_plan) const
{
    robot_process_platform::core::Task task(process_plan.task_id, process_plan.template_name);

    // BuildTaskFromPlan 只负责收集每个工艺块展开后的连续执行块，
    // 不在这里重新计算箱子级工艺语义。
    for (const auto& box_plan : process_plan.box_plans)
    {
        for (const auto& process_block_plan : box_plan.process_blocks)
        {
            task.execution_blocks.push_back(BuildExecutionBlockFromProcessBlock(process_block_plan));
        }
    }

    return task;
}

robot_process_platform::core::ExecutionBlock PalletizingTemplate::BuildExecutionBlockFromProcessBlock(
    const robot_process_platform::plugin::ProcessBlockPlan& process_block_plan) const
{
    if (process_block_plan.block_name == "pick_block")
    {
        return BuildExecutionBlockForPickBlock(process_block_plan);
    }

    if (process_block_plan.block_name == "scan_block")
    {
        return BuildExecutionBlockForScanBlock(process_block_plan);
    }

    if (process_block_plan.block_name == "transfer_block")
    {
        return BuildExecutionBlockForTransferBlock(process_block_plan);
    }

    if (process_block_plan.block_name == "place_block")
    {
        return BuildExecutionBlockForPlaceBlock(process_block_plan);
    }

    throw std::runtime_error("Unsupported process block: " + process_block_plan.block_name);
}

robot_process_platform::core::ExecutionBlock PalletizingTemplate::BuildExecutionBlockForPickBlock(
    const robot_process_platform::plugin::ProcessBlockPlan& process_block_plan) const
{
    robot_process_platform::core::ExecutionBlock execution_block(
        "PickBlock",
        "box",
        std::stoi(process_block_plan.block_parameters.at("box_index")));
    execution_block.process_block_name = process_block_plan.block_name;

    execution_block.actions.push_back(
        robot_process_platform::core::Action(
            "MoveToSourcePose",
            robot_process_platform::core::TaskActionType::MoveJoint,
            {{"target_pose", process_block_plan.block_parameters.at("source_pose")}},
            3000));
    execution_block.actions.push_back(
        robot_process_platform::core::Action(
            "EnableGripper",
            robot_process_platform::core::TaskActionType::SetDigitalOutput,
            {{"signal_name", "gripper_close"}, {"signal_value", "true"}},
            1000));
    execution_block.actions.push_back(
        robot_process_platform::core::Action(
            "LiftFromSourcePose",
            robot_process_platform::core::TaskActionType::MoveLinear,
            {{"target_pose", process_block_plan.block_parameters.at("pick_lift_pose")}},
            3000));

    return execution_block;
}

robot_process_platform::core::ExecutionBlock PalletizingTemplate::BuildExecutionBlockForScanBlock(
    const robot_process_platform::plugin::ProcessBlockPlan& process_block_plan) const
{
    robot_process_platform::core::ExecutionBlock execution_block(
        "ScanBlock",
        "box",
        std::stoi(process_block_plan.block_parameters.at("box_index")));
    execution_block.process_block_name = process_block_plan.block_name;

    execution_block.actions.push_back(
        robot_process_platform::core::Action(
            "MoveToScanPose",
            robot_process_platform::core::TaskActionType::MoveJoint,
            {{"target_pose", process_block_plan.block_parameters.at("scan_pose")}},
            3000));
    execution_block.actions.push_back(
        robot_process_platform::core::Action(
            "TriggerScanner",
            robot_process_platform::core::TaskActionType::SetDigitalOutput,
            {{"signal_name", "scanner_trigger"}, {"signal_value", "true"}},
            1000));
    execution_block.actions.push_back(
        robot_process_platform::core::Action(
            "WaitForScanResult",
            robot_process_platform::core::TaskActionType::WaitDigitalInput,
            {{"signal_name", "scanner_done"}, {"expected_value", "true"}},
            3000));

    return execution_block;
}

robot_process_platform::core::ExecutionBlock PalletizingTemplate::BuildExecutionBlockForTransferBlock(
    const robot_process_platform::plugin::ProcessBlockPlan& process_block_plan) const
{
    robot_process_platform::core::ExecutionBlock execution_block(
        "TransferBlock",
        "box",
        std::stoi(process_block_plan.block_parameters.at("box_index")));
    execution_block.process_block_name = process_block_plan.block_name;

    execution_block.actions.push_back(
        robot_process_platform::core::Action(
            "MoveToTransferPose",
            robot_process_platform::core::TaskActionType::MoveJoint,
            {{"target_pose", process_block_plan.block_parameters.at("transfer_pose")}},
            3000));

    return execution_block;
}

robot_process_platform::core::ExecutionBlock PalletizingTemplate::BuildExecutionBlockForPlaceBlock(
    const robot_process_platform::plugin::ProcessBlockPlan& process_block_plan) const
{
    robot_process_platform::core::ExecutionBlock execution_block(
        "PlaceBlock",
        "box",
        std::stoi(process_block_plan.block_parameters.at("box_index")));
    execution_block.process_block_name = process_block_plan.block_name;

    execution_block.actions.push_back(
        robot_process_platform::core::Action(
            "MoveToTargetPose",
            robot_process_platform::core::TaskActionType::MoveLinear,
            {{"target_pose", process_block_plan.block_parameters.at("target_pose")}},
            3000));
    execution_block.actions.push_back(
        robot_process_platform::core::Action(
            "DisableGripper",
            robot_process_platform::core::TaskActionType::SetDigitalOutput,
            {{"signal_name", "gripper_close"}, {"signal_value", "false"}},
            1000));
    execution_block.actions.push_back(
        robot_process_platform::core::Action(
            "LiftFromTargetPose",
            robot_process_platform::core::TaskActionType::MoveLinear,
            {{"target_pose", process_block_plan.block_parameters.at("place_lift_pose")}},
            3000));

    return execution_block;
}

PalletizingTaskParameters PalletizingTemplate::ParseTaskParametersFromJson(
    const std::string& task_context_json) const
{
    const robot_process_platform::core::json::JsonValue root_value =
        robot_process_platform::core::json::JsonParser(task_context_json).Parse();
    const robot_process_platform::core::json::JsonValue& programme_value =
        robot_process_platform::core::json::GetObjectField(root_value, "programmeVo");
    const robot_process_platform::core::json::JsonValue& box_object =
        robot_process_platform::core::json::GetObjectField(programme_value, "box");
    const robot_process_platform::core::json::JsonValue& left_point_models =
        robot_process_platform::core::json::GetObjectField(root_value, "leftPointModels");

    return PalletizingTaskParameters(
        robot_process_platform::core::json::GetStringField(root_value, "uuid"),
        robot_process_platform::core::json::GetStringField(programme_value, "palletDirection"),
        robot_process_platform::core::json::SerializeJsonValue(left_point_models),
        robot_process_platform::core::json::GetNumberFieldAsString(box_object, "length"),
        robot_process_platform::core::json::GetNumberFieldAsString(box_object, "width"),
        robot_process_platform::core::json::GetNumberFieldAsString(box_object, "height"));
}

// 文件底部保留扩展空间：
// 如果后续这份 JSON 继续变复杂，优先继续直接改 ParseTaskParametersFromJson，
// 不急着拆很多通用 helper，先保持模板逻辑集中。

}  // namespace palletizing
