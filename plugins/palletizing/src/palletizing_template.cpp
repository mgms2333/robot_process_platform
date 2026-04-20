#include "palletizing/palletizing_template.h"

#include <array>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <vector>

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
    |     -> robot_process_platform::plugin::ProcessUnitPlan pick_unit
    |     -> robot_process_platform::plugin::ProcessUnitPlan scan_unit
    |     -> robot_process_platform::plugin::ProcessUnitPlan transfer_unit
    |     -> robot_process_platform::plugin::ProcessUnitPlan place_unit
    v
std::vector<robot_process_platform::plugin::ProcessUnitPlan> process_units
    |
    | BuildTaskFromPlan()
    | 输入：const robot_process_platform::plugin::ProcessPlan&
    v
robot_process_platform::core::Task task
    |
    | BuildStepsFromProcessUnit()
    | 输入：const robot_process_platform::plugin::ProcessUnitPlan&
    v
std::vector<robot_process_platform::core::Step> steps_for_unit
    |
    | BuildStepsForPickUnit()
    | BuildStepsForScanUnit()
    | BuildStepsForTransferUnit()
    | BuildStepsForPlaceUnit()
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

        robot_process_platform::plugin::ProcessUnitPlan pick_unit_plan("pick_unit");
        pick_unit_plan.unit_parameters["box_index"] = std::to_string(box_index);
        pick_unit_plan.unit_parameters["source_pose"] = pick_pose;
        pick_unit_plan.unit_parameters["pick_lift_pose"] = pick_lift_pose;
        pick_unit_plan.unit_parameters["box_length_mm"] = task_parameters.box_length_mm;
        pick_unit_plan.unit_parameters["box_width_mm"] = task_parameters.box_width_mm;
        pick_unit_plan.unit_parameters["box_height_mm"] = task_parameters.box_height_mm;
        pick_unit_plan.unit_parameters["pick_runtime_json"] = BuildUnitRuntimeJson(
            {{"box_index", std::to_string(box_index)},
             {"pick_confirm_timeout_ms", "1000"}});

        robot_process_platform::plugin::ProcessUnitPlan scan_unit_plan("scan_unit");
        scan_unit_plan.unit_parameters["box_index"] = std::to_string(box_index);
        scan_unit_plan.unit_parameters["scan_pose"] = scan_pose;
        scan_unit_plan.unit_parameters["scan_runtime_json"] = BuildUnitRuntimeJson(
            {{"box_index", std::to_string(box_index)},
             {"pallet_direction", task_parameters.pallet_direction}});

        robot_process_platform::plugin::ProcessUnitPlan transfer_unit_plan("transfer_unit");
        transfer_unit_plan.unit_parameters["box_index"] = std::to_string(box_index);
        transfer_unit_plan.unit_parameters["transfer_pose"] = transfer_pose;
        transfer_unit_plan.unit_parameters["transfer_runtime_json"] = BuildUnitRuntimeJson(
            {{"box_index", std::to_string(box_index)},
             {"transfer_pose", transfer_pose}});

        robot_process_platform::plugin::ProcessUnitPlan place_unit_plan("place_unit");
        place_unit_plan.unit_parameters["box_index"] = std::to_string(box_index);
        place_unit_plan.unit_parameters["target_pose"] = place_pose;
        place_unit_plan.unit_parameters["place_lift_pose"] = place_lift_pose;
        place_unit_plan.unit_parameters["box_length_mm"] = task_parameters.box_length_mm;
        place_unit_plan.unit_parameters["box_width_mm"] = task_parameters.box_width_mm;
        place_unit_plan.unit_parameters["box_height_mm"] = task_parameters.box_height_mm;
        place_unit_plan.unit_parameters["place_runtime_json"] = BuildUnitRuntimeJson(
            {{"box_index", std::to_string(box_index)},
             {"place_release_timeout_ms", "1000"}});

        process_plan.process_units.push_back(pick_unit_plan);
        process_plan.process_units.push_back(scan_unit_plan);
        process_plan.process_units.push_back(transfer_unit_plan);
        process_plan.process_units.push_back(place_unit_plan);
    }

    return process_plan;
}

robot_process_platform::core::Task PalletizingTemplate::BuildTaskFromPlan(
    const robot_process_platform::plugin::ProcessPlan& process_plan) const
{
    robot_process_platform::core::Task task(process_plan.task_id, process_plan.template_name);

    // BuildTaskFromPlan 只负责收集每个工艺单元展开后的 Step，
    // 不在这里直接拼接 Action。
    for (const auto& process_unit_plan : process_plan.process_units)
    {
        const std::vector<robot_process_platform::core::Step> steps_for_unit =
            BuildStepsFromProcessUnit(process_unit_plan);

        for (const auto& step : steps_for_unit)
        {
            task.steps.push_back(step);
        }
    }

    return task;
}

std::vector<robot_process_platform::core::Step> PalletizingTemplate::BuildStepsFromProcessUnit(
    const robot_process_platform::plugin::ProcessUnitPlan& process_unit_plan) const
{
    if (process_unit_plan.unit_name == "pick_unit")
    {
        return BuildStepsForPickUnit(process_unit_plan);
    }

    if (process_unit_plan.unit_name == "scan_unit")
    {
        return BuildStepsForScanUnit(process_unit_plan);
    }

    if (process_unit_plan.unit_name == "transfer_unit")
    {
        return BuildStepsForTransferUnit(process_unit_plan);
    }

    if (process_unit_plan.unit_name == "place_unit")
    {
        return BuildStepsForPlaceUnit(process_unit_plan);
    }

    throw std::runtime_error("Unsupported process unit: " + process_unit_plan.unit_name);
}

std::vector<robot_process_platform::core::Step> PalletizingTemplate::BuildStepsForPickUnit(
    const robot_process_platform::plugin::ProcessUnitPlan& process_unit_plan) const
{
    std::vector<robot_process_platform::core::Step> steps;

    robot_process_platform::core::Step descend_to_pick_step("DescendToPickPoint");
    descend_to_pick_step.actions.push_back(
        robot_process_platform::core::Action(
            "MoveToSourcePose",
            robot_process_platform::core::TaskActionType::MoveJoint,
            {{"target_pose", process_unit_plan.unit_parameters.at("source_pose")}},
            3000));
    steps.push_back(descend_to_pick_step);

    robot_process_platform::core::Step enable_vacuum_step("EnableVacuum");
    enable_vacuum_step.actions.push_back(
        robot_process_platform::core::Action(
            "EnableGripper",
            robot_process_platform::core::TaskActionType::SetDigitalOutput,
            {{"signal_name", "gripper_close"}, {"signal_value", "true"}},
            1000));
    steps.push_back(enable_vacuum_step);

    robot_process_platform::core::Step lift_after_pick_step("LiftAfterPick");
    lift_after_pick_step.actions.push_back(
        robot_process_platform::core::Action(
            "LiftFromSourcePose",
            robot_process_platform::core::TaskActionType::MoveLinear,
            {{"target_pose", process_unit_plan.unit_parameters.at("pick_lift_pose")}},
            3000));
    steps.push_back(lift_after_pick_step);

    return steps;
}

std::vector<robot_process_platform::core::Step> PalletizingTemplate::BuildStepsForScanUnit(
    const robot_process_platform::plugin::ProcessUnitPlan& process_unit_plan) const
{
    std::vector<robot_process_platform::core::Step> steps;

    robot_process_platform::core::Step move_to_scan_step("MoveToScanPose");
    move_to_scan_step.actions.push_back(
        robot_process_platform::core::Action(
            "MoveToScanPose",
            robot_process_platform::core::TaskActionType::MoveJoint,
            {{"target_pose", process_unit_plan.unit_parameters.at("scan_pose")}},
            3000));
    steps.push_back(move_to_scan_step);

    robot_process_platform::core::Step trigger_scan_step("TriggerScanner");
    trigger_scan_step.actions.push_back(
        robot_process_platform::core::Action(
            "TriggerScanner",
            robot_process_platform::core::TaskActionType::SetDigitalOutput,
            {{"signal_name", "scanner_trigger"}, {"signal_value", "true"}},
            1000));
    steps.push_back(trigger_scan_step);

    robot_process_platform::core::Step wait_scan_result_step("WaitScanResult");
    wait_scan_result_step.actions.push_back(
        robot_process_platform::core::Action(
            "WaitForScanResult",
            robot_process_platform::core::TaskActionType::WaitDigitalInput,
            {{"signal_name", "scanner_done"}, {"expected_value", "true"}},
            3000));
    steps.push_back(wait_scan_result_step);

    return steps;
}

std::vector<robot_process_platform::core::Step> PalletizingTemplate::BuildStepsForTransferUnit(
    const robot_process_platform::plugin::ProcessUnitPlan& process_unit_plan) const
{
    std::vector<robot_process_platform::core::Step> steps;

    robot_process_platform::core::Step move_to_transfer_step("MoveToTransferPose");
    move_to_transfer_step.actions.push_back(
        robot_process_platform::core::Action(
            "MoveToTransferPose",
            robot_process_platform::core::TaskActionType::MoveJoint,
            {{"target_pose", process_unit_plan.unit_parameters.at("transfer_pose")}},
            3000));
    steps.push_back(move_to_transfer_step);

    return steps;
}

std::vector<robot_process_platform::core::Step> PalletizingTemplate::BuildStepsForPlaceUnit(
    const robot_process_platform::plugin::ProcessUnitPlan& process_unit_plan) const
{
    std::vector<robot_process_platform::core::Step> steps;

    robot_process_platform::core::Step descend_to_place_step("DescendToPlacePoint");
    descend_to_place_step.actions.push_back(
        robot_process_platform::core::Action(
            "MoveToTargetPose",
            robot_process_platform::core::TaskActionType::MoveLinear,
            {{"target_pose", process_unit_plan.unit_parameters.at("target_pose")}},
            3000));
    steps.push_back(descend_to_place_step);

    robot_process_platform::core::Step disable_vacuum_step("DisableVacuum");
    disable_vacuum_step.actions.push_back(
        robot_process_platform::core::Action(
            "DisableGripper",
            robot_process_platform::core::TaskActionType::SetDigitalOutput,
            {{"signal_name", "gripper_close"}, {"signal_value", "false"}},
            1000));
    steps.push_back(disable_vacuum_step);

    robot_process_platform::core::Step lift_after_place_step("LiftAfterPlace");
    lift_after_place_step.actions.push_back(
        robot_process_platform::core::Action(
            "LiftFromTargetPose",
            robot_process_platform::core::TaskActionType::MoveLinear,
            {{"target_pose", process_unit_plan.unit_parameters.at("place_lift_pose")}},
            3000));
    steps.push_back(lift_after_place_step);

    return steps;
}

PalletizingTaskParameters PalletizingTemplate::ParseTaskParametersFromJson(
    const std::string& task_context_json) const
{
    // 当前这里故意保持为一个模板专用的大解析函数，
    // 方便后续直接按业务需要继续改，不再拆很多零散 helper。

    const auto extract_string_value =
        [&](const std::string& text, const std::string& key) -> std::string
        {
            const std::string key_token = "\"" + key + "\"";
            const std::size_t key_position = text.find(key_token);
            if (key_position == std::string::npos)
            {
                throw std::runtime_error("Missing string key: " + key);
            }

            const std::size_t colon_position = text.find(':', key_position);
            const std::size_t first_quote = text.find('"', colon_position + 1);
            const std::size_t second_quote = text.find('"', first_quote + 1);
            if (colon_position == std::string::npos || first_quote == std::string::npos || second_quote == std::string::npos)
            {
                throw std::runtime_error("Invalid string value for key: " + key);
            }

            return text.substr(first_quote + 1, second_quote - first_quote - 1);
        };

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

    const auto extract_object_text =
        [&](const std::string& text, const std::string& object_key) -> std::string
        {
            const std::string key_token = "\"" + object_key + "\"";
            const std::size_t key_position = text.find(key_token);
            if (key_position == std::string::npos)
            {
                throw std::runtime_error("Missing object key: " + object_key);
            }

            const std::size_t object_begin = text.find('{', key_position);
            if (object_begin == std::string::npos)
            {
                throw std::runtime_error("Object begin not found for key: " + object_key);
            }

            int depth = 0;
            for (std::size_t index = object_begin; index < text.size(); ++index)
            {
                if (text[index] == '{')
                {
                    ++depth;
                }
                else if (text[index] == '}')
                {
                    --depth;
                    if (depth == 0)
                    {
                        return text.substr(object_begin, index - object_begin + 1);
                    }
                }
            }

            throw std::runtime_error("Object end not found for key: " + object_key);
        };

    const auto extract_array_text =
        [&](const std::string& text, const std::string& array_key) -> std::string
        {
            const std::string key_token = "\"" + array_key + "\"";
            const std::size_t key_position = text.find(key_token);
            if (key_position == std::string::npos)
            {
                throw std::runtime_error("Missing array key: " + array_key);
            }

            const std::size_t array_begin = text.find('[', key_position);
            if (array_begin == std::string::npos)
            {
                throw std::runtime_error("Array begin not found for key: " + array_key);
            }

            int depth = 0;
            for (std::size_t index = array_begin; index < text.size(); ++index)
            {
                if (text[index] == '[')
                {
                    ++depth;
                }
                else if (text[index] == ']')
                {
                    --depth;
                    if (depth == 0)
                    {
                        return text.substr(array_begin, index - array_begin + 1);
                    }
                }
            }

            throw std::runtime_error("Array end not found for key: " + array_key);
        };

    const std::string box_object_text = extract_object_text(task_context_json, "box");
    const std::string left_point_models_json = extract_array_text(task_context_json, "leftPointModels");
    const std::string box_height_mm = extract_number_value(box_object_text, "height");

    return PalletizingTaskParameters(
        extract_string_value(task_context_json, "uuid"),
        extract_string_value(task_context_json, "palletDirection"),
        left_point_models_json,
        extract_number_value(box_object_text, "length"),
        extract_number_value(box_object_text, "width"),
        box_height_mm);
}

// 文件底部保留扩展空间：
// 如果后续这份 JSON 继续变复杂，优先继续直接改 ParseTaskParametersFromJson，
// 不急着拆很多通用 helper，先保持模板逻辑集中。

}  // namespace palletizing
