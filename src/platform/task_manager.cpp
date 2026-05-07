#include "robot_process_platform/platform/task_manager.h"

#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "robot_process_platform/core/json_utils.h"
#include "robot_process_platform/platform/logger.h"

namespace robot_process_platform::platform
{

namespace
{

// TaskManager 持久化任务时使用的平台内部 json 格式。
std::string SerializeTaskToJson(const robot_process_platform::core::Task& task)
{
    std::ostringstream output_stream;
    output_stream << "{\n";
    output_stream << "  \"task_id\": \""
                  << robot_process_platform::core::json::EscapeJsonString(task.task_id) << "\",\n";
    output_stream << "  \"template_name\": \""
                  << robot_process_platform::core::json::EscapeJsonString(task.template_name) << "\",\n";
    output_stream << "  \"execution_blocks\": [\n";

    for (std::size_t block_index = 0; block_index < task.execution_blocks.size(); ++block_index)
    {
        const auto& execution_block = task.execution_blocks[block_index];
        output_stream << "    {\n";
        output_stream << "      \"block_name\": \""
                      << robot_process_platform::core::json::EscapeJsonString(execution_block.block_name)
                      << "\",\n";
        output_stream << "      \"semantic_type\": \""
                      << robot_process_platform::core::json::EscapeJsonString(execution_block.semantic_type)
                      << "\",\n";
        output_stream << "      \"semantic_index\": " << execution_block.semantic_index << ",\n";
        output_stream << "      \"process_block_name\": \""
                      << robot_process_platform::core::json::EscapeJsonString(execution_block.process_block_name)
                      << "\",\n";
        output_stream << "      \"actions\": [\n";

        for (std::size_t action_index = 0; action_index < execution_block.actions.size(); ++action_index)
        {
            const auto& action = execution_block.actions[action_index];
            output_stream << "        {\n";
            output_stream << "          \"action_name\": \""
                          << robot_process_platform::core::json::EscapeJsonString(action.action_name) << "\",\n";
            output_stream << "          \"action_type\": \""
                          << robot_process_platform::core::json::EscapeJsonString(
                                 robot_process_platform::core::ToString(action.action_type))
                          << "\",\n";
            output_stream << "          \"timeout_ms\": " << action.timeout_ms << ",\n";
            output_stream << "          \"action_parameters\": {";

            std::size_t parameter_index = 0;
            for (const auto& parameter_entry : action.action_parameters)
            {
                if (parameter_index > 0)
                {
                    output_stream << ", ";
                }

                output_stream << "\""
                              << robot_process_platform::core::json::EscapeJsonString(parameter_entry.first)
                              << "\": "
                              << "\""
                              << robot_process_platform::core::json::EscapeJsonString(parameter_entry.second)
                              << "\"";
                ++parameter_index;
            }

            output_stream << "}\n";
            output_stream << "        }";

            if (action_index + 1 < execution_block.actions.size())
            {
                output_stream << ",";
            }

            output_stream << "\n";
        }

        output_stream << "      ]\n";
        output_stream << "    }";

        if (block_index + 1 < task.execution_blocks.size())
        {
            output_stream << ",";
        }

        output_stream << "\n";
    }

    output_stream << "  ]\n";
    output_stream << "}\n";
    return output_stream.str();
}

// 将持久化字符串动作类型还原为 TaskActionType 枚举。
robot_process_platform::core::TaskActionType ParseActionType(const std::string& action_type_name)
{
    using robot_process_platform::core::TaskActionType;

    if (action_type_name == "MoveJoint")
    {
        return TaskActionType::MoveJoint;
    }
    if (action_type_name == "MoveLinear")
    {
        return TaskActionType::MoveLinear;
    }
    if (action_type_name == "SetDigitalOutput")
    {
        return TaskActionType::SetDigitalOutput;
    }
    if (action_type_name == "WaitDigitalInput")
    {
        return TaskActionType::WaitDigitalInput;
    }
    if (action_type_name == "Delay")
    {
        return TaskActionType::Delay;
    }

    throw std::runtime_error("unknown_action_type: " + action_type_name);
}

// 读取完整文本文件，供加载 task json 使用。
std::string ReadAllText(const std::string& file_path)
{
    std::ifstream input_file(file_path);
    if (!input_file.is_open())
    {
        throw std::runtime_error("failed_to_open_file: " + file_path);
    }

    std::ostringstream text_stream;
    text_stream << input_file.rdbuf();
    return text_stream.str();
}

// 将平台保存的 task json 反序列化为统一 Task 模型。
robot_process_platform::core::Task DeserializeTaskFromJson(const std::string& task_json)
{
    const robot_process_platform::core::json::JsonValue root_value =
        robot_process_platform::core::json::JsonParser(task_json).Parse();

    robot_process_platform::core::Task task(
        robot_process_platform::core::json::GetStringField(root_value, "task_id"),
        robot_process_platform::core::json::GetStringField(root_value, "template_name"));

    const std::vector<robot_process_platform::core::json::JsonValue>& block_values =
        robot_process_platform::core::json::GetArrayField(root_value, "execution_blocks");
    for (const robot_process_platform::core::json::JsonValue& block_value : block_values)
    {
        robot_process_platform::core::ExecutionBlock execution_block(
            robot_process_platform::core::json::GetStringField(block_value, "block_name"),
            robot_process_platform::core::json::GetStringField(block_value, "semantic_type"),
            robot_process_platform::core::json::GetIntField(block_value, "semantic_index"));
        execution_block.process_block_name =
            robot_process_platform::core::json::GetStringField(block_value, "process_block_name");

        const std::vector<robot_process_platform::core::json::JsonValue>& action_values =
            robot_process_platform::core::json::GetArrayField(block_value, "actions");
        for (const robot_process_platform::core::json::JsonValue& action_value : action_values)
        {
            execution_block.actions.emplace_back(
                robot_process_platform::core::json::GetStringField(action_value, "action_name"),
                ParseActionType(
                    robot_process_platform::core::json::GetStringField(action_value, "action_type")),
                robot_process_platform::core::json::GetStringMapField(action_value, "action_parameters"),
                robot_process_platform::core::json::GetIntField(action_value, "timeout_ms"));
        }

        task.execution_blocks.push_back(std::move(execution_block));
    }

    return task;
}

}  // namespace

TaskManager::TaskManager(TemplateManager& template_manager_value)
    : template_manager(template_manager_value) {}

robot_process_platform::core::Result<robot_process_platform::core::Task> TaskManager::CreateAndSaveTask(
    const std::string& template_name,
    const std::string& task_context_json,
    const std::string& local_directory_path) const
{
    if (template_name.empty())
    {
        return core::MakeErrorResult<core::Task>(
            core::ErrorCode::InvalidArgument,
            "template_name_is_empty");
    }

    Logger::GetInstance().LogI(
        "TaskManager",
        "Creating task by template: " + template_name);

    try
    {
        std::filesystem::create_directories(local_directory_path);

        TemplateManager::TemplatePtr template_instance = template_manager.CreateTemplate(template_name);
        robot_process_platform::core::Task created_task = template_instance->CreateTask(task_context_json);
        created_task.task_id = GenerateTaskId(template_name);

        const std::string task_file_path =
            BuildTaskFilePath(created_task.task_id, local_directory_path);
        std::ofstream output_file(task_file_path);
        if (!output_file.is_open())
        {
            const std::string error_message =
                "failed_to_create_task_file: " + created_task.task_id;
            Logger::GetInstance().LogE("TaskManager", error_message);
            return core::MakeErrorResult<core::Task>(
                core::ErrorCode::TaskSaveFailed,
                error_message);
        }

        output_file << SerializeTaskToJson(created_task);
        Logger::GetInstance().LogI(
            "TaskManager",
            "Task created and saved: " + created_task.task_id + " -> " + task_file_path);
        return core::MakeSuccessResult<core::Task>(std::move(created_task));
    }
    catch (const std::runtime_error& exception)
    {
        const std::string exception_message = exception.what();
        const core::ErrorCode error_code =
            exception_message.find("Template is not loaded") != std::string::npos
                ? core::ErrorCode::TemplateNotFound
                : core::ErrorCode::TaskCreateFailed;
        Logger::GetInstance().LogE(
            "TaskManager",
            "CreateAndSaveTask failed: " + exception_message);
        return core::MakeErrorResult<core::Task>(error_code, exception_message);
    }
    catch (const std::exception& exception)
    {
        Logger::GetInstance().LogE(
            "TaskManager",
            "CreateAndSaveTask failed: " + std::string(exception.what()));
        return core::MakeErrorResult<core::Task>(
            core::ErrorCode::InternalError,
            exception.what());
    }
}

robot_process_platform::core::Result<std::string> TaskManager::CreateTask(
    const std::string& template_name,
    const std::string& task_context_json,
    const std::string& local_directory_path) const
{
    const robot_process_platform::core::Result<robot_process_platform::core::Task> create_result =
        CreateAndSaveTask(template_name, task_context_json, local_directory_path);
    if (!create_result.Ok())
    {
        return core::MakeErrorResult<std::string>(create_result.code, create_result.message);
    }

    return core::MakeSuccessResult<std::string>(create_result.value->task_id);
}

robot_process_platform::core::Result<robot_process_platform::core::Task> TaskManager::LoadTask(
    const std::string& task_id,
    const std::string& local_directory_path) const
{
    if (task_id.empty())
    {
        return core::MakeErrorResult<core::Task>(
            core::ErrorCode::InvalidArgument,
            "task_id_is_empty");
    }

    const std::string task_file_path = BuildTaskFilePath(task_id, local_directory_path);
    Logger::GetInstance().LogI(
        "TaskManager",
        "Loading task file: " + task_file_path);

    try
    {
        const std::string task_json = ReadAllText(task_file_path);
        robot_process_platform::core::Task task = DeserializeTaskFromJson(task_json);
        robot_process_platform::core::Result<robot_process_platform::core::Task> load_result =
            core::MakeSuccessResult<core::Task>(std::move(task));
        Logger::GetInstance().LogI(
            "TaskManager",
            "Task loaded successfully: " + load_result.value->task_id);
        return load_result;
    }
    catch (const std::runtime_error& exception)
    {
        const std::string exception_message = exception.what();
        const core::ErrorCode error_code =
            exception_message.find("failed_to_open_file") != std::string::npos
                ? core::ErrorCode::TaskFileNotFound
                : core::ErrorCode::TaskDeserializeFailed;
        Logger::GetInstance().LogE(
            "TaskManager",
            "LoadTask failed: " + exception_message);
        return core::MakeErrorResult<core::Task>(error_code, exception_message);
    }
    catch (const std::exception& exception)
    {
        Logger::GetInstance().LogE(
            "TaskManager",
            "LoadTask failed: " + std::string(exception.what()));
        return core::MakeErrorResult<core::Task>(
            core::ErrorCode::InternalError,
            exception.what());
    }
}

robot_process_platform::core::Status TaskManager::DeleteTask(
    const std::string& task_id,
    const std::string& local_directory_path) const
{
    try
    {
        const bool remove_success =
            std::filesystem::remove(BuildTaskFilePath(task_id, local_directory_path));

        if (remove_success)
        {
            Logger::GetInstance().LogI(
                "TaskManager",
                "Task file deleted: " + task_id);
            return core::MakeSuccessStatus();
        }

        Logger::GetInstance().LogW(
            "TaskManager",
            "Task file not found when deleting: " + task_id);
        return core::MakeErrorStatus(
            core::ErrorCode::TaskFileNotFound,
            "task_file_not_found: " + task_id);
    }
    catch (const std::exception& exception)
    {
        Logger::GetInstance().LogE(
            "TaskManager",
            "DeleteTask failed: " + std::string(exception.what()));
        return core::MakeErrorStatus(
            core::ErrorCode::TaskDeleteFailed,
            exception.what());
    }
}

std::string TaskManager::GenerateTaskId(const std::string& template_name) const
{
    const auto now = std::chrono::system_clock::now();
    const auto now_count = now.time_since_epoch().count();
    return template_name + "_" + std::to_string(now_count);
}

std::string TaskManager::BuildTaskFilePath(const std::string& task_id,
                                           const std::string& local_directory_path) const
{
    return local_directory_path + "/" + task_id + ".json";
}

}  // namespace robot_process_platform::platform
