#include "robot_process_platform/platform/task_manager.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "robot_process_platform/platform/logger.h"

namespace robot_process_platform::platform
{

namespace
{

std::string EscapeJsonString(const std::string& raw_text)
{
    std::string escaped_text;
    for (const char character : raw_text)
    {
        if (character == '"' || character == '\\')
        {
            escaped_text.push_back('\\');
        }

        escaped_text.push_back(character);
    }

    return escaped_text;
}

std::string SerializeTaskToJson(const robot_process_platform::core::Task& task)
{
    std::ostringstream output_stream;
    output_stream << "{\n";
    output_stream << "  \"task_id\": \"" << EscapeJsonString(task.task_id) << "\",\n";
    output_stream << "  \"template_name\": \"" << EscapeJsonString(task.template_name) << "\",\n";
    output_stream << "  \"execution_blocks\": [\n";

    for (std::size_t block_index = 0; block_index < task.execution_blocks.size(); ++block_index)
    {
        const auto& execution_block = task.execution_blocks[block_index];
        output_stream << "    {\n";
        output_stream << "      \"block_name\": \"" << EscapeJsonString(execution_block.block_name) << "\",\n";
        output_stream << "      \"semantic_type\": \"" << EscapeJsonString(execution_block.semantic_type) << "\",\n";
        output_stream << "      \"semantic_index\": " << execution_block.semantic_index << ",\n";
        output_stream << "      \"process_block_name\": \""
                      << EscapeJsonString(execution_block.process_block_name) << "\",\n";
        output_stream << "      \"actions\": [\n";

        for (std::size_t action_index = 0; action_index < execution_block.actions.size(); ++action_index)
        {
            const auto& action = execution_block.actions[action_index];
            output_stream << "        {\n";
            output_stream << "          \"action_name\": \"" << EscapeJsonString(action.action_name) << "\",\n";
            output_stream << "          \"action_type\": \"" << EscapeJsonString(robot_process_platform::core::ToString(action.action_type)) << "\",\n";
            output_stream << "          \"timeout_ms\": " << action.timeout_ms << ",\n";
            output_stream << "          \"action_parameters\": {";

            std::size_t parameter_index = 0;
            for (const auto& parameter_entry : action.action_parameters)
            {
                if (parameter_index > 0)
                {
                    output_stream << ", ";
                }

                output_stream << "\"" << EscapeJsonString(parameter_entry.first) << "\": "
                              << "\"" << EscapeJsonString(parameter_entry.second) << "\"";
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

}  // namespace

TaskManager::TaskManager(TemplateManager& template_manager_value)
    :
    template_manager(template_manager_value) {}

robot_process_platform::core::Task TaskManager::CreateAndSaveTask(
    const std::string& template_name,
    const std::string& task_context_json,
    const std::string& local_directory_path) const
{
    Logger::GetInstance().LogI(
        "TaskManager",
        "Creating task by template: " + template_name);

    std::filesystem::create_directories(local_directory_path);

    TemplateManager::TemplatePtr template_instance = template_manager.CreateTemplate(template_name);
    robot_process_platform::core::Task created_task = template_instance->CreateTask(task_context_json);
    created_task.task_id = GenerateTaskId(template_name);

    const std::string task_file_path = BuildTaskFilePath(created_task.task_id, local_directory_path);
    std::ofstream output_file(task_file_path);
    if (!output_file.is_open())
    {
        Logger::GetInstance().LogE(
            "TaskManager",
            "Failed to create task file: " + created_task.task_id);
        throw std::runtime_error("Failed to create task file for task: " + created_task.task_id);
    }

    output_file << SerializeTaskToJson(created_task);
    Logger::GetInstance().LogI(
        "TaskManager",
        "Task created and saved: " + created_task.task_id + " -> " + task_file_path);
    return created_task;
}

std::string TaskManager::CreateTask(const std::string& template_name,
                                    const std::string& task_context_json,
                                    const std::string& local_directory_path) const
{
    const robot_process_platform::core::Task created_task =
        CreateAndSaveTask(template_name, task_context_json, local_directory_path);
    return created_task.task_id;
}

bool TaskManager::DeleteTask(const std::string& task_id,
                             const std::string& local_directory_path) const
{
    const bool remove_success =
        std::filesystem::remove(BuildTaskFilePath(task_id, local_directory_path));

    if (remove_success)
    {
        Logger::GetInstance().LogI(
            "TaskManager",
            "Task file deleted: " + task_id);
    }
    else
    {
        Logger::GetInstance().LogW(
            "TaskManager",
            "Task file not found when deleting: " + task_id);
    }

    return remove_success;
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
