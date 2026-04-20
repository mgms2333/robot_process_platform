#include "robot_process_platform/platform/task_manager.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

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
    output_stream << "  \"steps\": [\n";

    for (std::size_t step_index = 0; step_index < task.steps.size(); ++step_index)
    {
        const auto& step = task.steps[step_index];
        output_stream << "    {\n";
        output_stream << "      \"step_name\": \"" << EscapeJsonString(step.step_name) << "\",\n";
        output_stream << "      \"priority\": " << step.priority << ",\n";
        output_stream << "      \"actions\": [\n";

        for (std::size_t action_index = 0; action_index < step.actions.size(); ++action_index)
        {
            const auto& action = step.actions[action_index];
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

            if (action_index + 1 < step.actions.size())
            {
                output_stream << ",";
            }

            output_stream << "\n";
        }

        output_stream << "      ]\n";
        output_stream << "    }";

        if (step_index + 1 < task.steps.size())
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

std::string TaskManager::CreateTask(const std::string& template_name,
                                    const std::string& task_context_json,
                                    const std::string& local_directory_path) const
{
    std::filesystem::create_directories(local_directory_path);

    TemplateManager::TemplatePtr template_instance = template_manager.CreateTemplate(template_name);
    robot_process_platform::core::Task created_task =
        template_instance->CreateTask(task_context_json);
    created_task.task_id = GenerateTaskId(template_name);

    std::ofstream output_file(BuildTaskFilePath(created_task.task_id, local_directory_path));
    if (!output_file.is_open())
    {
        throw std::runtime_error("Failed to create task file for task: " + created_task.task_id);
    }

    output_file << SerializeTaskToJson(created_task);
    return created_task.task_id;
}

bool TaskManager::DeleteTask(const std::string& task_id,
                             const std::string& local_directory_path) const
{
    return std::filesystem::remove(BuildTaskFilePath(task_id, local_directory_path));
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
