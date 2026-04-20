#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "robot_process_platform/platform/task_manager.h"
#include "robot_process_platform/platform/template_manager.h"

namespace
{

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

}  // namespace

int main()
{
    const std::string template_library_path = "./build/plugins/libpalletizing_template.so";
    const std::string task_context_file_path = "./tests/97582da7-be0d-4139-81cc-8d0775bdd49e.json";
    const std::string task_output_directory = "./tests/generated_tasks";

    robot_process_platform::platform::TemplateManager template_manager;
    robot_process_platform::platform::TaskManager task_manager(template_manager);

    const bool load_success = template_manager.LoadTemplateLibrary(template_library_path);
    if (!load_success)
    {
        std::cout << "failed_to_load_template" << '\n';
        return 1;
    }

    const std::string task_context_json = ReadAllText(task_context_file_path);
    const std::string task_id =
        task_manager.CreateTask("palletizing", task_context_json, task_output_directory);

    std::cout << "palletizing_template_loaded" << '\n';
    std::cout << "created_task_id=" << task_id << '\n';
    std::cout << "task_output_directory=" << task_output_directory << '\n';
    return 0;
}
