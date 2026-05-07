#include <fstream>
#include <iostream>
#include <sstream>

#include "robot_process_platform/core/error_code.h"
#include "robot_process_platform/platform/task_manager.h"
#include "robot_process_platform/platform/template_manager.h"
#include "robot_process_platform/platform/logger.h"
#include "robot_process_platform/device/robot/hy_robot.h"
#include "robot_process_platform/runtime/execution_context.h"
#include "robot_process_platform/runtime/task_runner.h"

namespace
{

std::string ReadAllText(const std::string& file_path)
{
    std::ifstream input_file(file_path);
    if (!input_file.is_open())
    {
        return "";
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
    const std::string log_output_directory = "./tests/logs";

    robot_process_platform::platform::Logger::GetInstance().Initialize(log_output_directory, true);
    robot_process_platform::platform::Logger::GetInstance().LogI(
        "Runner",
        "Runner started.");

    robot_process_platform::platform::TemplateManager template_manager;
    robot_process_platform::platform::TaskManager task_manager(template_manager);
    robot_process_platform::device::HyRobot hy_robot;
    robot_process_platform::runtime::TaskRunner task_runner(hy_robot);
    robot_process_platform::runtime::ExecutionContext execution_context;

    const bool load_success = template_manager.LoadTemplateLibrary(template_library_path);
    if (!load_success)
    {
        robot_process_platform::platform::Logger::GetInstance().LogE(
            "Runner",
            "Failed to load template library.");
        std::cout << "failed_to_load_template" << '\n';
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }

    const std::string task_context_json = ReadAllText(task_context_file_path);
    if (task_context_json.empty())
    {
        std::cout << "failed_to_read_task_context_json" << '\n';
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }

    const robot_process_platform::core::Result<robot_process_platform::core::Task> create_task_result =
        task_manager.CreateAndSaveTask("palletizing", task_context_json, task_output_directory);
    if (!create_task_result.Ok())
    {
        std::cout << "create_task_failed" << '\n';
        std::cout << "error_code="
                  << robot_process_platform::core::ToString(create_task_result.code) << '\n';
        if (!create_task_result.message.empty())
        {
            std::cout << "error_message=" << create_task_result.message << '\n';
        }
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }

    const robot_process_platform::core::Task& created_task = create_task_result.value.value();

    const robot_process_platform::core::Result<robot_process_platform::core::Task> load_task_result =
        task_manager.LoadTask(created_task.task_id, task_output_directory);
    if (!load_task_result.Ok())
    {
        std::cout << "load_task_failed" << '\n';
        std::cout << "error_code="
                  << robot_process_platform::core::ToString(load_task_result.code) << '\n';
        if (!load_task_result.message.empty())
        {
            std::cout << "error_message=" << load_task_result.message << '\n';
        }
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }

    const robot_process_platform::core::Task& loaded_task = load_task_result.value.value();

    bool run_success = task_runner.LoadTask(loaded_task, execution_context);
    if (run_success)
    {
        run_success = task_runner.Start(execution_context);
    }

    while (run_success &&
           execution_context.runtime_state == robot_process_platform::runtime::RuntimeState::Running)
    {
        run_success = task_runner.Tick(execution_context);
    }

    run_success = run_success &&
        execution_context.runtime_state == robot_process_platform::runtime::RuntimeState::Completed;

    std::cout << "palletizing_template_loaded" << '\n';
    std::cout << "created_task_id=" << created_task.task_id << '\n';
    std::cout << "task_output_directory=" << task_output_directory << '\n';
    std::cout << "runtime_state="
              << robot_process_platform::runtime::ToString(execution_context.runtime_state) << '\n';
    std::cout << "execution_block_index=" << execution_context.current_block_index << '\n';
    std::cout << "execution_action_index=" << execution_context.current_action_index << '\n';
    std::cout << "execution_success=" << (run_success ? "true" : "false") << '\n';

    if (!execution_context.last_error.empty())
    {
        std::cout << "last_error=" << execution_context.last_error << '\n';
    }

    std::cout << "hy_robot_log_begin" << '\n';
    for (const auto& execution_log_entry : hy_robot.GetExecutionLog())
    {
        std::cout << execution_log_entry << '\n';
    }
    std::cout << "hy_robot_log_end" << '\n';
    robot_process_platform::platform::Logger::GetInstance().LogI(
        "Runner",
        "Runner finished.");
    robot_process_platform::platform::Logger::GetInstance().Shutdown();
    return run_success ? 0 : 1;
}
