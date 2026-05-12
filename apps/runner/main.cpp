#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

#include "robot_process_platform/core/error_code.h"
#include "robot_process_platform/device/robot/hy_robot.h"
#include "robot_process_platform/platform/logger.h"
#include "robot_process_platform/platform/task_manager.h"
#include "robot_process_platform/platform/template_manager.h"
#include "robot_process_platform/runtime/execution_context.h"
#include "robot_process_platform/runtime/runtime_service.h"

namespace
{

constexpr auto kRuntimePollInterval = std::chrono::milliseconds(10);

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

bool IsTerminalState(robot_process_platform::runtime::RuntimeState runtime_state)
{
    return runtime_state == robot_process_platform::runtime::RuntimeState::Completed ||
           runtime_state == robot_process_platform::runtime::RuntimeState::Stopped ||
           runtime_state == robot_process_platform::runtime::RuntimeState::Fault ||
           runtime_state == robot_process_platform::runtime::RuntimeState::EmergencyStop ||
           runtime_state == robot_process_platform::runtime::RuntimeState::Failed;
}

}  // namespace

int main()
{
    const std::string template_name = "palletizing";
    const std::string template_library_file_path = "./build/plugins/libpalletizing_template.so";
    const std::string task_context_json_file_path = "./tests/97582da7-be0d-4139-81cc-8d0775bdd49e.json";
    const std::string generated_task_directory_path = "./tests/generated_tasks";
    const std::string log_directory_path = "./tests/logs";

    robot_process_platform::platform::Logger::GetInstance().Initialize(log_directory_path, true);
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Runner started.");

    robot_process_platform::platform::TemplateManager template_manager;
    robot_process_platform::platform::TaskManager task_manager(template_manager);
    robot_process_platform::device::HyRobot hy_robot;
    robot_process_platform::runtime::RuntimeService runtime_service(hy_robot);

    const robot_process_platform::core::Status template_load_status =
        template_manager.LoadTemplateLibrary(template_library_file_path);
    if (!template_load_status.Ok())
    {
        robot_process_platform::platform::Logger::GetInstance().LogE("Runner", "Failed to load template library.");
        std::cout << "failed_to_load_template" << '\n';
        std::cout << "error_code=" << robot_process_platform::core::ToString(template_load_status.code) << '\n';
        if (!template_load_status.message.empty())
        {
            std::cout << "error_message=" << template_load_status.message << '\n';
        }
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Template library loaded.");

    const robot_process_platform::core::Status start_service_status = runtime_service.StartService();
    if (!start_service_status.Ok())
    {
        robot_process_platform::platform::Logger::GetInstance().LogE("Runner", "Failed to start runtime service.");
        std::cout << "failed_to_start_runtime_service" << '\n';
        std::cout << "error_code=" << robot_process_platform::core::ToString(start_service_status.code) << '\n';
        if (!start_service_status.message.empty())
        {
            std::cout << "error_message=" << start_service_status.message << '\n';
        }
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Runtime service started.");

    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Reading task context json: " + task_context_json_file_path);
    const std::string task_context_json_text = ReadAllText(task_context_json_file_path);
    if (task_context_json_text.empty())
    {
        robot_process_platform::platform::Logger::GetInstance().LogE("Runner", "Failed to read task context json.");
        std::cout << "failed_to_read_task_context_json" << '\n';
        runtime_service.StopService();
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }

    const robot_process_platform::core::Result<robot_process_platform::core::Task> task_creation_result =
        task_manager.CreateAndSaveTask(template_name, task_context_json_text, generated_task_directory_path);
    if (!task_creation_result.Ok())
    {
        std::cout << "create_task_failed" << '\n';
        std::cout << "error_code=" << robot_process_platform::core::ToString(task_creation_result.code) << '\n';
        if (!task_creation_result.message.empty())
        {
            std::cout << "error_message=" << task_creation_result.message << '\n';
        }
        runtime_service.StopService();
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Task created successfully: " + task_creation_result.value->task_id);

    const robot_process_platform::core::Task& created_task = task_creation_result.value.value();

    const robot_process_platform::core::Result<robot_process_platform::core::Task> task_reload_result =
        task_manager.LoadTask(created_task.task_id, generated_task_directory_path);
    if (!task_reload_result.Ok())
    {
        std::cout << "load_task_failed" << '\n';
        std::cout << "error_code=" << robot_process_platform::core::ToString(task_reload_result.code) << '\n';
        if (!task_reload_result.message.empty())
        {
            std::cout << "error_message=" << task_reload_result.message << '\n';
        }
        runtime_service.StopService();
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Task loaded successfully: " + task_reload_result.value->task_id);

    const robot_process_platform::core::Task& reloaded_task = task_reload_result.value.value();

    const robot_process_platform::core::Status load_task_command_status = runtime_service.SubmitLoadTask(reloaded_task);
    if (!load_task_command_status.Ok())
    {
        robot_process_platform::platform::Logger::GetInstance().LogE("Runner", "Failed to submit load task command.");
        std::cout << "submit_load_task_failed" << '\n';
        std::cout << "error_code=" << robot_process_platform::core::ToString(load_task_command_status.code) << '\n';
        if (!load_task_command_status.message.empty())
        {
            std::cout << "error_message=" << load_task_command_status.message << '\n';
        }
        runtime_service.StopService();
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "LoadTask command submitted.");

    const robot_process_platform::core::Status start_task_command_status = runtime_service.SubmitStartTask();
    if (!start_task_command_status.Ok())
    {
        robot_process_platform::platform::Logger::GetInstance().LogE("Runner", "Failed to submit start task command.");
        std::cout << "submit_start_task_failed" << '\n';
        std::cout << "error_code=" << robot_process_platform::core::ToString(start_task_command_status.code) << '\n';
        if (!start_task_command_status.message.empty())
        {
            std::cout << "error_message=" << start_task_command_status.message << '\n';
        }
        runtime_service.StopService();
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "StartTask command submitted.");
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Runtime execution loop started.");

    robot_process_platform::runtime::ExecutionContext runtime_snapshot = runtime_service.GetContextSnapshot();
    while (!IsTerminalState(runtime_snapshot.runtime_state))
    {
        std::this_thread::sleep_for(kRuntimePollInterval);
        runtime_snapshot = runtime_service.GetContextSnapshot();
    }

    runtime_service.StopService();

    const bool run_success = runtime_snapshot.runtime_state == robot_process_platform::runtime::RuntimeState::Completed;

    std::cout << "palletizing_template_loaded" << '\n';
    std::cout << "created_task_id=" << created_task.task_id << '\n';
    std::cout << "task_output_directory=" << generated_task_directory_path << '\n';
    std::cout << "runtime_state=" << robot_process_platform::runtime::ToString(runtime_snapshot.runtime_state) << '\n';
    std::cout << "execution_block_index=" << runtime_snapshot.current_block_index << '\n';
    std::cout << "execution_action_index=" << runtime_snapshot.current_action_index << '\n';
    std::cout << "execution_success=" << (run_success ? "true" : "false") << '\n';

    if (!runtime_snapshot.last_error.empty())
    {
        std::cout << "last_error_code=" << robot_process_platform::core::ToString(runtime_snapshot.last_error_code) << '\n';
        std::cout << "last_error=" << runtime_snapshot.last_error << '\n';
    }
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner",
                                                                 "Runtime execution loop finished: runtime_state=" +
                                                                     robot_process_platform::runtime::ToString(runtime_snapshot.runtime_state));

    std::cout << "hy_robot_log_begin" << '\n';
    for (const auto& execution_log_entry : hy_robot.GetExecutionLog())
    {
        std::cout << execution_log_entry << '\n';
    }
    std::cout << "hy_robot_log_end" << '\n';
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Runner finished.");
    robot_process_platform::platform::Logger::GetInstance().Shutdown();
    return run_success ? 0 : 1;
}
