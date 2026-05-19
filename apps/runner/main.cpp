#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

#include "robot_process_platform/core/error_code.h"
#include "robot_process_platform/platform/logger.h"
#include "robot_process_platform/platform/platform_service.h"
#include "robot_process_platform/runtime/execution_context.h"

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
    const std::string platform_config_file_path = "./config/platform_config.json";
    const std::string log_directory_path = "./tests/logs";

    robot_process_platform::platform::Logger::GetInstance().Initialize(log_directory_path, true);
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Runner started.");

    robot_process_platform::platform::PlatformService platform_service;

    const robot_process_platform::core::Status initialize_platform_status =
        platform_service.Initialize(platform_config_file_path, false);
    if (!initialize_platform_status.Ok())
    {
        robot_process_platform::platform::Logger::GetInstance().LogE("Runner", "Failed to initialize platform service.");
        std::cout << "failed_to_initialize_platform_service" << '\n';
        std::cout << "error_code=" << robot_process_platform::core::ToString(initialize_platform_status.code) << '\n';
        if (!initialize_platform_status.message.empty())
        {
            std::cout << "error_message=" << initialize_platform_status.message << '\n';
        }
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Platform service initialized.");

    const robot_process_platform::core::Status template_load_status =
        platform_service.LoadTemplateLibrary(template_library_file_path);
    if (!template_load_status.Ok())
    {
        robot_process_platform::platform::Logger::GetInstance().LogE("Runner", "Failed to load template library.");
        std::cout << "failed_to_load_template" << '\n';
        std::cout << "error_code=" << robot_process_platform::core::ToString(template_load_status.code) << '\n';
        if (!template_load_status.message.empty())
        {
            std::cout << "error_message=" << template_load_status.message << '\n';
        }
        platform_service.Shutdown();
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Template library loaded.");

    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Reading task context json: " + task_context_json_file_path);
    const std::string task_context_json_text = ReadAllText(task_context_json_file_path);
    if (task_context_json_text.empty())
    {
        robot_process_platform::platform::Logger::GetInstance().LogE("Runner", "Failed to read task context json.");
        std::cout << "failed_to_read_task_context_json" << '\n';
        platform_service.Shutdown();
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }

    const robot_process_platform::core::Result<robot_process_platform::core::Task> task_creation_result =
        platform_service.CreateTask(template_name,
                                    task_context_json_text,
                                    generated_task_directory_path);
    if (!task_creation_result.Ok())
    {
        std::cout << "create_task_failed" << '\n';
        std::cout << "error_code=" << robot_process_platform::core::ToString(task_creation_result.code) << '\n';
        if (!task_creation_result.message.empty())
        {
            std::cout << "error_message=" << task_creation_result.message << '\n';
        }
        platform_service.Shutdown();
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Task created successfully: " + task_creation_result.value->task_id);

    const robot_process_platform::core::Task& created_task = task_creation_result.value.value();

    const robot_process_platform::core::Status start_task_status =
        platform_service.StartTask(created_task.task_id, generated_task_directory_path, 0, 0);
    if (!start_task_status.Ok())
    {
        robot_process_platform::platform::Logger::GetInstance().LogE("Runner", "Failed to start task by id.");
        std::cout << "start_task_failed" << '\n';
        std::cout << "error_code=" << robot_process_platform::core::ToString(start_task_status.code) << '\n';
        if (!start_task_status.message.empty())
        {
            std::cout << "error_message=" << start_task_status.message << '\n';
        }
        platform_service.Shutdown();
        robot_process_platform::platform::Logger::GetInstance().Shutdown();
        return 1;
    }
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Platform StartTask completed.");
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Runtime execution loop started.");

    robot_process_platform::runtime::ExecutionContext runtime_snapshot =
        platform_service.GetRuntimeSnapshot();
    while (!IsTerminalState(runtime_snapshot.runtime_state))
    {
        std::this_thread::sleep_for(kRuntimePollInterval);
        runtime_snapshot = platform_service.GetRuntimeSnapshot();
    }

    const robot_process_platform::platform::PlatformState platform_state =
        platform_service.GetPlatformState();
    platform_service.Shutdown();

    const bool run_success = runtime_snapshot.runtime_state == robot_process_platform::runtime::RuntimeState::Completed;

    std::cout << "palletizing_template_loaded" << '\n';
    std::cout << "created_task_id=" << created_task.task_id << '\n';
    std::cout << "task_output_directory=" << generated_task_directory_path << '\n';
    std::cout << "active_robot_name=" << platform_state.active_robot_name << '\n';
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

    const robot_process_platform::core::Result<robot_process_platform::device::IRobot*> active_robot_result =
        platform_service.GetActiveRobot();
    robot_process_platform::device::HyRobot* active_hy_robot = nullptr;
    if (active_robot_result.Ok())
    {
        active_hy_robot =
            dynamic_cast<robot_process_platform::device::HyRobot*>(active_robot_result.value.value());
    }

    std::cout << "hy_robot_log_begin" << '\n';
    if (active_hy_robot != nullptr)
    {
        for (const auto& execution_log_entry : active_hy_robot->GetExecutionLog())
        {
            std::cout << execution_log_entry << '\n';
        }
    }
    std::cout << "hy_robot_log_end" << '\n';
    robot_process_platform::platform::Logger::GetInstance().LogI("Runner", "Runner finished.");
    robot_process_platform::platform::Logger::GetInstance().Shutdown();
    return run_success ? 0 : 1;
}
