#pragma once

#include <string>

#include "robot_process_platform/platform/template_manager.h"

namespace robot_process_platform::platform
{

// TaskManager 负责根据模板名称和上下文 JSON 生成 Task，并保存或删除本地任务文件。
class TaskManager
{
public:
    explicit TaskManager(TemplateManager& template_manager_value);

    core::Task CreateAndSaveTask(const std::string& template_name,
                                 const std::string& task_context_json,
                                 const std::string& local_directory_path) const;

    std::string CreateTask(const std::string& template_name,
                           const std::string& task_context_json,
                           const std::string& local_directory_path) const;

    bool DeleteTask(const std::string& task_id,
                    const std::string& local_directory_path) const;

private:
    std::string GenerateTaskId(const std::string& template_name) const;
    std::string BuildTaskFilePath(const std::string& task_id,
                                  const std::string& local_directory_path) const;

    TemplateManager& template_manager;
};

}  // namespace robot_process_platform::platform
