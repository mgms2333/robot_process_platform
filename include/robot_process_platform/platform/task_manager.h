#pragma once

#include <string>

#include "robot_process_platform/core/error_code.h"
#include "robot_process_platform/platform/template_manager.h"

namespace robot_process_platform::platform
{

// TaskManager 负责根据模板名称和上下文 JSON 生成 Task，并保存或删除本地任务文件。
class TaskManager
{
public:
    explicit TaskManager(TemplateManager& template_manager_value);

    // 调用指定模板创建 Task，并将生成结果保存为本地 task json。
    core::Result<core::Task> CreateAndSaveTask(const std::string& template_name,
                                               const std::string& task_context_json,
                                               const std::string& local_directory_path) const;

    // 创建并保存 Task，只向调用方返回 task_id。
    core::Result<std::string> CreateTask(const std::string& template_name,
                                         const std::string& task_context_json,
                                         const std::string& local_directory_path) const;

    // 根据 task_id 从本地任务文件中读取并反序列化 Task。
    core::Result<core::Task> LoadTask(const std::string& task_id,
                                      const std::string& local_directory_path) const;

    // 删除指定 task_id 对应的本地任务文件。
    core::Status DeleteTask(const std::string& task_id,
                            const std::string& local_directory_path) const;

private:
    // task_id 由模板名和时间戳组成，保证本地任务文件名稳定可追踪。
    std::string GenerateTaskId(const std::string& template_name) const;

    // 统一生成任务文件绝对路径，避免创建、加载、删除时各自拼接。
    std::string BuildTaskFilePath(const std::string& task_id,
                                  const std::string& local_directory_path) const;

    TemplateManager& template_manager;
};

}  // namespace robot_process_platform::platform
