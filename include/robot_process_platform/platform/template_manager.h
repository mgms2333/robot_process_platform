#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "robot_process_platform/core/error_code.h"
#include "robot_process_platform/plugin/template_types.h"

namespace robot_process_platform::platform
{

// TemplateManager 负责模板插件的加载、卸载和模板实例创建。
class TemplateManager
{
public:
    using TemplateDeleter = std::function<void(plugin::ITemplate*)>;
    using TemplatePtr = std::unique_ptr<plugin::ITemplate, TemplateDeleter>;

    // TemplateInfo 用于向上层返回已加载模板的基础元信息。
    // 当前先提供模板名和库路径，后续可以继续扩展版本、安装时间、描述等字段。
    struct TemplateInfo
    {
        std::string template_name;
        std::string shared_library_path;
    };

    TemplateManager() = default;
    ~TemplateManager();

    core::Status LoadTemplateLibrary(const std::string& shared_library_path);
    core::Status UnloadTemplate(const std::string& template_name);
    core::Result<TemplatePtr> CreateTemplate(const std::string& template_name) const;
    std::vector<TemplateInfo> GetLoadedTemplates() const;

private:
    struct LoadedTemplateLibrary
    {
        void* library_handle = nullptr;
        plugin::ITemplate* (*create_template)() = nullptr;
        void (*destroy_template)(plugin::ITemplate*) = nullptr;
        std::string shared_library_path;
    };

    std::unordered_map<std::string, LoadedTemplateLibrary> loaded_templates;
};

}  // namespace robot_process_platform::platform
