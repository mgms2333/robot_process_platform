#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "robot_process_platform/plugin/template_types.h"

namespace robot_process_platform::platform
{

// TemplateManager 负责模板插件的加载、卸载和模板实例创建。
class TemplateManager
{
public:
    using TemplateDeleter = std::function<void(plugin::ITemplate*)>;
    using TemplatePtr = std::unique_ptr<plugin::ITemplate, TemplateDeleter>;

    TemplateManager() = default;
    ~TemplateManager();

    bool LoadTemplateLibrary(const std::string& shared_library_path);
    bool UnloadTemplate(const std::string& template_name);
    TemplatePtr CreateTemplate(const std::string& template_name) const;

private:
    struct LoadedTemplateLibrary
    {
        void* library_handle = nullptr;
        plugin::ITemplate* (*create_template)() = nullptr;
        void (*destroy_template)(plugin::ITemplate*) = nullptr;
    };

    std::unordered_map<std::string, LoadedTemplateLibrary> loaded_templates;
};

}  // namespace robot_process_platform::platform
