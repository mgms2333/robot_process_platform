#include "robot_process_platform/platform/template_manager.h"

#include <dlfcn.h>
#include <stdexcept>

#include "robot_process_platform/platform/logger.h"

namespace robot_process_platform::platform
{

namespace
{

using GetTemplateNameFunction = const char* (*)();
using CreateTemplateFunction = robot_process_platform::plugin::ITemplate* (*)();
using DestroyTemplateFunction = void (*)(robot_process_platform::plugin::ITemplate*);

}  // namespace

TemplateManager::~TemplateManager()
{
    for (auto& loaded_template_entry : loaded_templates)
    {
        if (loaded_template_entry.second.library_handle != nullptr)
        {
            Logger::GetInstance().LogI("TemplateManager", "Unloading template library: " + loaded_template_entry.first);
            dlclose(loaded_template_entry.second.library_handle);
        }
    }
}

core::Status TemplateManager::LoadTemplateLibrary(const std::string& shared_library_path)
{
    Logger::GetInstance().LogI("TemplateManager", "Loading template library: " + shared_library_path);

    void* library_handle = dlopen(shared_library_path.c_str(), RTLD_NOW);
    if (library_handle == nullptr)
    {
        Logger::GetInstance().LogE("TemplateManager", "Failed to load template library: " + shared_library_path);
        return core::MakeErrorStatus(core::ErrorCode::TemplateLoadFailed,
                                     "failed_to_load_template_library: " + shared_library_path);
    }

    const auto get_template_name =
        reinterpret_cast<GetTemplateNameFunction>(dlsym(library_handle, "GetTemplateName"));
    const auto create_template =
        reinterpret_cast<CreateTemplateFunction>(dlsym(library_handle, "CreateTemplate"));
    const auto destroy_template =
        reinterpret_cast<DestroyTemplateFunction>(dlsym(library_handle, "DestroyTemplate"));

    if (get_template_name == nullptr || create_template == nullptr || destroy_template == nullptr)
    {
        Logger::GetInstance().LogE("TemplateManager", "Template library is missing required plugin symbols: " + shared_library_path);
        dlclose(library_handle);
        return core::MakeErrorStatus(core::ErrorCode::TemplateSymbolMissing,
                                     "template_library_missing_required_symbols: " + shared_library_path);
    }

    const std::string template_name = get_template_name();

    LoadedTemplateLibrary loaded_template_library;
    loaded_template_library.library_handle = library_handle;
    loaded_template_library.create_template = create_template;
    loaded_template_library.destroy_template = destroy_template;
    loaded_templates[template_name] = loaded_template_library;

    Logger::GetInstance().LogI("TemplateManager", "Template library loaded successfully: " + template_name);
    return core::MakeSuccessStatus();
}

core::Status TemplateManager::UnloadTemplate(const std::string& template_name)
{
    Logger::GetInstance().LogI("TemplateManager", "UnloadTemplate requested: " + template_name);
    const auto loaded_template_it = loaded_templates.find(template_name);
    if (loaded_template_it == loaded_templates.end())
    {
        Logger::GetInstance().LogW("TemplateManager", "Template is not loaded, cannot unload: " + template_name);
        return core::MakeErrorStatus(core::ErrorCode::TemplateNotFound,
                                     "template_is_not_loaded: " + template_name);
    }

    if (loaded_template_it->second.library_handle != nullptr)
    {
        Logger::GetInstance().LogI("TemplateManager", "Unloading template by name: " + template_name);
        dlclose(loaded_template_it->second.library_handle);
    }

    loaded_templates.erase(loaded_template_it);
    Logger::GetInstance().LogI("TemplateManager", "Template unloaded successfully: " + template_name);
    return core::MakeSuccessStatus();
}

core::Result<TemplateManager::TemplatePtr> TemplateManager::CreateTemplate(const std::string& template_name) const
{
    Logger::GetInstance().LogD("TemplateManager", "Creating template instance: " + template_name);
    const auto loaded_template_it = loaded_templates.find(template_name);
    if (loaded_template_it == loaded_templates.end())
    {
        Logger::GetInstance().LogE("TemplateManager", "CreateTemplate failed because template is not loaded: " + template_name);
        return core::MakeErrorResult<TemplatePtr>(core::ErrorCode::TemplateNotFound,
                                                  "template_is_not_loaded: " + template_name);
    }

    plugin::ITemplate* raw_template = loaded_template_it->second.create_template();
    if (raw_template == nullptr)
    {
        Logger::GetInstance().LogE("TemplateManager", "CreateTemplate returned null instance: " + template_name);
        return core::MakeErrorResult<TemplatePtr>(core::ErrorCode::TemplateCreateFailed,
                                                  "template_instance_is_null: " + template_name);
    }

    Logger::GetInstance().LogD("TemplateManager", "Template instance created: " + template_name);
    return core::MakeSuccessResult<TemplatePtr>(
        TemplatePtr(raw_template, loaded_template_it->second.destroy_template));
}

}  // namespace robot_process_platform::platform
