#include "robot_process_platform/platform/template_manager.h"

#include <dlfcn.h>
#include <stdexcept>

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
            dlclose(loaded_template_entry.second.library_handle);
        }
    }
}

bool TemplateManager::LoadTemplateLibrary(const std::string& shared_library_path)
{
    void* library_handle = dlopen(shared_library_path.c_str(), RTLD_NOW);
    if (library_handle == nullptr)
    {
        return false;
    }

    const auto get_template_name =
        reinterpret_cast<GetTemplateNameFunction>(dlsym(library_handle, "GetTemplateName"));
    const auto create_template =
        reinterpret_cast<CreateTemplateFunction>(dlsym(library_handle, "CreateTemplate"));
    const auto destroy_template =
        reinterpret_cast<DestroyTemplateFunction>(dlsym(library_handle, "DestroyTemplate"));

    if (get_template_name == nullptr || create_template == nullptr || destroy_template == nullptr)
    {
        dlclose(library_handle);
        return false;
    }

    const std::string template_name = get_template_name();

    LoadedTemplateLibrary loaded_template_library;
    loaded_template_library.library_handle = library_handle;
    loaded_template_library.create_template = create_template;
    loaded_template_library.destroy_template = destroy_template;
    loaded_templates[template_name] = loaded_template_library;
    return true;
}

bool TemplateManager::UnloadTemplate(const std::string& template_name)
{
    const auto loaded_template_it = loaded_templates.find(template_name);
    if (loaded_template_it == loaded_templates.end())
    {
        return false;
    }

    if (loaded_template_it->second.library_handle != nullptr)
    {
        dlclose(loaded_template_it->second.library_handle);
    }

    loaded_templates.erase(loaded_template_it);
    return true;
}

TemplateManager::TemplatePtr TemplateManager::CreateTemplate(const std::string& template_name) const
{
    const auto loaded_template_it = loaded_templates.find(template_name);
    if (loaded_template_it == loaded_templates.end())
    {
        throw std::runtime_error("Template is not loaded: " + template_name);
    }

    plugin::ITemplate* raw_template = loaded_template_it->second.create_template();
    return TemplatePtr(raw_template, loaded_template_it->second.destroy_template);
}

}  // namespace robot_process_platform::platform
