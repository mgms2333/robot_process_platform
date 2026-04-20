#include "palletizing/palletizing_template.h"

#include "robot_process_platform/plugin/template_types.h"

namespace
{

palletizing::PalletizingTemplate g_palletizing_template;

}

extern "C" const char* GetTemplateName()
{
    return "palletizing";
}

extern "C" const robot_process_platform::plugin::ITemplate* CreateTemplate()
{
    return &g_palletizing_template;
}

extern "C" void DestroyTemplate(const robot_process_platform::plugin::ITemplate* template_instance)
{
    (void)template_instance;
}
