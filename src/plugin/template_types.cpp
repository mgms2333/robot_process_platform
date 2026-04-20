#include "robot_process_platform/plugin/template_types.h"

namespace robot_process_platform::plugin
{

ProcessUnitPlan::ProcessUnitPlan(const std::string& unit_name_value)
    :
    unit_name(unit_name_value) {}

ProcessPlan::ProcessPlan(const std::string& task_id_value,
                         const std::string& template_name_value,
                         const std::string& source_context_json_value)
    :
    task_id(task_id_value),
    template_name(template_name_value),
    source_context_json(source_context_json_value) {}

}  // namespace robot_process_platform::plugin
