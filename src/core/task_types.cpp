#include "robot_process_platform/core/task_types.h"

namespace robot_process_platform::core
{

Action::Action(const std::string& action_name_value,
               TaskActionType action_type_value,
               const std::map<std::string, std::string>& action_parameters_value,
               std::int32_t timeout_ms_value)
    :
    action_name(action_name_value),
    action_type(action_type_value),
    action_parameters(action_parameters_value),
    timeout_ms(timeout_ms_value) {}

Step::Step(const std::string& step_name_value)
    :
    step_name(step_name_value) {}

ExecutionBlock::ExecutionBlock(const std::string& block_name_value,
                               const std::string& semantic_type_value,
                               int semantic_index_value)
    :
    block_name(block_name_value),
    semantic_type(semantic_type_value),
    semantic_index(semantic_index_value) {}

Task::Task(const std::string& task_id_value, const std::string& template_name_value)
    :
    task_id(task_id_value),
    template_name(template_name_value) {}

std::string ToString(TaskActionType action_type)
{
    switch (action_type)
    {
        case TaskActionType::MoveJoint:
            return "MoveJoint";
        case TaskActionType::MoveLinear:
            return "MoveLinear";
        case TaskActionType::SetDigitalOutput:
            return "SetDigitalOutput";
        case TaskActionType::WaitDigitalInput:
            return "WaitDigitalInput";
        case TaskActionType::Delay:
            return "Delay";
    }

    return "Unknown";
}

}  // namespace robot_process_platform::core
