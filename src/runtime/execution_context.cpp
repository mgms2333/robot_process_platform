#include "robot_process_platform/runtime/execution_context.h"

namespace robot_process_platform::runtime
{

ExecutionContext::ExecutionContext() = default;

std::string ToString(RuntimeState runtime_state)
{
    switch (runtime_state)
    {
        case RuntimeState::Idle:
            return "Idle";
        case RuntimeState::Ready:
            return "Ready";
        case RuntimeState::Running:
            return "Running";
        case RuntimeState::Paused:
            return "Paused";
        case RuntimeState::Fault:
            return "Fault";
        case RuntimeState::EmergencyStop:
            return "EmergencyStop";
        case RuntimeState::Stopped:
            return "Stopped";
        case RuntimeState::Completed:
            return "Completed";
        case RuntimeState::Failed:
            return "Failed";
    }

    return "Unknown";
}

}  // namespace robot_process_platform::runtime
