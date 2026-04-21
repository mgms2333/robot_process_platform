#include "robot_process_platform/runtime/execution_context.h"

namespace robot_process_platform::runtime
{

ExecutionContext::ExecutionContext() = default;

std::string ToString(ExecutionStatus execution_status)
{
    switch (execution_status)
    {
        case ExecutionStatus::Idle:
            return "Idle";
        case ExecutionStatus::Running:
            return "Running";
        case ExecutionStatus::Completed:
            return "Completed";
        case ExecutionStatus::Failed:
            return "Failed";
    }

    return "Unknown";
}

}  // namespace robot_process_platform::runtime
