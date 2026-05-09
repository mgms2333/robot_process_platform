#include "robot_process_platform/runtime/state_machine.h"

namespace robot_process_platform::runtime
{

StateMachine::StateMachine() = default;

bool StateMachine::HandleEvent(StateEvent event, ExecutionContext& execution_context)
{
    switch (execution_context.runtime_state)
    {
        case RuntimeState::Idle:
            if (event == StateEvent::TaskLoaded)
            {
                execution_context.runtime_state = RuntimeState::Ready;
                return true;
            }
            break;
        case RuntimeState::Ready:
            if (event == StateEvent::StopRequested)
            {
                execution_context.runtime_state = RuntimeState::Stopped;
                return true;
            }
            if (event == StateEvent::StartRequested)
            {
                execution_context.runtime_state = RuntimeState::Running;
                return true;
            }
            if (event == StateEvent::EmergencyStopTriggered)
            {
                execution_context.runtime_state = RuntimeState::EmergencyStop;
                return true;
            }
            break;
        case RuntimeState::Running:
            if (event == StateEvent::StopRequested)
            {
                execution_context.runtime_state = RuntimeState::Stopped;
                return true;
            }
            if (event == StateEvent::PauseRequested)
            {
                execution_context.runtime_state = RuntimeState::Paused;
                return true;
            }
            if (event == StateEvent::BlockFailed)
            {
                execution_context.runtime_state = RuntimeState::Fault;
                return true;
            }
            if (event == StateEvent::TaskCompleted)
            {
                execution_context.runtime_state = RuntimeState::Completed;
                return true;
            }
            if (event == StateEvent::EmergencyStopTriggered)
            {
                execution_context.runtime_state = RuntimeState::EmergencyStop;
                return true;
            }
            if (event == StateEvent::BlockCompleted)
            {
                return true;
            }
            break;
        case RuntimeState::Paused:
            if (event == StateEvent::StopRequested)
            {
                execution_context.runtime_state = RuntimeState::Stopped;
                return true;
            }
            if (event == StateEvent::ResumeRequested)
            {
                execution_context.runtime_state = RuntimeState::Running;
                return true;
            }
            if (event == StateEvent::EmergencyStopTriggered)
            {
                execution_context.runtime_state = RuntimeState::EmergencyStop;
                return true;
            }
            break;
        case RuntimeState::Fault:
            if (event == StateEvent::StopRequested)
            {
                execution_context.runtime_state = RuntimeState::Stopped;
                return true;
            }
            if (event == StateEvent::FaultResetRequested)
            {
                execution_context.runtime_state = RuntimeState::Ready;
                execution_context.last_error_code = core::ErrorCode::Ok;
                execution_context.last_error.clear();
                return true;
            }
            if (event == StateEvent::EmergencyStopTriggered)
            {
                execution_context.runtime_state = RuntimeState::EmergencyStop;
                return true;
            }
            break;
        case RuntimeState::EmergencyStop:
            if (event == StateEvent::FaultResetRequested)
            {
                execution_context.runtime_state = RuntimeState::Ready;
                execution_context.last_error_code = core::ErrorCode::Ok;
                execution_context.last_error.clear();
                return true;
            }
            break;
        case RuntimeState::Stopped:
        case RuntimeState::Completed:
        case RuntimeState::Failed:
            break;
    }

    return false;
}

bool StateMachine::CanPushBlockToQueue(const ExecutionContext& execution_context) const
{
    return execution_context.runtime_state == RuntimeState::Running;
}

RuntimeState StateMachine::CurrentState(const ExecutionContext& execution_context) const
{
    return execution_context.runtime_state;
}

std::string ToString(StateEvent state_event)
{
    switch (state_event)
    {
        case StateEvent::TaskLoaded:
            return "TaskLoaded";
        case StateEvent::StartRequested:
            return "StartRequested";
        case StateEvent::PauseRequested:
            return "PauseRequested";
        case StateEvent::ResumeRequested:
            return "ResumeRequested";
        case StateEvent::StopRequested:
            return "StopRequested";
        case StateEvent::BlockCompleted:
            return "BlockCompleted";
        case StateEvent::BlockFailed:
            return "BlockFailed";
        case StateEvent::TaskCompleted:
            return "TaskCompleted";
        case StateEvent::EmergencyStopTriggered:
            return "EmergencyStopTriggered";
        case StateEvent::FaultResetRequested:
            return "FaultResetRequested";
    }

    return "Unknown";
}

}  // namespace robot_process_platform::runtime
