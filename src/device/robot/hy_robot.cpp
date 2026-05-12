#include "robot_process_platform/device/robot/hy_robot.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <optional>
#include <sstream>
#include <thread>

#include "robot_process_platform/platform/logger.h"

#ifdef ROBOT_PROCESS_PLATFORM_USE_HYROBOT_SDK
#include "HR_Pro.h"
#endif

namespace robot_process_platform::device
{

namespace
{

std::string Trim(const std::string& text)
{
    std::size_t begin_index = 0;
    while (begin_index < text.size() &&
           std::isspace(static_cast<unsigned char>(text[begin_index])))
    {
        ++begin_index;
    }

    std::size_t end_index = text.size();
    while (end_index > begin_index &&
           std::isspace(static_cast<unsigned char>(text[end_index - 1])))
    {
        --end_index;
    }

    return text.substr(begin_index, end_index - begin_index);
}

std::string NormalizeNumericListText(const std::string& text)
{
    std::string normalized_text = text;
    std::replace(normalized_text.begin(), normalized_text.end(), ',', ' ');
    std::replace(normalized_text.begin(), normalized_text.end(), ';', ' ');
    std::replace(normalized_text.begin(), normalized_text.end(), '|', ' ');
    return normalized_text;
}

std::optional<std::array<double, 6>> ParsePoseText(const std::string& pose_text)
{
    const std::string normalized_text = NormalizeNumericListText(pose_text);
    std::istringstream input_stream(normalized_text);

    std::array<double, 6> pose_values{};
    for (double& pose_value : pose_values)
    {
        if (!(input_stream >> pose_value))
        {
            return std::nullopt;
        }
    }

    return pose_values;
}

std::optional<std::array<double, 6>> ParseJointText(const std::string& joints_text)
{
    return ParsePoseText(joints_text);
}

std::optional<bool> ParseBoolText(const std::string& bool_text)
{
    const std::string normalized_text = Trim(bool_text);

    if (normalized_text == "1" || normalized_text == "true" ||
        normalized_text == "True" || normalized_text == "TRUE")
    {
        return true;
    }

    if (normalized_text == "0" || normalized_text == "false" ||
        normalized_text == "False" || normalized_text == "FALSE")
    {
        return false;
    }

    return std::nullopt;
}

std::optional<int> ParseIntText(const std::string& int_text)
{
    try
    {
        return std::stoi(Trim(int_text));
    }
    catch (...)
    {
        return std::nullopt;
    }
}

std::optional<double> ParseDoubleText(const std::string& double_text)
{
    try
    {
        return std::stod(Trim(double_text));
    }
    catch (...)
    {
        return std::nullopt;
    }
}

std::string GetStringParameter(const std::map<std::string, std::string>& action_parameters,
                               const std::string& parameter_name,
                               const std::string& default_value)
{
    const auto parameter_iterator = action_parameters.find(parameter_name);
    if (parameter_iterator == action_parameters.end())
    {
        return default_value;
    }

    return parameter_iterator->second;
}

std::optional<int> GetSignalBit(const std::map<std::string, std::string>& action_parameters,
                                const std::string& signal_name,
                                const std::map<std::string, int>& configured_bits)
{
    const auto bit_iterator = action_parameters.find("signal_bit");
    if (bit_iterator != action_parameters.end())
    {
        return ParseIntText(bit_iterator->second);
    }

    const auto configured_iterator = configured_bits.find(signal_name);
    if (configured_iterator == configured_bits.end())
    {
        return std::nullopt;
    }

    return configured_iterator->second;
}

}  // namespace

HyRobot::HyRobot()
    : HyRobot(HyRobotConfig())
{
}

HyRobot::HyRobot(const HyRobotConfig& config_value)
    : config(config_value)
{
}

HyRobot::~HyRobot()
{
    Disconnect();
}

RobotCommandResult HyRobot::MoveJoint(
    const std::map<std::string, std::string>& action_parameters)
{
    execution_log.push_back(BuildLogEntry("MoveJoint", action_parameters, 0));

    if (!config.enable_sdk)
    {
        return ExecuteMockCommand("MoveJoint", action_parameters, 0);
    }

#ifndef ROBOT_PROCESS_PLATFORM_USE_HYROBOT_SDK
    platform::Logger::GetInstance().LogW("HyRobot", "MoveJoint requested but hy SDK is not enabled in build.");
    return RobotCommandResult(false, "hy_sdk_not_enabled_in_build");
#else
    if (!sdk_connected)
    {
        platform::Logger::GetInstance().LogE("HyRobot", "MoveJoint rejected: hy robot is not connected.");
        return RobotCommandResult(false, "hy_robot_not_connected");
    }

    const auto target_pose_iterator = action_parameters.find("target_pose");
    if (target_pose_iterator == action_parameters.end())
    {
        return RobotCommandResult(false, "missing_target_pose");
    }

    const std::optional<std::array<double, 6>> target_pose =
        ParsePoseText(target_pose_iterator->second);
    if (!target_pose.has_value())
    {
        return RobotCommandResult(false, "invalid_target_pose");
    }

    JointsData joints{};
    int is_use_joint = 0;

    const auto reference_joints_iterator = action_parameters.find("reference_joints");
    if (reference_joints_iterator != action_parameters.end())
    {
        const std::optional<std::array<double, 6>> reference_joints =
            ParseJointText(reference_joints_iterator->second);
        if (!reference_joints.has_value())
        {
            return RobotCommandResult(false, "invalid_reference_joints");
        }

        for (std::size_t joint_index = 0; joint_index < 6; ++joint_index)
        {
            joints.dJoint[joint_index] = reference_joints->at(joint_index);
        }
    }

    const auto use_joint_iterator = action_parameters.find("use_joint");
    if (use_joint_iterator != action_parameters.end())
    {
        const std::optional<bool> use_joint_value = ParseBoolText(use_joint_iterator->second);
        if (!use_joint_value.has_value())
        {
            return RobotCommandResult(false, "invalid_use_joint");
        }

        is_use_joint = *use_joint_value ? 1 : 0;
    }

    const double velocity = ParseDoubleText(
        GetStringParameter(action_parameters, "velocity", std::to_string(config.default_velocity)))
                                .value_or(config.default_velocity);
    const double acceleration = ParseDoubleText(
        GetStringParameter(action_parameters,
                           "acceleration",
                           std::to_string(config.default_acceleration)))
                                    .value_or(config.default_acceleration);
    const double radius = ParseDoubleText(
        GetStringParameter(action_parameters, "radius", std::to_string(config.default_radius)))
                              .value_or(config.default_radius);
    const std::string tcp_name = GetStringParameter(action_parameters, "tcp_name", config.tcp_name);
    const std::string ucs_name = GetStringParameter(action_parameters, "ucs_name", config.ucs_name);
    const std::string command_id = GetStringParameter(action_parameters, "command_id", "runtime_movej");

    const int sdk_result = HRIF_MoveJ_nJ(config.box_id,
                                         config.robot_id,
                                         target_pose->at(0),
                                         target_pose->at(1),
                                         target_pose->at(2),
                                         target_pose->at(3),
                                         target_pose->at(4),
                                         target_pose->at(5),
                                         joints,
                                         tcp_name,
                                         ucs_name,
                                         velocity,
                                         acceleration,
                                         radius,
                                         is_use_joint,
                                         0,
                                         0,
                                         0,
                                         command_id);
    if (sdk_result != 0)
    {
        platform::Logger::GetInstance().LogE("HyRobot", "HRIF_MoveJ_nJ failed: error_code=" + std::to_string(sdk_result));
        return RobotCommandResult(false, "HRIF_MoveJ_nJ_failed:" + std::to_string(sdk_result));
    }

    const auto wait_begin_time = std::chrono::steady_clock::now();
    while (true)
    {
        bool motion_done = false;
        const int wait_result = HRIF_IsMotionDone(config.box_id, config.robot_id, motion_done);
        if (wait_result != 0)
        {
            return RobotCommandResult(false, "HRIF_IsMotionDone_failed:" + std::to_string(wait_result));
        }

        if (motion_done)
        {
            platform::Logger::GetInstance().LogD("HyRobot", "MoveJoint executed by hy SDK.");
            return RobotCommandResult(true, "MoveJoint executed by hy SDK.");
        }

        const auto elapsed_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - wait_begin_time)
                .count();
        if (elapsed_time_ms >= config.motion_done_timeout_ms)
        {
            return RobotCommandResult(false, "move_joint_timeout");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config.io_poll_interval_ms));
    }
#endif
}

RobotCommandResult HyRobot::MoveLinear(
    const std::map<std::string, std::string>& action_parameters)
{
    execution_log.push_back(BuildLogEntry("MoveLinear", action_parameters, 0));

    if (!config.enable_sdk)
    {
        return ExecuteMockCommand("MoveLinear", action_parameters, 0);
    }

#ifndef ROBOT_PROCESS_PLATFORM_USE_HYROBOT_SDK
    platform::Logger::GetInstance().LogW("HyRobot", "MoveLinear requested but hy SDK is not enabled in build.");
    return RobotCommandResult(false, "hy_sdk_not_enabled_in_build");
#else
    if (!sdk_connected)
    {
        platform::Logger::GetInstance().LogE("HyRobot", "MoveLinear rejected: hy robot is not connected.");
        return RobotCommandResult(false, "hy_robot_not_connected");
    }

    const auto target_pose_iterator = action_parameters.find("target_pose");
    if (target_pose_iterator == action_parameters.end())
    {
        return RobotCommandResult(false, "missing_target_pose");
    }

    const std::optional<std::array<double, 6>> target_pose =
        ParsePoseText(target_pose_iterator->second);
    if (!target_pose.has_value())
    {
        return RobotCommandResult(false, "invalid_target_pose");
    }

    JointsData joints{};

    const auto reference_joints_iterator = action_parameters.find("reference_joints");
    if (reference_joints_iterator != action_parameters.end())
    {
        const std::optional<std::array<double, 6>> reference_joints =
            ParseJointText(reference_joints_iterator->second);
        if (!reference_joints.has_value())
        {
            return RobotCommandResult(false, "invalid_reference_joints");
        }

        for (std::size_t joint_index = 0; joint_index < 6; ++joint_index)
        {
            joints.dJoint[joint_index] = reference_joints->at(joint_index);
        }
    }

    const double velocity = ParseDoubleText(
        GetStringParameter(action_parameters, "velocity", std::to_string(config.default_velocity)))
                                .value_or(config.default_velocity);
    const double acceleration = ParseDoubleText(
        GetStringParameter(action_parameters,
                           "acceleration",
                           std::to_string(config.default_acceleration)))
                                    .value_or(config.default_acceleration);
    const double radius = ParseDoubleText(
        GetStringParameter(action_parameters, "radius", std::to_string(config.default_radius)))
                              .value_or(config.default_radius);
    const std::string tcp_name = GetStringParameter(action_parameters, "tcp_name", config.tcp_name);
    const std::string ucs_name = GetStringParameter(action_parameters, "ucs_name", config.ucs_name);
    const std::string command_id = GetStringParameter(action_parameters, "command_id", "runtime_movel");

    const int sdk_result = HRIF_MoveL_nJ(config.box_id,
                                         config.robot_id,
                                         target_pose->at(0),
                                         target_pose->at(1),
                                         target_pose->at(2),
                                         target_pose->at(3),
                                         target_pose->at(4),
                                         target_pose->at(5),
                                         joints,
                                         tcp_name,
                                         ucs_name,
                                         velocity,
                                         acceleration,
                                         radius,
                                         0,
                                         0,
                                         0,
                                         command_id);
    if (sdk_result != 0)
    {
        platform::Logger::GetInstance().LogE("HyRobot", "HRIF_MoveL_nJ failed: error_code=" + std::to_string(sdk_result));
        return RobotCommandResult(false, "HRIF_MoveL_nJ_failed:" + std::to_string(sdk_result));
    }

    const auto wait_begin_time = std::chrono::steady_clock::now();
    while (true)
    {
        bool motion_done = false;
        const int wait_result = HRIF_IsMotionDone(config.box_id, config.robot_id, motion_done);
        if (wait_result != 0)
        {
            return RobotCommandResult(false, "HRIF_IsMotionDone_failed:" + std::to_string(wait_result));
        }

        if (motion_done)
        {
            platform::Logger::GetInstance().LogD("HyRobot", "MoveLinear executed by hy SDK.");
            return RobotCommandResult(true, "MoveLinear executed by hy SDK.");
        }

        const auto elapsed_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - wait_begin_time)
                .count();
        if (elapsed_time_ms >= config.motion_done_timeout_ms)
        {
            return RobotCommandResult(false, "move_linear_timeout");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config.io_poll_interval_ms));
    }
#endif
}

RobotCommandResult HyRobot::SetDigitalOutput(
    const std::map<std::string, std::string>& action_parameters)
{
    execution_log.push_back(BuildLogEntry("SetDigitalOutput", action_parameters, 0));

    if (!config.enable_sdk)
    {
        return ExecuteMockCommand("SetDigitalOutput", action_parameters, 0);
    }

#ifndef ROBOT_PROCESS_PLATFORM_USE_HYROBOT_SDK
    platform::Logger::GetInstance().LogW("HyRobot", "SetDigitalOutput requested but hy SDK is not enabled in build.");
    return RobotCommandResult(false, "hy_sdk_not_enabled_in_build");
#else
    if (!sdk_connected)
    {
        platform::Logger::GetInstance().LogE("HyRobot", "SetDigitalOutput rejected: hy robot is not connected.");
        return RobotCommandResult(false, "hy_robot_not_connected");
    }

    const auto signal_name_iterator = action_parameters.find("signal_name");
    if (signal_name_iterator == action_parameters.end())
    {
        return RobotCommandResult(false, "missing_signal_name");
    }

    const std::optional<int> signal_bit =
        GetSignalBit(action_parameters, signal_name_iterator->second, config.box_do_bits);
    if (!signal_bit.has_value())
    {
        return RobotCommandResult(false, "unknown_signal_bit");
    }

    const auto signal_value_iterator = action_parameters.find("signal_value");
    if (signal_value_iterator == action_parameters.end())
    {
        return RobotCommandResult(false, "missing_signal_value");
    }

    const std::optional<bool> signal_value = ParseBoolText(signal_value_iterator->second);
    if (!signal_value.has_value())
    {
        return RobotCommandResult(false, "invalid_signal_value");
    }

    const int sdk_result =
        HRIF_SetBoxDO(config.box_id, *signal_bit, *signal_value ? 1 : 0);
    if (sdk_result != 0)
    {
        platform::Logger::GetInstance().LogE("HyRobot", "HRIF_SetBoxDO failed: error_code=" + std::to_string(sdk_result));
        return RobotCommandResult(false, "HRIF_SetBoxDO_failed:" + std::to_string(sdk_result));
    }

    platform::Logger::GetInstance().LogD("HyRobot", "SetDigitalOutput executed by hy SDK.");
    return RobotCommandResult(true, "SetDigitalOutput executed by hy SDK.");
#endif
}

RobotCommandResult HyRobot::WaitDigitalInput(
    const std::map<std::string, std::string>& action_parameters,
    int timeout_ms)
{
    execution_log.push_back(BuildLogEntry("WaitDigitalInput", action_parameters, timeout_ms));

    if (!config.enable_sdk)
    {
        return ExecuteMockCommand("WaitDigitalInput", action_parameters, timeout_ms);
    }

#ifndef ROBOT_PROCESS_PLATFORM_USE_HYROBOT_SDK
    platform::Logger::GetInstance().LogW("HyRobot", "WaitDigitalInput requested but hy SDK is not enabled in build.");
    return RobotCommandResult(false, "hy_sdk_not_enabled_in_build");
#else
    if (!sdk_connected)
    {
        platform::Logger::GetInstance().LogE("HyRobot", "WaitDigitalInput rejected: hy robot is not connected.");
        return RobotCommandResult(false, "hy_robot_not_connected");
    }

    const auto signal_name_iterator = action_parameters.find("signal_name");
    if (signal_name_iterator == action_parameters.end())
    {
        return RobotCommandResult(false, "missing_signal_name");
    }

    const std::optional<int> signal_bit =
        GetSignalBit(action_parameters, signal_name_iterator->second, config.box_di_bits);
    if (!signal_bit.has_value())
    {
        return RobotCommandResult(false, "unknown_signal_bit");
    }

    const auto expected_value_iterator = action_parameters.find("expected_value");
    if (expected_value_iterator == action_parameters.end())
    {
        return RobotCommandResult(false, "missing_expected_value");
    }

    const std::optional<bool> expected_value =
        ParseBoolText(expected_value_iterator->second);
    if (!expected_value.has_value())
    {
        return RobotCommandResult(false, "invalid_expected_value");
    }

    const auto wait_begin_time = std::chrono::steady_clock::now();
    while (true)
    {
        int current_value = 0;
        const int sdk_result = HRIF_ReadBoxDI(config.box_id, *signal_bit, current_value);
        if (sdk_result != 0)
        {
            return RobotCommandResult(false, "HRIF_ReadBoxDI_failed:" + std::to_string(sdk_result));
        }

        if ((current_value != 0) == *expected_value)
        {
            platform::Logger::GetInstance().LogD("HyRobot", "WaitDigitalInput matched by hy SDK.");
            return RobotCommandResult(true, "WaitDigitalInput matched by hy SDK.");
        }

        const auto elapsed_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - wait_begin_time)
                .count();
        if (elapsed_time_ms >= timeout_ms)
        {
            return RobotCommandResult(false, "wait_digital_input_timeout");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config.io_poll_interval_ms));
    }
#endif
}

RobotCommandResult HyRobot::Delay(
    const std::map<std::string, std::string>& action_parameters,
    int timeout_ms)
{
    execution_log.push_back(BuildLogEntry("Delay", action_parameters, timeout_ms));
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
    platform::Logger::GetInstance().LogD("HyRobot", "Delay executed.");
    return RobotCommandResult(true, "Delay executed.");
}

bool HyRobot::Connect()
{
    if (!config.enable_sdk)
    {
        platform::Logger::GetInstance().LogW("HyRobot", "Connect requested but hy SDK mode is disabled.");
        return false;
    }

#ifndef ROBOT_PROCESS_PLATFORM_USE_HYROBOT_SDK
    platform::Logger::GetInstance().LogW("HyRobot", "Connect requested but hy SDK is not enabled in build.");
    return false;
#else
    if (sdk_connected)
    {
        return true;
    }

    const int connect_result = HRIF_Connect(config.box_id, config.host_name.c_str(), config.port);
    if (connect_result != 0)
    {
        platform::Logger::GetInstance().LogE("HyRobot", "HRIF_Connect failed: error_code=" + std::to_string(connect_result));
        return false;
    }

    const int connect_box_result = HRIF_Connect2Box(config.box_id);
    if (connect_box_result != 0)
    {
        platform::Logger::GetInstance().LogE("HyRobot", "HRIF_Connect2Box failed: error_code=" + std::to_string(connect_box_result));
        return false;
    }

    if (config.auto_electrify)
    {
        const int electrify_result = HRIF_Electrify(config.box_id);
        if (electrify_result != 0)
        {
            platform::Logger::GetInstance().LogE("HyRobot", "HRIF_Electrify failed: error_code=" + std::to_string(electrify_result));
            return false;
        }
    }

    if (config.auto_connect_controller)
    {
        const int controller_result = HRIF_Connect2Controller(config.box_id);
        if (controller_result != 0)
        {
            platform::Logger::GetInstance().LogE("HyRobot", "HRIF_Connect2Controller failed: error_code=" + std::to_string(controller_result));
            return false;
        }
    }

    sdk_connected = HRIF_IsConnected(config.box_id);
    if (sdk_connected)
    {
        platform::Logger::GetInstance().LogI("HyRobot", "Hy robot SDK connected successfully.");
    }

    return sdk_connected;
#endif
}

void HyRobot::Disconnect()
{
    if (!sdk_connected)
    {
        return;
    }

#ifdef ROBOT_PROCESS_PLATFORM_USE_HYROBOT_SDK
    const int disconnect_result = HRIF_DisConnect(config.box_id);
    if (disconnect_result != 0)
    {
        platform::Logger::GetInstance().LogW("HyRobot", "HRIF_DisConnect returned error_code=" + std::to_string(disconnect_result));
    }
#endif

    sdk_connected = false;
    platform::Logger::GetInstance().LogI("HyRobot", "Hy robot disconnected.");
}

bool HyRobot::IsConnected() const
{
    return sdk_connected;
}

const std::vector<std::string>& HyRobot::GetExecutionLog() const
{
    return execution_log;
}

const HyRobotConfig& HyRobot::GetConfig() const
{
    return config;
}

RobotCommandResult HyRobot::ExecuteMockCommand(
    const std::string& command_name,
    const std::map<std::string, std::string>& action_parameters,
    int timeout_ms)
{
    if (command_name == "Delay" && timeout_ms > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
    }

    platform::Logger::GetInstance().LogD("HyRobot", command_name + " executed in mock mode.");
    return RobotCommandResult(true, command_name + " executed in mock mode.");
}

std::string HyRobot::BuildLogEntry(
    const std::string& command_name,
    const std::map<std::string, std::string>& action_parameters,
    int timeout_ms) const
{
    std::ostringstream output_stream;
    output_stream << command_name;

    if (timeout_ms > 0)
    {
        output_stream << "(timeout_ms=" << timeout_ms << ")";
    }

    if (!action_parameters.empty())
    {
        output_stream << " ";
        std::size_t parameter_index = 0;
        for (const auto& parameter_entry : action_parameters)
        {
            if (parameter_index > 0)
            {
                output_stream << ", ";
            }

            output_stream << parameter_entry.first << "=" << parameter_entry.second;
            ++parameter_index;
        }
    }

    return output_stream.str();
}

}  // namespace robot_process_platform::device
