#pragma once

#include <map>
#include <string>

namespace robot_process_platform::device
{

// CameraCommandResult 表示一次视觉调用的最小返回结果。
// 当前阶段先保留 success / message / result_parameters 三部分，
// 后续接真实厂家 SDK 时再补强。
struct CameraCommandResult
{
    CameraCommandResult(bool success_value,
                        const std::string& message_value,
                        const std::map<std::string, std::string>& result_parameters_value);

    bool success = false;
    std::string message;
    std::map<std::string, std::string> result_parameters;
};

// ICamera 定义当前平台最小视觉抽象。
// 当前阶段先只约束一个代表性能力：AcquirePose。
class ICamera
{
public:
    virtual ~ICamera() = default;

    virtual CameraCommandResult AcquirePose(
        const std::map<std::string, std::string>& action_parameters) = 0;
};

}  // namespace robot_process_platform::device
