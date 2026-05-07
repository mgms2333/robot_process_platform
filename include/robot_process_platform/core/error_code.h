#pragma once

#include <optional>
#include <string>

namespace robot_process_platform::core
{

enum class ErrorCode
{
    Ok = 0,

    InvalidArgument,
    InternalError,

    TemplateNotFound,
    TemplateCreateFailed,

    TaskCreateFailed,
    TaskDeserializeFailed,
    TaskSaveFailed,
    TaskDeleteFailed,
    TaskFileNotFound
};

inline std::string ToString(ErrorCode error_code)
{
    switch (error_code)
    {
        case ErrorCode::Ok:
            return "Ok";
        case ErrorCode::InvalidArgument:
            return "InvalidArgument";
        case ErrorCode::InternalError:
            return "InternalError";
        case ErrorCode::TemplateNotFound:
            return "TemplateNotFound";
        case ErrorCode::TemplateCreateFailed:
            return "TemplateCreateFailed";
        case ErrorCode::TaskCreateFailed:
            return "TaskCreateFailed";
        case ErrorCode::TaskDeserializeFailed:
            return "TaskDeserializeFailed";
        case ErrorCode::TaskSaveFailed:
            return "TaskSaveFailed";
        case ErrorCode::TaskDeleteFailed:
            return "TaskDeleteFailed";
        case ErrorCode::TaskFileNotFound:
            return "TaskFileNotFound";
    }

    return "Unknown";
}

struct Status
{
    ErrorCode code = ErrorCode::Ok;
    std::string message;

    bool Ok() const
    {
        return code == ErrorCode::Ok;
    }
};

template <typename T>
struct Result
{
    ErrorCode code = ErrorCode::Ok;
    std::string message;
    std::optional<T> value;

    bool Ok() const
    {
        return code == ErrorCode::Ok;
    }
};

template <typename T>
inline Result<T> MakeErrorResult(ErrorCode error_code, const std::string& message)
{
    Result<T> result;
    result.code = error_code;
    result.message = message;
    return result;
}

template <typename T>
inline Result<T> MakeSuccessResult(T value)
{
    Result<T> result;
    result.value = std::move(value);
    return result;
}

inline Status MakeErrorStatus(ErrorCode error_code, const std::string& message)
{
    Status status;
    status.code = error_code;
    status.message = message;
    return status;
}

inline Status MakeSuccessStatus()
{
    return Status();
}

}  // namespace robot_process_platform::core
