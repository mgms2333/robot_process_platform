#include "robot_process_platform/platform/logger.h"

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace robot_process_platform::platform
{

Logger& Logger::GetInstance()
{
    static Logger logger_instance;
    return logger_instance;
}

Logger::Logger() = default;

Logger::~Logger()
{
    Shutdown();
}

void Logger::Initialize(const std::string& log_directory_path,
                        bool enable_console_output)
{
    std::lock_guard<std::mutex> lock(log_lock);

    if (initialized)
    {
        return;
    }

    log_directory = log_directory_path;
    console_output_enabled = enable_console_output;
    stop_logging = false;

    std::filesystem::create_directories(log_directory);
    log_file.open(BuildLogFilePath(log_directory), std::ios::out | std::ios::app);

    initialized = true;
    log_thread = std::thread(&Logger::ProcessLogs, this);
}

void Logger::Shutdown()
{
    {
        std::lock_guard<std::mutex> lock(log_lock);
        if (!initialized)
        {
            return;
        }

        stop_logging = true;
    }

    queue_condition.notify_all();

    if (log_thread.joinable())
    {
        log_thread.join();
    }

    {
        std::lock_guard<std::mutex> lock(log_lock);
        if (log_file.is_open())
        {
            log_file.flush();
            log_file.close();
        }

        initialized = false;
    }
}

void Logger::LogD(const std::string& source, const std::string& message)
{
    EnqueueLog(LogLevel::Debug, source, message);
}

void Logger::LogI(const std::string& source, const std::string& message)
{
    EnqueueLog(LogLevel::Info, source, message);
}

void Logger::LogW(const std::string& source, const std::string& message)
{
    EnqueueLog(LogLevel::Warning, source, message);
}

void Logger::LogE(const std::string& source, const std::string& message)
{
    EnqueueLog(LogLevel::Error, source, message);
}

void Logger::EnqueueLog(LogLevel level,
                        const std::string& source,
                        const std::string& message)
{
    std::lock_guard<std::mutex> lock(log_lock);

    if (!initialized)
    {
        return;
    }

    log_queue.push(BuildLogLine(level, source, message));
    queue_condition.notify_one();
}

void Logger::ProcessLogs()
{
    while (true)
    {
        std::string log_line;

        {
            std::unique_lock<std::mutex> lock(log_lock);
            queue_condition.wait(
                lock,
                [this]()
                {
                    return stop_logging || !log_queue.empty();
                });

            if (stop_logging && log_queue.empty())
            {
                break;
            }

            log_line = log_queue.front();
            log_queue.pop();
        }

        if (console_output_enabled)
        {
            std::cout << log_line << '\n';
        }

        if (log_file.is_open())
        {
            log_file << RemoveAnsiCodes(log_line) << '\n';
            log_file.flush();
        }
    }
}

std::string Logger::BuildLogFilePath(const std::string& log_directory_path) const
{
    return log_directory_path + "/robot_process_platform_" + CurrentDateStamp() + ".log";
}

std::string Logger::BuildLogLine(LogLevel level,
                                 const std::string& source,
                                 const std::string& message) const
{
    std::ostringstream output_stream;
    const std::string level_color = LogLevelToColor(level);

    output_stream << level_color
                  << "[" << CurrentDateTime() << "] "
                  << "[" << LogLevelToString(level) << "] "
                  << "[" << source << "] "
                  << message
                  << "\033[0m";
    return output_stream.str();
}

std::string Logger::CurrentDateTime() const
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::tm local_time {};
    localtime_r(&now_time, &local_time);

    std::ostringstream output_stream;
    output_stream << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S")
                  << "."
                  << std::setw(3)
                  << std::setfill('0')
                  << milliseconds.count();
    return output_stream.str();
}

std::string Logger::CurrentDateStamp() const
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::tm local_time {};
    localtime_r(&now_time, &local_time);

    std::ostringstream output_stream;
    output_stream << std::put_time(&local_time, "%Y%m%d");
    return output_stream.str();
}

std::string Logger::LogLevelToString(LogLevel level) const
{
    switch (level)
    {
        case LogLevel::Debug:
            return "DEBUG";
        case LogLevel::Info:
            return "INFO";
        case LogLevel::Warning:
            return "WARNING";
        case LogLevel::Error:
            return "ERROR";
    }

    return "UNKNOWN";
}

std::string Logger::LogLevelToColor(LogLevel level) const
{
    switch (level)
    {
        case LogLevel::Debug:
            return "\033[36m";
        case LogLevel::Info:
            return "\033[32m";
        case LogLevel::Warning:
            return "\033[33m";
        case LogLevel::Error:
            return "\033[31m";
    }

    return "";
}

std::string Logger::RemoveAnsiCodes(const std::string& text) const
{
    std::string output_text;

    for (std::size_t index = 0; index < text.size(); ++index)
    {
        if (text[index] == '\033')
        {
            while (index < text.size() && text[index] != 'm')
            {
                ++index;
            }

            continue;
        }

        output_text.push_back(text[index]);
    }

    return output_text;
}

}  // namespace robot_process_platform::platform
