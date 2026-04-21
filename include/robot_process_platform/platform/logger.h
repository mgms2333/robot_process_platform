#pragma once

#include <condition_variable>
#include <fstream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace robot_process_platform::platform
{

enum class LogLevel
{
    Debug,
    Info,
    Warning,
    Error
};

// Logger 提供项目级异步日志能力。
// 当前版本支持：
// 1. 控制台输出
// 2. 文件异步落盘
// 3. DEBUG / INFO / WARNING / ERROR 四个级别
//
// 为了避免不同模块各自持有多个日志实例，当前先采用单例方式统一使用。
class Logger
{
public:
    static Logger& GetInstance();

    void Initialize(const std::string& log_directory_path,
                    bool enable_console_output = true);

    void Shutdown();

    void LogD(const std::string& source, const std::string& message);
    void LogI(const std::string& source, const std::string& message);
    void LogW(const std::string& source, const std::string& message);
    void LogE(const std::string& source, const std::string& message);

private:
    Logger();
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void EnqueueLog(LogLevel level,
                    const std::string& source,
                    const std::string& message);

    void ProcessLogs();

    std::string BuildLogFilePath(const std::string& log_directory_path) const;
    std::string BuildLogLine(LogLevel level,
                             const std::string& source,
                             const std::string& message) const;
    std::string CurrentDateTime() const;
    std::string CurrentDateStamp() const;
    std::string LogLevelToString(LogLevel level) const;
    std::string LogLevelToColor(LogLevel level) const;
    std::string RemoveAnsiCodes(const std::string& text) const;

    std::ofstream log_file;
    std::queue<std::string> log_queue;
    std::mutex log_lock;
    std::condition_variable queue_condition;
    std::thread log_thread;
    bool stop_logging = false;
    bool initialized = false;
    bool console_output_enabled = true;
    std::string log_directory;
};

}  // namespace robot_process_platform::platform
