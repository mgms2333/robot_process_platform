#pragma once

// RuntimeService 设计说明。
//
// 本文件只描述运行时服务线程的职责边界，不参与编译实现。
//
// RuntimeService 不是简单的 Tick 线程。
// 它是运行时控制入口，负责把外部命令、安全事件、设备事件和 TaskRunner
// 的调度推进组织到同一条运行时控制链中。
//
// 推荐分层：
//
// HMI / API / MES / EventBus / SafetyManager / DeviceManager
//         |
//         v
// RuntimeService
//         |
//         | owns
//         v
// TaskRunner + ExecutionContext
//         |
//         v
// StateMachine / Scheduler / Queue / Executor
//
// RuntimeService 的核心职责：
//
// 1. 生命周期管理
//    - 启动运行时线程
//    - 停止运行时线程
//    - 维护 service_running 状态
//
// 2. 命令接收
//    - LoadTask
//    - StartTask
//    - PauseTask
//    - ResumeTask
//    - StopTask
//    - EmergencyStop
//    - ResetFault
//
// 3. 命令串行化
//    - 外部线程不直接调用 TaskRunner
//    - 外部线程只把命令写入 RuntimeService
//    - RuntimeService 线程按顺序处理命令
//    - 避免 HMI、API、安全回调同时修改 ExecutionContext
//
// 4. 任务装载
//    - 接收 core::Task 对象
//    - 或接收 task_id 后交给 TaskRepository 读取本地 task json
//    - task json 必须先反序列化为 core::Task
//    - 反序列化后的 core::Task 再交给 TaskRunner::LoadTask
//
// 5. 调度推进
//    - 在 Running 状态下周期性调用 TaskRunner::Tick
//    - 一次 Tick 最多推进一个 Action
//    - Tick 不是线程本身，只是 RuntimeService 线程中的一个步骤
//
// 6. 状态快照
//    - 对外提供 ExecutionContext 的只读快照
//    - HMI 查询状态时不直接读写 TaskRunner 内部对象
//
// 7. 安全与设备事件入口
//    - SafetyManager 发现急停时通知 RuntimeService
//    - DeviceManager 或设备适配器发现故障时通知 RuntimeService
//    - RuntimeService 再统一调用 TaskRunner::EmergencyStop 或 Stop
//
// 运行时线程每轮循环不只做 Tick，而是执行以下顺序：
//
// while (service_running)
// {
//     1. 取出并处理外部命令
//        - LoadTask
//        - StartTask
//        - PauseTask
//        - ResumeTask
//        - StopTask
//        - EmergencyStop
//        - ResetFault
//
//     2. 处理安全和设备事件
//        - emergency_stop_triggered
//        - robot_error
//        - camera_error
//        - io_error
//
//     3. 根据 ExecutionContext 判断是否推进任务
//        - Running: 调用 TaskRunner::Tick
//        - Paused: 不推进
//        - Fault: 不推进，等待 ResetFault 或 Stop
//        - EmergencyStop: 不推进，等待 ResetFault 或人工处理
//        - Completed: 不推进，等待新任务
//
//     4. 更新状态快照
//
//     5. 发布运行时事实事件
//        - task.started
//        - task.paused
//        - task.resumed
//        - task.stopped
//        - task.completed
//        - task.failed
//        - task.block.completed
//        - task.action.completed
//
//     6. 等待下一个调度周期或被新命令唤醒
// }
//
// 伪代码结构：
//
// enum class RuntimeCommandType
// {
//     LoadTask,
//     StartTask,
//     PauseTask,
//     ResumeTask,
//     StopTask,
//     EmergencyStop,
//     ResetFault
// };
//
// struct RuntimeCommand
// {
//     RuntimeCommandType type;
//     core::Task task;
//     int start_block_index;
//     int start_action_index;
// };
//
// class RuntimeService
// {
// public:
//     RuntimeService(device::IRobot& robot_instance);
//
//     void StartService();
//     void StopService();
//
//     void SubmitCommand(const RuntimeCommand& command);
//     ExecutionContext GetContextSnapshot() const;
//
// private:
//     void RuntimeLoop();
//     void ProcessPendingCommands();
//     void ProcessRuntimeCommand(const RuntimeCommand& command);
//     void ProcessSafetyAndDeviceEvents();
//     void TickIfAllowed();
//     void PublishRuntimeEvents();
//
//     TaskRunner task_runner;
//     ExecutionContext execution_context;
//     std::queue<RuntimeCommand> command_queue;
//     std::thread runtime_thread;
//     bool service_running = false;
// };
//
// Task 运行方式：
//
// 1. 创建任务时：
//    - TaskManager 根据模板和输入 json 创建 core::Task
//    - TaskManager 保存 task json
//    - RuntimeService 接收 LoadTask 命令
//
// 2. 执行已保存任务时：
//    - TaskRepository 根据 task_id 读取 task json
//    - TaskRepository 反序列化为 core::Task
//    - RuntimeService 接收 LoadTask 命令
//
// 3. 启动任务时：
//    - RuntimeService 接收 StartTask 命令
//    - RuntimeService 线程内调用 TaskRunner::Start
//
// 4. 暂停、恢复、停止、急停时：
//    - 外部系统提交命令
//    - RuntimeService 线程内串行调用 TaskRunner 对应接口
//
// 关键原则：
//
// - RuntimeService 是线程宿主。
// - TaskRunner 是调度核心。
// - ExecutionContext 只能在 RuntimeService 线程内被修改。
// - 外部系统通过命令或事件影响 RuntimeService。
// - 外部系统不直接调用 Tick。
// - 外部系统不直接修改 ExecutionContext。

