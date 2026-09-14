#pragma once

#include <cstdlib>
#include <fstream>
#include <mutex>
#include <source_location>
#include <string_view>

namespace mss {

// 断言失败：把消息与调用位置写入 mss_error.log 后退出（exit 1）。
// 用函数而非宏，才能经 source_location 拿到调用处信息（C++20）。
inline void assert_(bool condition, std::string_view message,
                    const std::source_location location =
                        std::source_location::current()) {
    // 条件不满足时记录调用位置并终止进程；算法层用它暴露破坏调用契约的路径，
    // 不返回默认值继续计算，以免把无效状态传给后续概率/搜索层。
    if (condition) return;

    static std::mutex logMutex;
    std::lock_guard lock(logMutex);
    std::ofstream log("mss_error.log", std::ios::app);
    if (log) {
        log << "[assert]\n"
            << "message: " << message << '\n'
            << "file: " << location.file_name() << '\n'
            << "line: " << location.line() << '\n'
            << "function: " << location.function_name() << "\n\n";
        log.flush();
        log.close();
    }

    std::exit(1);
}

// 警告：非致命，记录到 mss_error.log，继续执行。
inline void warn_(std::string_view message,
                  const std::source_location location =
                      std::source_location::current()) {
    // 记录非致命诊断信息并继续执行；只用于确实不影响结果的诊断分支。
    static std::mutex logMutex;
    std::lock_guard lock(logMutex);
    std::ofstream log("mss_error.log", std::ios::app);
    if (log) {
        log << "[warn]\n"
            << "message: " << message << '\n'
            << "file: " << location.file_name() << '\n'
            << "line: " << location.line() << '\n'
            << "function: " << location.function_name() << "\n\n";
        log.flush();
        log.close();
    }
}

}  // namespace mss
