#pragma once

namespace mss {

// 版本号唯一来源：CMake 不带 VERSION，避免两处漂移。
// 消费者：测试/日志的 run 头。
inline constexpr char kVersion[] = "1.5.0";

} // namespace mss