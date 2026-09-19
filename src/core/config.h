#pragma once

namespace mss {

// 全局配置：只放跨层共享的魔法数字。
// 各层内部专用阈值（搜索 eps、近似参数等）放在所属层，不堆在这里。

// 暴力搜索的并行线程上限，包含调用线程。
inline constexpr int kMaxBruteforceCores = 5;

} // namespace mss
