#pragma once

namespace mss {

// 全局配置：只放跨层共享的魔法数字。
// 各层内部专用阈值（搜索 eps、近似参数等）放在所属层，不堆在这里。

// 暴力枚举的最大方案数上限。残局入口在构建 CommonSession 前先估算组件赋值与
// Unknown 组合数，超过即拒绝进入指数枚举；这个值控制可接受的最坏延迟。
inline constexpr int kMaxBruteforceCount = 200000;

}  // namespace mss
