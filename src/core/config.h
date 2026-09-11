#pragma once

namespace mss {

// 全局配置：只放跨层共享的魔法数字。
// 各层内部专用阈值（搜索 eps、近似参数等）放在所属层，不堆在这里。

// 暴力枚举的最大方案数上限。all_distribute / 残局求解在枚举前先预估候选
// 方案数，超过即放弃枚举（指数级枚举否则会卡死）；需放宽时改这个数。
inline constexpr int kMaxBruteforceCount = 200000;

}  // namespace mss
