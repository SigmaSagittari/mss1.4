#pragma once

#include "basic/basic.h"

namespace mss {

// 全项目工作区汇总：所有模块的复用缓冲都挂在这一个地方。
//   - 一眼看清内存去向（哪个模块吃得多），不用满仓库找 thread_local 全局；
//   - 访问形如 workspace.basic.XXX —— 每个模块只碰自己那一份，
//     跨模块污染在语法上就不可能。
// 新增模块时在这里加一行：Foo::Scratch foo;
struct Workspace {
    Basic::Scratch basic;
};

} // namespace mss