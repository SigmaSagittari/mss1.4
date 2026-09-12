/*
 * mss1.4 编码规约（强制；src/algo/ref/ 不适用）。
 * 新增或修改代码前先遵守本规约；无法遵守时必须先说明原因并获批。
 *
 * 组织与依赖
 * - 项目使用 C++20；除 main.cpp 外保持头文件化，头文件用 #pragma once，定义用 inline。
 * - 标准库 include 在前、项目 include 在后；只 include 直接依赖；禁止 using namespace。
 * - 生产代码放 mss，测试代码放 test；类型 PascalCase，函数/局部变量 camelCase，常量 kCamelCase。
 *   私有成员用 trailing underscore；radix_sort 是既有的算法命名例外。
 * - 对外接口优先使用 struct + 嵌套 Result/Delta/Pool；实现按声明后 //============================================================================== 分段。
 * - 4 空格缩进、K&R 大括号；短的单语句分支可省略大括号，但嵌套逻辑必须保持清楚。
 *
 * 数据与不变量
 * - Grid 使用 1-based 坐标和一行一列 padding；CellId 是 x*(cols+1)+y 的稠密 int 句柄。
 * - CellId/ComponentId/BoxId/ShapeId/DistributionId 只作所属 vector/pool 的下标；越界就是 bug。
 * - 身份使用句柄和 pool，不使用裸指针；interned pool 只增不删，缓存命中依赖完整 U128 内容身份。
 * - 结构变化沿用 analyze / update / applyDelta(reverse=true)；Delta 必须能反向恢复父状态。
 * - 需要拥有大块数据的 Result 默认 move-only；span 只指向其所属对象仍然存活的存储。
 *
 * 错误处理与表达
 * - 信任已测组件和明确的调用契约；禁止为不可达路径添加回退值、静默修复或重复防御检查。
 * - 不可达路径直接 assert(false) 或 exit；边界契约使用 assert_，测试失败使用 test::check。
 * - 热路径禁用 assert；正常的空结果、false 和 -1 哨兵仍可作为业务结果返回。
 * - 遵循项目既有转换习惯，默认不添加非必要的 static_cast；只有语义或编译确实要求时才使用。
 *   C 风格的 (int) 等既有写法不因个人偏好改写；不使用异常、new/delete 表达式或宏式业务逻辑。
 *
 * 性能与资源
 * - 热路径优先复用 thread_local 工作区、clear/resize/reserve 和调用方提供的临时 buffer；避免循环内分配。
 * - 整数句柄、紧凑的 uint8_t/char 存储、FlatHashTable、DynamicBitset 属于既有性能模型，不随意换回通用容器。
 * - 算法专用阈值放在所属模块；跨层共享的魔法数字才放 core/config.h；改动性能必须用测试或基准验证。
 * - FlatHashTable 只支持插入/查找，不支持删除；clear 保留容量，调用方必须遵守这个生命周期约定。
 *
 * 测试与改动边界
 * - 测试使用确定输入/种子、inline test 函数、test::check 和成功输出；新增算法必须覆盖核心结果及 Delta 回放。
 * - 注释只写不变量、接口契约和不明显的性能理由；不要用注释掩盖错误实现。
 * - 默认一次只改一个文件；只修用户指出的问题，保持无关改动不变；禁止擅自添加胶水代码。
 */

#include <chrono>
#include <iostream>

#include "algo/bruteforce/bruteforce.h"
#include "algo/observed_board.h"

int main() {
    auto board = mss::ObservedBoard::analyze(5, 5, 4);
    const auto basic = mss::Basic::analyze(board);
    mss::Structure::ShapePool shapes;
    const auto structure = mss::Structure::analyze(board, basic, shapes);
    const mss::BruteForce::Config config{false, 1};
    const auto start = std::chrono::steady_clock::now();
    const auto result = mss::BruteForce::solve(
        board, basic, structure, shapes, config);
    const double milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    std::cout << "5x5/4 possibilities=" << result.possibilities
              << " wins=" << result.moves[0].wins
              << " nodes=" << result.nodes
              << " time_ms=" << milliseconds << '\n';
    return 0;
}
