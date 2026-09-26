# MSS 1.4 项目说明

## 1. 项目概览

MSS 1.4 是一个基于 C++20 的扫雷局面分析与残局求解项目。核心职责是把观测到的棋盘转换为可复用的约束模型，计算满足总雷数条件的概率，并在需要时对残局进行完整搜索。

项目当前以头文件实现为主，`src/main.cpp` 只负责启动测试 harness。生产算法和测试夹具分离，便于在保持同一套核心接口的前提下切换功能测试、人工构造盘面和性能测试。

## 2. 目录结构

```text
src/
├─ algo/
│  ├─ probability_engine/     概率引擎、结构分析和残局搜索
│  │  ├─ observed_board.h     观测棋盘和可逆更新
│  │  ├─ basic.h              局部确定性约束传播
│  │  ├─ structure.h          连通组件、Shape、Box 和池化
│  │  ├─ probability/         全局概率与点开结果分布
│  │  ├─ shape_solver/        组件分布求解和 Graph DP 顺序
│  │  └─ bruteforce/          残局暴力搜索后端
│  └─ winrate_solver/
│     └─ java_transplant/     Java 胜率求解器移植和长期风险分析
├─ core/
│  ├─ types.h                句柄和基础类型
│  ├─ workspace.h            按源文件集中管理的线程局部工作区
│  └─ utility/               Grid、哈希表、排序等基础组件
├─ test/                      功能、压力和性能测试
└─ main.cpp                   测试程序入口
```

项目文件是 `mss1.4.vcxproj`，解决方案文件是 `mss1.4.sln`。构建产物默认放在 `build-vs/` 或 `build-mingw/`，不应作为源码依赖提交。

## 3. 核心处理流水线

```text
ObservedBoard
    │  观测状态和 Delta
    ▼
Basic
    │  H/T/S/F 标记、局部约束传播
    ▼
Structure
    │  连通组件、Shape、Box、Instance
    ▼
ShapeSolver
    │  组件在各总雷数下的 ways 和 Box 期望
    ▼
Probability
    │  组件卷积、总雷数条件化、格子概率
    ├──────────────► 点开结果分布 observe
    └──────────────► 残局决策或 BruteForce
```

### 3.1 ObservedBoard

`ObservedBoard::Result` 保存盘面尺寸、总雷数和每格观测状态。坐标使用 1-based 语义，`Grid` 带一圈 padding；`CellId` 使用稠密整数编码，适合作为数组和池的下标。

`ObservedBoard::Delta` 记录一批格子的状态变化。正常增量流程必须先调用 `ObservedBoard::update`，再把同一批变化交给后续层。搜索或测试回滚时使用 `applyDelta(..., true)`，并保持 Delta 的逆序恢复语义。

### 3.2 Basic

`Basic::analyze` 从观测盘面建立初始标记并执行局部确定性传播：

- `H`：前沿候选，仍与已知数字相邻；
- `T`：普通 Unknown，尚未进入前沿；
- `S`：确定安全；
- `F`：确定为雷。

`Basic::Result::valid == false` 表示当前观测、强制状态或总雷数互相矛盾。无效结果不能继续交给概率层。

点击或强制状态发生变化时，优先使用 `Basic::update` 和 `Basic::applyDelta`，不要无条件重新构建整个结果。

### 3.3 Structure

`Structure` 将数字与前沿候选组成的约束图拆成独立组件。组件中具有相同数字邻接签名的候选格会压缩为一个 `Box`，因此后续求解枚举的是 Box 的雷数，而不是每个格子的具体雷位。

- `Shape`：不依赖具体坐标的约束形状；
- `Instance`：Shape 在当前盘面中的 Box 和数字位置；
- `structPool`：按内容哈希复用 Shape 和 Instance；
- `Result::cellLoc`：把当前格子映射到组件和 Box；
- `Structure::Delta`：支持受影响组件的增量重建和回放。

组件池只增不删，句柄只在所属池的生命周期内有效。结构更新必须在 `ObservedBoard` 和 `Basic` 已经同步更新后执行。

### 3.4 ShapeSolver

`ShapeSolver::analyze` 按组件 Box 数量选择后端：

- 小组件使用 DFS 枚举合法 Box 雷数分配；
- 较大组件使用 Graph DP，沿消元顺序维护 frontier 状态；
- `graphThreshold` 是性能切换点，不是正确性边界。

组件分布的 `ways[k]` 表示组件恰有 `k` 颗雷时的具体布局权重。分布还保存每个总雷数下各 Box 的期望雷数，供全局概率层展开到格子。

Graph DP 的 `OrderAlgo` 只影响消元顺序、状态峰值和耗时，不改变最终分布：

| 策略 | 特点 |
| --- | --- |
| `Adjacent` | 轻量贪心，顺序质量一般 |
| `Window3` | 观察局部三步，开销和顺序质量居中 |
| `SA` | 模拟退火，开销最高，顺序通常接近最优 |
| `Auto` | 混合轻量策略，默认用于通常局面 |
| `AutoSA` | 在混合策略基础上追加 SA，面向人工构造的极端局面 |

算法选择通过函数参数传入，例如：

```cpp
auto probability = Probability::analyze(board, basic, structure, shapes, distributions,
                                         ShapeSolver::OrderAlgo::Auto);
```

同一个 `Shape` 和 `OrderAlgo` 下，算法应保持确定性。顺序策略不能改变结果，只能改变计算代价。

### 3.5 Probability

`Probability::analyze` 把每个组件分布与组件外 Unknown 的组合数做全局卷积，再按总雷数条件化，生成：

- 每个组件 Box 的条件雷概率；
- 组件外 Unknown 的统一雷概率；
- 满足当前总雷数约束的加权方案数 `candidates()`。

`Probability::observe` 计算点击某个 Hidden 格后的数字或爆炸分布。它与全局概率共用组件分布，但使用独立的点开转移工作区。

### 3.6 BruteForce

`BruteForce` 用于概率分析之后的残局搜索。普通后端和多掩码后端使用不同的状态表示，但必须返回一致的可能性数量、胜局数和动作顺序。`Route::Automatic` 选择默认后端，`Route::Common` 可用于结果对拍。

暴力搜索只适合候选集合已经足够小的残局。调用方应先使用 Basic/Probability 结果缩小范围，避免把整盘未知状态直接交给搜索。

## 4. 推荐调用方式

### 4.1 首次分析

```cpp
mss::ObservedBoard::Result board = mss::ObservedBoard::analyze(rows, cols, mines);
mss::Basic::Result basic = mss::Basic::analyze(board);

mss::Structure::structPool shapes;
mss::Structure::Result structure = mss::Structure::analyze(board, basic, shapes);

mss::ShapeSolver::Distribution::Pool distributions;
mss::Probability::Result probability =
    mss::Probability::analyze(board, basic, structure, shapes, distributions,
                              mss::ShapeSolver::OrderAlgo::Auto);
```

### 4.2 增量更新

```cpp
mss::ObservedBoard::Delta boardDelta;
boardDelta.changes.push_back({cell, mss::ObservedBoard::CellState::Num3});

mss::ObservedBoard::update(board, boardDelta);
mss::Basic::Delta basicDelta;
mss::Structure::Delta structureDelta;

mss::Basic::update(basic, basicDelta, board, boardDelta);
mss::Structure::update(structure, structureDelta, board, basic, shapes, boardDelta);
mss::Probability::analyze(board, basic, structure, shapes, distributions, probability,
                          mss::ShapeSolver::OrderAlgo::Auto);
```

增量更新的顺序是固定契约：

```text
board → basic → structure → probability
```

搜索分支结束后，按相反顺序调用各层的 `applyDelta(..., true)` 恢复父状态。

## 5. Workspace 和内存复用

热路径使用的临时容器集中在 [src/core/workspace.h](../src/core/workspace.h)。每个源文件对应一个壳 struct，函数相关的类型和 `inline static thread_local` 工作区放在对应壳内。这样可以：

- 让不同线程拥有独立的临时容器；
- 在连续调用之间复用 `vector`、哈希表和 DP 层容量；
- 避免工作区定义散落在算法头文件中；
- 避免把不同函数的临时状态误合并。

工作区只负责暂存和容量复用，不拥有对外结果。返回的 `Result` 必须拥有自己的数据；调用方不能保存指向 workspace 内部容器的指针或 span。

新增热路径临时数据时，应先确认它属于哪个源文件和函数，再放入对应壳 struct；不要在算法函数中重新声明长期复用的线程局部容器。

## 6. 构建和运行

### MSVC

项目使用 C++20、x64 和 Release/Debug 两种配置。Visual Studio 中打开 `mss1.4.sln` 即可；命令行构建示例：

```powershell
msbuild mss1.4.vcxproj /t:Build /p:Configuration=Release /p:Platform=x64
```

可执行文件通常为：

```text
build-vs/Release/mss1.4-test.exe
```

### MinGW

仓库中的 `build.bat` 使用 `D:\codeenvset\mingw64\bin\g++.exe`，并以 `-std=gnu++20 -O3` 编译 `src/main.cpp`。如果工具链路径不同，应先调整脚本中的 `CXX`，不要把本机路径写入项目源码。

## 7. 测试入口

测试入口集中在 [src/test/harness.h](../src/test/harness.h)。一次通常只启用一个测试函数，通过注释切换：

- `observedBoard()`：观测状态、更新和回滚；
- `basic()`：局部传播与增量结果；
- `structure()`：组件、Box、池和 Delta；
- `probabilityCase()`：人工构造的概率压力盘面；
- `bruteforce()`：普通后端与多掩码后端对拍；
- `radixSort()`、`flatHashtable()`：基础组件回归；
- `performance()`：固定种子实战盘面吞吐基线；
- `real_endgame_performance(...)`：残局节点、耗时和结果比较。

性能测试使用固定种子，但运行时间和吞吐会随编译器、CPU、优化选项和系统负载变化。比较性能时应保持配置一致，关注同一环境下的相对变化，不把单次绝对值当作功能断言。

运行长时间性能测试时应设置明确的外部时间上限。人工盘面压力测试使用独立的超时配置，避免异常状态无限消耗资源。

## 8. 开发约定

- 新算法先明确输入、输出和不变量，再接入现有管线；不要绕过 Basic/Structure 直接复制整套推理。
- 需要新增算法选择时，通过显式 enum 或函数参数传递；不要使用隐藏的全局模式开关。
- 只读接口使用 `const` 引用；会修改状态的函数必须明确携带可逆 Delta 或结果对象。
- 句柄和 pool 的归属关系不可跨池混用；`span` 只能在拥有其底层存储的对象仍存活时使用。
- 生产算法中的随机行为必须可复现；排序和 tie-break 需要稳定且确定。
- 热路径优先复用已有工作区和容量，性能改动必须有功能测试或基准结果支撑。
- 修改后至少执行一次目标配置编译；涉及算法、状态回放或性能路径时补充对应测试。

## 9. 常见问题

### 程序没有运行想要的测试

检查 `src/test/harness.h` 当前实际取消注释的入口。`main.cpp` 只调用 `test::harness()`，不会自动发现测试。

### 分析结果变成 invalid

优先检查是否按 `ObservedBoard → Basic → Structure → Probability` 的顺序应用了同一批更新，以及是否重复使用了已经回写过 `previous` 的 Delta。

### Graph DP 很慢

先确认组件 Box 数量和消元顺序，再比较 `Adjacent`、`Window3`、`Auto` 和 `AutoSA`。顺序策略只应影响耗时和峰值状态，不应改变分布结果。

### 内存峰值异常

检查新增临时容器是否进入了 [src/core/workspace.h](../src/core/workspace.h)，以及调用之间是否使用 `clear/resize/reserve` 复用容量。不要只看某个 vector 的当前 size；哈希表和 DP 层的 capacity 也会保留。
