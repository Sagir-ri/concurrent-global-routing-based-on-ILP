# concurrent-global-routing-based-on-ILP
基本上是从Alliance NERO布线器的A*算法朝着基于ILP的并发全局布线改的
# Alliance NeRo 布线器 ILP 改进项目（实验性/半成品）

## 项目概述

这是一个**实验性项目**，尝试将 Alliance NeRo 布线器从传统的 A* 算法改进为基于整数线性规划（Integer Linear Programming, ILP）的并发全局路由算法。该项目采用 GLPK 数学优化求解器来替代原有的启发式搜索方法。

**重要提示：这是一个半成品项目，在实际测试中发现了显著问题，不建议用于生产环境。**

## 核心特性

### 项目目标
- **算法替换**: 尝试用 ILP 优化替代 A* 启发式搜索
- **并发路由**: 实现多网络同时优化的全局路由
- **数学建模**: 将布线问题转化为整数线性规划问题
- **性能提升**: 期望获得更优的布线质量和拥塞处理

### 技术实现
- **C++11/14 兼容**: 现代 C++ 标准实现
- **GLPK 集成**: 专业级数学优化库支持
- **接口兼容**: 保持与原 Alliance NeRo 的接口兼容
- **委托模式**: CAStar 类委托给 CILPRouter 实现

## 已知问题与限制

### 关键问题（mipsR3000 测试发现）

1. **层间布线不足**
   - 结果几乎全部在同一层进行布线
   - 通孔（via）数量极少，未充分利用多层结构
   - 可能原因：通孔惩罚系数过高或层间成本建模不当

2. **连接性问题**
   - 部分器件未能正确连接
   - 存在孤立的组件，影响设计完整性
   - 可能原因：路径生成算法或约束建模不完善

3. **网络隔离问题**
   - 每个网络独立进行 ILP 求解过于孤立
   - 缺乏网络间的全局协调和优化
   - 导致整体布线质量和资源利用不佳

4. **算法收敛性**
   - ILP 求解时间可能过长
   - 在复杂设计中可能无法找到可行解
   - 需要更好的启发式初始解

### 项目状态
- **实验阶段**: 核心功能已实现但存在重大缺陷
- **测试失败**: mipsR3000 测试结果不理想
- **不建议生产使用**: 需要大量改进工作

## 架构设计

### 核心类结构

```
CILPRouter (主路由器类)
├── GLPKSolver (GLPK求解器接口)
├── RoutingGraph (路由图结构)
├── NetData (网络数据表示)
├── Edge (边表示)
└── ResourceUsage (资源使用统计)
```

### 关键组件

1. **CILPRouter**: 主要的路由器类，负责整个路由流程
2. **GLPKSolver**: GLPK 求解器的 C++ 封装
3. **RoutingGraph**: 全局路由图的表示和管理
4. **NetData**: 单个网络的路径候选和变量管理

## 编译与安装

### 系统要求
- C++ 编译器 (GCC 4.8+ 或 Clang 3.4+)
- GLPK 开发库 (4.60+)
- Alliance CAD System

### 编译步骤

```bash
# 安装 GLPK 依赖 (Ubuntu/Debian)
sudo apt-get install libglpk-dev

# 编译项目
make clean
make FORCE_ILP_ONLY=1

# 或者使用 CMake
mkdir build && cd build
cmake -DFORCE_ILP_ONLY=ON ..
make
```

### 编译选项
- `FORCE_ILP_ONLY=1`: 强制启用纯 ILP 模式
- `NO_ASTAR_FALLBACK=1`: 禁用 A* 回退机制
- `DEBUG_ILP_CALLS=1`: 启用调试输出

## 使用建议与警告

### 使用前须知
```bash
# 仅用于研究和实验目的
echo "警告：这是实验性代码，不建议用于生产环境"

# 建议先在简单设计上测试
nero -G -v simple_test.vst simple_test.ap

# 复杂设计可能失败
nero -G -v mipsR3000.vst mipsR3000.ap  # 已知会有问题
```

### 故障排除

#### 常见失败场景
1. **通孔过少**: 调整 `via_penalty` 参数
2. **器件未连接**: 检查终端定义和路径生成
3. **ILP 求解超时**: 减少候选路径数量或增加时间限制
4. **内存不足**: 使用分块处理大型设计

#### 调试命令
```bash
# 启用详细调试输出
export DEBUG_ILP_CALLS=1
export ILP_VERIFY_MODE=1

# 检查日志文件
tail -f rnet_ilp_calls.log

# 分析失败原因
grep "FAILED" rnet_ilp_calls.log

# 使用对比脚本分析结果差异
python tools/routing_compare.py --files original.ap routed.ap

# 批量分析测试结果
python tools/routing_compare.py --dirs astar_baseline/ ilp_results/
```

### 编程接口

```cpp
#include "AAstar.h"

// 初始化 ILP 路由器
initializeGlobalRouter(drgrid, netsched);

// 获取路由器实例
CILPRouter* router = getGlobalRouter();

// 配置参数
router->setParameters(2.0, 1.5, 50);  // 拥塞惩罚, 通孔惩罚, 最大迭代数

// 路由单个网络
router->route(net);

// 并发路由多个网络
std::vector<CNet*> nets = {net1, net2, net3};
router->routeAllConcurrently(nets);

// 获取统计信息
auto usage = router->calculateResourceUsage();
std::cout << "线长: " << usage.total_wire_length << std::endl;
std::cout << "通孔数: " << usage.total_vias << std::endl;

// 清理
cleanupGlobalRouter();
```

## 测试与评估工具

### 布线结果对比脚本

项目附带了一个 Python 脚本 `routing_compare.py`，用于对比 A* 算法和 ILP 算法的布线结果差异。

#### 脚本功能
- **AP 文件解析**: 完整解析 Alliance AP 格式的布线结果文件
- **多维度对比**: 从多个角度分析两种算法的差异
- **详细统计**: 生成全面的对比报告和 JSON 格式的详细数据
- **批量处理**: 支持单文件对比和目录批量对比

#### 使用方法

```bash
# 对比单个文件
python routing_compare.py --files mips_astar.ap mips_ilp.ap

# 对比整个目录
python routing_compare.py --dirs astar_results/ ilp_results/
```

#### 对比维度

1. **基本统计对比**
   - 实例数量、连接数、线段数、通孔数
   - 变化百分比计算

2. **布线质量对比**
   - 总线长对比
   - 通孔数量对比
   - 布线密度分析

3. **布线路径对比**
   - 路径方向分布（水平/垂直）
   - 线段长度统计分析

4. **金属层使用对比**
   - 各层使用频率统计
   - 层间分布差异分析

5. **通孔使用对比**
   - 不同类型通孔统计
   - 通孔位置分布分析

6. **连接性对比**
   - 信号覆盖情况
   - 连接完整性检查

#### 示例输出格式

```
================================================================================
A* vs ILP Routing Comparison Analysis
================================================================================
A* File: mipsR3000_astar.ap
ILP File: mipsR3000_ilp.ap

📊 BASIC STATISTICS COMPARISON
--------------------------------------------------
Metric          A*         ILP        Difference   Change %
------------------------------------------------------------
instances       [分析中]   [分析中]   [对比结果]    [变化率]
connections     [分析中]   [分析中]   [对比结果]    [变化率]
segments        [分析中]   [分析中]   [对比结果]    [变化率]
vias            [分析中]   [分析中]   [对比结果]    [变化率]

🎯 ROUTING QUALITY COMPARISON
--------------------------------------------------
Total Wire Length:
  A*:  [数值] units
  ILP: [数值] units
  Change: [对比结果]

Total Vias:
  A*:  [数值]
  ILP: [数值]
  Change: [对比结果]

[结果分析]: Files are [IDENTICAL/DIFFERENT]
   The ILP algorithm [分析结论]
```

#### 生成的报告文件示例

脚本会自动生成 JSON 格式的详细报告：
```json
{
  "timestamp": "20241217_143022",
  "astar_file": "mipsR3000_astar.ap",
  "ilp_file": "mipsR3000_ilp.ap", 
  "astar_stats": {
    "instances": "[解析结果]",
    "connections": "[解析结果]",
    "segments": "[解析结果]",
    "vias": "[解析结果]"
  },
  "ilp_stats": {
    "instances": "[解析结果]",
    "connections": "[解析结果]",
    "segments": "[解析结果]",
    "vias": "[解析结果]"
  },
  "identical": "[true/false]"
}
```

### mipsR3000 处理器测试结果

使用对比脚本 `routing_compare.py` 对 mipsR3000 处理器进行布线测试，发现了以下具体问题：

在对 mipsR3000 处理器进行布线测试时，发现了以下具体问题：

#### 对比脚本测试结果
```bash
# 运行对比分析
python routing_compare.py --files mipsR3000_astar.ap mipsR3000_ilp.ap

# 脚本可以帮助分析：
测试设计: mipsR3000 RISC 处理器
分析维度: 
  - 基本统计对比（实例数、连接数、线段数、通孔数）
  - 布线质量对比（总线长、通孔数量、布线密度）
  - 布线路径对比（方向分布、线段长度统计）
  - 金属层使用对比（各层使用频率）
  - 通孔使用对比（类型分布）
  - 连接性对比（信号覆盖情况）

已发现问题:
  - 布线集中在单一层（通常是最低层）
  - 通孔使用率极低（<5%，正常应为20-30%）
  - 部分器件未连接（连接性问题）
  - 整体布线质量不如原A*算法
```

#### 根本原因分析

1. **成本函数建模缺陷**
   ```cpp
   // 当前实现的问题
   if (edge.direction >= 4) cost *= _via_penalty;  // 通孔惩罚过高
   
   // 建议改进
   // 需要更细致的层间成本建模
   ```

2. **网络独立优化的局限性**
   ```cpp
   // 当前：每个网络单独求解
   void CILPRouter::route(CNet* pNet) {
       // 单网络 ILP 求解，缺乏全局视野
   }
   
   // 需要：真正的并发全局优化
   void routeAllConcurrently(std::vector<CNet*>& nets);
   ```

3. **路径生成策略不完善**
   - A* 启发式路径生成可能偏向平面路由
   - 候选路径多样性不足
   - 缺乏有效的层间路径探索

## 改进方向与建议

### 短期改进
1. **调整通孔惩罚系数**
   ```cpp
   // 当前设置
   router->setParameters(2.0, 1.5, 50);  // via_penalty = 1.5 太高
   
   // 建议调整
   router->setParameters(2.0, 0.8, 50);  // 降低通孔惩罚
   ```

2. **改进路径生成**
   - 增加强制层间路径候选
   - 实现更智能的 3D 路径探索算法
   - 平衡平面路径和层间路径的权重

3. **增强连接性检查**
   ```cpp
   // 添加连接性验证
   void verifyConnectivity(CNet* net);
   void repairBrokenConnections(CNet* net);
   ```

### 中期改进
1. **真正的并发全局优化**
   - 将所有网络同时放入一个大型 ILP 问题
   - 考虑网络间的相互影响和资源竞争
   - 实现分层分解算法处理大规模问题

2. **改进数学建模**
   ```cpp
   // 更精确的目标函数
   minimize: Σ(wire_length) + α·Σ(congestion) + β·Σ(via_count) + γ·Σ(timing_cost)
   ```

3. **自适应参数调整**
   - 根据设计特征自动调整参数
   - 实现机器学习辅助的参数优化

### 长期改进
1. **混合算法架构**
   - 结合 A* 的快速性和 ILP 的优化性
   - 使用 A* 生成初始解，ILP 进行局部优化
   
2. **分层优化策略**
   - 全局路由 + 详细路由的分层处理
   - 不同层面使用不同的优化策略

## 当前实现状态

### 已实现功能 ✅
- [x] 基本 ILP 框架搭建
- [x] GLPK 求解器集成
- [x] 与 Alliance NeRo 的接口适配
- [x] 基础路径生成算法
- [x] 简单的拥塞处理机制
- [x] 统计和日志功能

### 存在问题 ❌
- [ ] 通孔利用率极低
- [ ] 部分器件无法连接
- [ ] 网络间缺乏全局协调
- [ ] 算法收敛性不佳
- [ ] 性能不如原 A* 算法

### 待完成功能 ⏳
- [ ] 层间路由优化
- [ ] 连接性保证机制
- [ ] 真正的并发全局优化
- [ ] 参数自适应调整
- [ ] 详细的性能基准测试

## 配置参数

### 关键参数
- **拥塞惩罚系数** (`congestion_penalty`): 默认 2.0
- **通孔惩罚系数** (`via_penalty`): 默认 1.5  
- **最大迭代数** (`max_iterations`): 默认 50
- **候选路径数** (`num_candidate_paths`): 默认 3

### 调优建议
```cpp
// 高密度设计
router->setParameters(3.0, 2.0, 100);

// 快速路由
router->setParameters(1.5, 1.2, 20);

// 高质量路由
router->setParameters(2.5, 1.8, 200);
```

## 文件结构

```
src/
├── AAstar.h          # ILP路由器头文件
├── AAstar.cpp        # ILP路由器实现
├── ADefs.h           # 系统定义和接口
├── ASimple.cpp       # 简单路由器（使用ILP）
├── nero.cpp          # 主程序入口
└── RNet.cpp          # 网络路由实现

tools/
└── routing_compare.py # A*与ILP结果对比脚本

logs/
├── rnet_ilp_calls.log
├── ilp_verification.log
├── emergency_ilp_stats.log
└── routing_comparison_*.json  # 对比脚本生成的报告

test/
├── simple/
│   ├── inv.vst          # 简单反相器（测试通过）
│   └── adder4.vst       # 4位加法器（部分通过）
├── medium/
│   ├── alu.vst          # ALU（通孔问题）
│   └── cache.vst        # 缓存（连接问题）
└── complex/
    └── mipsR3000.vst    # MIPS处理器（严重问题）
```

## 兼容性说明

### 与原系统兼容
- **CAStar 接口**: 保持原有接口，内部委托给 ILP
- **CASimple 集成**: 无缝集成到现有调度系统
- **异常处理**: 兼容原有异常类型

### 版本要求
- Alliance CAD System
- GLPK 4.60+
- C++11 编译器

## 调试与故障排除

### 常见问题

1. **编译错误**: 检查 GLPK 库是否正确安装
   ```bash
   pkg-config --libs glpk
   ```

2. **运行时错误**: 确保定义了必要的宏
   ```cpp
   #define FORCE_ILP_ONLY 1
   #define NO_ASTAR_FALLBACK 1
   ```

3. **路由失败**: 检查网络连通性和设计规则

### 调试模式
```bash
# 启用调试输出
make DEBUG_ILP_CALLS=1

# 运行时验证
export ILP_VERIFY_MODE=1
```

## 性能对比

| 指标 | 原 A* 算法 | 当前 ILP 实现 | 期望 ILP 改进 |
|------|------------|---------------|---------------|
| 布线完成率 | 基准表现 | 明显低于A* | 期望超越A* |
| 通孔利用率 | 正常范围 | 极低（<5%） | 合理利用多层 |
| 运行时间 | 快速 | 较慢 | 可接受范围 |
| 布线质量 | 良好 | 存在问题 | 数学最优 |
| 拥塞处理 | 基础启发式 | 理论先进但实现有缺陷 | 全局优化 |
| 内存使用 | 低 | 高 | 中等 |

**注意**: 具体数值需要实际测试获得，对比脚本可提供精确的量化分析。

### mipsR3000 具体测试发现
```
设计规模: ~3000 器件, ~8000 网络

测试发现的问题:
  - 布线完成率低于预期
  - 通孔使用率极低（<5%，正常应为20-30%）
  - 部分器件未能正确连接
  - 运行时间比A*算法长
  
对比脚本可提供:
  - 精确的统计数据对比
  - 详细的布线质量分析
  - 层间使用情况评估
  - 连接性完整性检查
```

**注意**: 具体的数值需要实际运行对比脚本获得，这里不提供虚构数据。

## 贡献指南

### 当前急需改进的领域

1. **通孔优化算法** (重要)
   - 重新设计通孔成本函数
   - 实现强制层间路径生成
   - 优化 3D 路径搜索策略

2. **连接性保证机制** (重要)
   - 实现端到端连接验证
   - 添加修复断开连接的算法
   - 改进终端处理逻辑

3. **全局并发优化** (重要)
   - 真正的多网络同时优化
   - 大规模 ILP 问题分解
   - 网络间相互作用建模

### 开发环境设置
```bash
# 克隆项目
git clone <repo_url>
cd alliance-nero-ilp

# 安装依赖
sudo apt-get install libglpk-dev alliance
pip3 install json  # 对比脚本依赖

# 编译测试版本
make clean
make FORCE_ILP_ONLY=1 DEBUG_ILP_CALLS=1

# 运行测试
./test_simple_designs.sh

# 使用对比脚本分析结果
python tools/routing_compare.py --files baseline.ap result.ap
```

### 测试数据
```
test/
├── simple/
│   ├── inv.vst          # 简单反相器（测试通过）
│   └── adder4.vst       # 4位加法器（部分通过）
├── medium/
│   ├── alu.vst          # ALU（通孔问题）
│   └── cache.vst        # 缓存（连接问题）
└── complex/
    └── mipsR3000.vst    # MIPS处理器（严重问题）
```

### 代码结构待优化
```cpp
// 关键需要重构的类
class CILPRouter {
    // 需要分离关注点
    GLPKSolver* _solver;           // 求解器
    PathGenerator* _path_gen;      // 路径生成器  
    ConnectivityChecker* _checker; // 连接性检查器
    CongestionManager* _congestion; // 拥塞管理器
};
```

## 项目总结

### 原始愿景
将 Alliance NeRo 从启发式 A* 算法升级为数学优化的 ILP 算法，实现更优的全局布线质量。

### 当前现实
项目在理论框架上取得进展，但在实际测试中暴露出严重的工程实现问题：
- **理论可行，实现有缺陷**
- **适用于简单设计，复杂设计失败**
- **需要大量额外工作才能达到生产级别**

### 未来展望
这个项目为 VLSI 布线算法的研究提供了有价值的探索：
1. **ILP 在布线中的可行性得到初步验证**
2. **发现了传统启发式算法的一些局限性**
3. **为后续混合算法研究奠定基础**

### 免责声明
- 这是一个**实验性/学术性项目**
- **不适用于生产环境**
- 测试结果表明**当前实现不如原始 A* 算法**
- 需要**显著的额外投入**才能达到可用状态

---

**最后建议**: 如果您需要可靠的布线工具，请继续使用原始的 Alliance NeRo A* 实现。这个 ILP 版本更适合作为研究参考和算法探索的起点。
