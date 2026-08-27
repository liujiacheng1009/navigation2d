# 开发、验证与文档贡献指南

## 修改前确认边界

- 产品行为：`application/`、`costmap/`、`planning/`、`control/`、`sensor/`、`geometry/`。
- 测试环境：`simulation/`、根仓库 `datasets/navigation2d` 和 `tools/`。
- 不要为了 benchmark 方便把真值、位姿积分或场景生成接口放进产品核心。

## 构建门禁

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

编译启用 `-Wall -Wextra -Wpedantic -Werror`。新增依赖必须固定版本并进入 CMake，不允许只在
开发机手工安装后隐式使用。

## 分级回归

- 文档改动：检查链接、命令、配置字段和 `git diff --check`。
- 等价重构：C++ 单测 + weekly 9-case，并逐 case 比较轨迹 SHA256 与 costmap digest。
- 算法/参数变化：在上述基础上比较终点误差、航向误差、碰撞、路径长度、重规划和恢复指标，
  明确说明为什么允许 hash 改变。

## 多进程评测

```bash
python3 tools/run_navigation2d_regression.py --tier weekly --jobs 8
```

每个 case 必须是独立进程；共享一个 NavigationSystem、静态全局状态或输出文件会破坏并发
确定性。新增 case 后同步更新 manifest、tier 选择测试和数据集 README。

## 提交边界

优先拆为：算法/产品实现、benchmark/数据集、文档、基线更新。不要把无关清理混入算法提交；
未经明确要求不更新轨迹基线、不提交生成报告、不推送远端。
