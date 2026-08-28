# Navigation2D 文档

本文档覆盖当前纯 C++、无 ROS 的 Navigation2D 实现。代码入口、配置字段和回归命令均以
本仓库 `main` 分支为准。

## 快速入口

- [入门教程](GETTING_STARTED.md)：构建、运行单个 case 和解释输出。
- [系统架构](ARCHITECTURE.md)：产品数据流、所有权、调度与故障边界。
- [MPPI 控制器](MPPI_CONTROLLER.md)：开源来源、算法步骤、critics 与性能边界。
- [约束 MPC / MPCC 控制器](MPC_CONTROLLER.md)：轮廓控制、曲率约束、动态预测与后端边界。
- [全局规划器升级路线图](GLOBAL_PLANNER_ROADMAP.md)：State Lattice、anytime、增量修复与发布门槛。
- [模块索引](MODULES.md)：目录、类和职责的对应关系。
- [C++ API](API_REFERENCE.md)：产品入口和稳定值类型。
- [配置参考](CONFIGURATION_REFERENCE.md)：严格 YAML schema 与调参顺序。
- [开发与回归](DEVELOPMENT_GUIDE.md)：测试分级、多进程评测和轨迹 hash 门禁。

## 推荐阅读路径

第一次运行按“入门教程 → 配置参考”；修改算法按“系统架构 → 模块索引 → 开发与回归”；
接入 localization2d 或真实底盘按“C++ API → 系统架构”。

## 文档边界

`simulation/` 和 benchmark 数据集只用于确定性验证，不属于产品 API。文档不得把仿真真值、
位姿积分或动态障碍生成描述成 `NavigationSystem` 的运行时职责。
