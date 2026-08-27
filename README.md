# Navigation2D

面向差速扫地机器人的纯 C++ 2D 导航核心。该仓库保留 Nav2 方案中当前产品需要的
占用栅格、NavFn 风格全局规划和 Regulated Pure Pursuit 跟踪，不依赖 ROS、DDS、
pluginlib、生命周期节点、action 或行为树。

## 结构

```text
application/  导航闭环与 benchmark 入口
control/      Regulated Pure Pursuit 路径跟踪
mapping/      占用栅格、坐标转换与足迹碰撞检测
planning/     NavFn 风格栅格最短路径规划
types.h      导航模块共用的最小数据类型
```

## 构建

依赖 C++20、CMake 3.16 和 yaml-cpp：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

仓库根目录的 `tools/run_navigation2d_regression.py` 会只构建一次，然后用独立子进程
并行运行所有冻结 case。每个 case 直接加载数据集中的 occupancy grid，不启动容器，
也不存在 ROS domain、节点就绪或 action 超时竞争。

## 来源与许可

算法选择继承自 Open Navigation LLC 的 Navigation2 项目：NavFn 与 Regulated Pure
Pursuit。ROS 框架层已移除，代码按当前 2D 产品边界重新组织。原项目与本仓库许可见
`LICENSE`。
