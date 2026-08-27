# C++ API 概览

## 稳定入口

产品调用方只需要 `application/navigation_system.h` 和配置文件。planner/controller 具体类是
可替换实现，不是跨仓库集成接口。

## `NavigationSystem`

```cpp
NavigationSystem(NavigationConfig config, const std::string& map_path);
void SetGoal(Pose2d goal);
void ClearGoal();
void UpdateLaserScan(const Pose2d& sensor_pose, const LaserScan& scan);
void UpdatePointCloud(const Pose2d& sensor_pose, const PointCloud2d& cloud);
NavigationState ComputeCommand(const Pose2d& pose,
                               Twist2d measured_velocity,
                               double timestamp);
```

调用方负责提供 localization2d 输出的当前全局位姿、底盘实测速度、单调时间戳和已经标定到
正确传感器位姿的观测。核心不读取时钟、不积分真实机器人位姿、不发布控制指令。

## 值类型

- `Pose2d`：`Sophus::SE2d`，单位为米和弧度。
- `Twist2d`：差速底盘线速度 m/s 与角速度 rad/s。
- `LaserScan`：角度、量程边界和 ranges；无效量程由调用方按 IEEE 非有限值表达。
- `PointCloud2d`：传感器坐标系击中点，元素为 `Eigen::Vector2d`。
- `Path`：`std::vector<Pose2d>`。

## `NavigationState`

状态为 `kIdle`、`kNavigating`、`kSucceeded` 或 `kBlocked`。同时返回速度命令、重规划、
急停、恢复计数、首条全局路径长度和当前 costmap digest。到达条件同时检查 XY 和 yaw。

## 错误处理

配置 schema、地图分辨率和未知算法错误通过异常报告；运行期无路径转换为 `kBlocked`，不会
让异常越过控制循环。调用方仍必须设置独立的通信超时和最终硬件急停。
