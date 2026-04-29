# slamware_loop_slam_project

这是一个面向 **ROS1 Melodic + Python2.7 + Slamtec Slamware ROS SDK** 的 2D 激光 SLAM 课程项目。

项目目标：
- 直接订阅 `slamware_ros_sdk_server_node` 发布的 `/scan`
- 实现 **scan-to-scan ICP 前端**
- 在此前端基础上加入 **关键帧、回环检测、位姿图优化**
- 将轨迹、回环边、优化结果、绘图结果统一保存到项目内的 `result/` 文件夹

建议先看：
- `上机步骤.md`
- `代码功能说明.md`
- `README_上机步骤.md`
- `README_代码说明.md`
- `离线依赖清单.txt`

断网上机前，Python2 离线 pip 包已经放在：

```text
offline_deps/python2_pip/packages
```
