# README_上机步骤

> 更新后的正式上机说明请优先看 `上机步骤.md`。本文件保留为兼容旧文件名。

## 1. 项目结构

本项目的 ROS 包位于：

```text
catkin_ws/src/slamware_loop_slam
```

结果目录位于：

```text
catkin_ws/src/slamware_loop_slam/result
```

## 2. 重要说明

你们使用的是 **ROS1 Melodic + Python2.7**。
本项目中的两个节点脚本都已经按 Python2 写法处理。

官方 Slamware ROS SDK 建议的使用方式是：
1. 先启动 `slamware_ros_sdk_server_node.launch`
2. 再用 `view_slamware_ros_sdk_server_node.launch` 打开 RViz 观察地图、雷达和机器人状态

本项目的 SLAM 算法本身只需要 `/scan` 这个 LaserScan 话题即可。

## 3. 上机前准备

把以下内容提前拷到 go1：

- 本项目整个文件夹 `slamware_loop_slam_project`
- 如果实验环境断网，把 `离线依赖清单.txt` 里列出的依赖也提前准备好

## 4. 编译工作空间

进入项目目录后执行：

```bash
cd ~/slamware_loop_slam_project/catkin_ws
chmod +x src/slamware_loop_slam/scripts/*.py
catkin_make
source devel/setup.bash
```

如果你们机器上没有自动 source ROS 环境，先执行：

```bash
source /opt/ros/melodic/setup.bash
```

## 5. 先启动 Slamware 数据源

### 方案 A：只启动官方数据源

如果你想严格按照官方方式先起 Slamware，再单独起本项目算法：

```bash
source /opt/ros/melodic/setup.bash
source ~/UnitreeSLAM/catkin_lidar_slam_2d_go1/devel/setup.bash
roslaunch slamware_ros_sdk slamware_ros_sdk_server_node.launch ip_address:=192.168.31.11
```

然后另开一个终端，打开官方 RViz：

```bash
source /opt/ros/melodic/setup.bash
source ~/UnitreeSLAM/catkin_lidar_slam_2d_go1/devel/setup.bash
roslaunch slamware_ros_sdk view_slamware_ros_sdk_server_node.launch
```

### 方案 B：本项目直接帮你一起启动 Slamware 数据源

本项目已经写好了整合 launch，可以一条命令同时启动 Slamware 官方服务节点和你自己的 SLAM 节点。

#### 无回环基线版

```bash
cd ~/slamware_loop_slam_project/catkin_ws
source /opt/ros/melodic/setup.bash
source ~/UnitreeSLAM/catkin_lidar_slam_2d_go1/devel/setup.bash
source devel/setup.bash
roslaunch slamware_loop_slam slamware_no_loop.launch ip_address:=192.168.31.11
```

#### 回环 + 全局优化版

```bash
cd ~/slamware_loop_slam_project/catkin_ws
source /opt/ros/melodic/setup.bash
source ~/UnitreeSLAM/catkin_lidar_slam_2d_go1/devel/setup.bash
source devel/setup.bash
roslaunch slamware_loop_slam slamware_with_loop.launch ip_address:=192.168.31.11
```

## 6. 如果你已经单独启动了 Slamware 官方节点

那就不要再用 `slamware_no_loop.launch` / `slamware_with_loop.launch`，避免重复起官方节点。

此时只启动算法即可。

### 无回环前端

```bash
cd ~/slamware_loop_slam_project/catkin_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roslaunch slamware_loop_slam scan_icp.launch
```

### 回环版

```bash
cd ~/slamware_loop_slam_project/catkin_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roslaunch slamware_loop_slam pose_graph_slam.launch
```

## 7. 运行时如何检查是否真的收到雷达数据

先看话题：

```bash
rostopic list | egrep 'scan|odom|map'
```

检查 `/scan` 类型是否正确：

```bash
rostopic type /scan
```

应该看到：

```text
sensor_msgs/LaserScan
```

看频率：

```bash
rostopic hz /scan
```

## 8. 算法运行后会输出什么

### 无回环版输出

- `/slamware_icp_pose`
- `/slamware_icp_path`

### 回环版输出

- `/slamware_local_pose`
- `/slamware_local_path`
- `/slamware_pg_pose`
- `/slamware_pg_path`

## 9. 结果文件保存位置

所有结果默认保存在：

```text
catkin_ws/src/slamware_loop_slam/result
```

默认会生成：

### 无回环版
- `no_loop_path.csv`

### 回环版
- `pose_graph_local.csv`
- `pose_graph_optimized.csv`
- `pose_graph_loops.csv`

后续运行画图脚本后，还会生成：
- `no_loop_path.png`
- `pose_graph_compare.png`
- `pose_graph_loops.png`

## 10. 画图

运行结束后，可以执行：

```bash
cd ~/slamware_loop_slam_project
python2 tools/plot_results.py
```

如果环境里只有 python3 且 matplotlib 安装在 python3 环境，也可以改为：

```bash
python3 tools/plot_results.py
```

## 11. 建议的实际流程

### 做正式实验时
1. 启动 Slamware 官方 server node
2. 启动官方 RViz，确认 `/scan` 正常
3. 启动本项目的回环版 `pose_graph_slam.launch`
4. 手动带狗走一圈，尽量形成闭环路径
5. 结束后到 `result/` 检查 CSV 和 PNG

## 12. 常见问题

### 问题 1：提示找不到 `rospkg`
说明 Python 运行环境缺依赖，需要提前离线安装 `python-rospkg` 或 `python3-rospkg`。

### 问题 2：起了算法，但没有输出轨迹
先检查 `/scan` 是否真的有数据：

```bash
rostopic echo -n 1 /scan
```

### 问题 3：回环数量一直是 0
可能原因：
- 实际路线没有形成闭环
- `loop_candidate_distance` 太小
- `loop_descriptor_threshold` 太严格
- 环境特征较少

可以先按默认参数跑，再慢慢调。
