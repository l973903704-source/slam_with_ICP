# README_代码说明

> 更新后的正式代码说明请优先看 `代码功能说明.md`。本文件保留为兼容旧文件名。

## 1. 项目整体思路

本项目把算法拆成两层：

1. **前端**：scan-to-scan ICP，用相邻两帧激光做配准，得到局部位姿增量
2. **后端**：关键帧 + 回环检测 + 位姿图优化

也就是说：

```text
/scan -> 点云化 -> ICP 前端 -> 局部轨迹
                      -> 关键帧筛选 -> 回环检测 -> 位姿图优化 -> 全局优化轨迹
```

## 2. 目录说明

```text
slamware_loop_slam_project/
├── README.md
├── README_上机步骤.md
├── README_代码说明.md
├── 离线依赖清单.txt
├── tools/
│   └── plot_results.py
└── catkin_ws/
    └── src/
        ├── CMakeLists.txt
        └── slamware_loop_slam/
            ├── CMakeLists.txt
            ├── package.xml
            ├── setup.py
            ├── config/
            ├── launch/
            ├── result/
            ├── scripts/
            └── src/slamware_loop_slam/
```

## 3. launch 文件作用

### `launch/scan_icp.launch`
只启动本项目的 **无回环 ICP 前端**。
适合在官方 Slamware server node 已经启动的情况下使用。

### `launch/pose_graph_slam.launch`
只启动本项目的 **回环 + 位姿图优化版本**。
同样要求官方 Slamware server node 已经提前启动。

### `launch/slamware_no_loop.launch`
整合版 launch。
会先 include 官方 `slamware_ros_sdk_server_node.launch`，再启动无回环前端。

### `launch/slamware_with_loop.launch`
整合版 launch。
会先 include 官方 `slamware_ros_sdk_server_node.launch`，再启动回环版 SLAM。

## 4. config 参数文件说明

### `config/scan_icp_params.yaml`
无回环前端参数。
主要控制：
- 订阅哪个 scan 话题
- ICP 最大迭代次数
- 点云降采样
- 最小有效点数
- 是否保存 CSV

### `config/pose_graph_slam_params.yaml`
回环版参数。
除了前端 ICP 参数之外，还控制：
- 关键帧选取阈值
- 回环候选搜索范围
- 描述子阈值
- 回环边 ICP 误差阈值
- 位姿图优化频率
- 里程计边和回环边权重

## 5. scripts 中两个主节点

### `scripts/scan_icp_node.py`
无回环前端节点。
它的流程是：

```text
收到 /scan
-> 转成 2D 点集
-> 与上一帧做 ICP
-> 得到位姿增量
-> 累积成轨迹
-> 发布 pose/path
-> 保存 CSV
```

### `scripts/pose_graph_slam_node.py`
回环版主节点。
它的流程是：

```text
收到 /scan
-> 转成 2D 点集
-> 与上一帧做 ICP
-> 更新局部轨迹
-> 判断是否加入关键帧
-> 对最新关键帧做回环候选搜索
-> 对候选回环做 ICP 验证
-> 把里程计边 / 回环边加入位姿图
-> 用 least_squares 做全局优化
-> 发布局部轨迹与优化轨迹
-> 保存 CSV
```

## 6. src/slamware_loop_slam 各模块功能

### `align.py`
实现 2D 刚体配准里的基本数学计算。
输入两组对应点，估计旋转和平移。

### `icp.py`
实现 2D ICP 主循环。
主要做：
- 最近邻匹配
- 去掉较差匹配
- 估计刚体变换
- 迭代直到收敛

### `pose.py`
实现 2D 位姿的基础操作：
- 位姿合成
- 位姿求逆
- 计算相对位姿
- 角度归一化

### `scan_to_points.py`
把 `sensor_msgs/LaserScan` 转成二维点集。
同时做：
- 最小 / 最大量程过滤
- 无穷值过滤
- 降采样

### `loop_detection.py`
实现轻量回环检测。
当前方法不是复杂的深度学习描述子，而是更适合课程项目的：
- 把一帧 scan 变成径向直方图描述子
- 用描述子距离和空间距离联合筛选回环候选

### `pose_graph.py`
实现位姿图优化。
把每一条边都看成一个约束，然后用 `scipy.optimize.least_squares` 优化所有关键帧位姿。

### `transform.py`
提供点云刚体变换等小工具函数。

## 7. result 文件夹里放什么

统一把实验结果都放到 `result/` 中，避免散落在系统临时目录。

- CSV：轨迹、优化结果、回环边
- PNG：绘图脚本生成的路径图和回环图
- 后续你们也可以继续往这里加日志、截图、实验记录

## 8. tools/plot_results.py 做什么

这个脚本读取 `result/` 中的 CSV，然后自动绘图。

### 当只有无回环结果时
会画：
- `no_loop_path.png`

### 当存在回环版结果时
会画：
- `pose_graph_compare.png`，对比局部轨迹和优化轨迹
- `pose_graph_loops.png`，显示回环边连线

## 9. 为什么默认不发布 TF

Slamware 官方节点本身已经会广播自己的 tf。
如果你的算法节点也默认发布 tf，很容易和官方 tf 冲突。
所以本项目默认：
- 算法节点发布 pose 和 path
- 不主动抢 tf

如果后续你们课程实验明确要求，也可以在 launch 参数里打开 tf 发布。
