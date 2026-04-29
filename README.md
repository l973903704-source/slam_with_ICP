# slamware_loop_slam_project

This is a 2D LiDAR SLAM course project designed for **ROS1 Melodic + Python 2.7 + Slamtec Slamware ROS SDK**.

## Project Goal

The goal of this project is to build a complete 2D SLAM pipeline based on the LiDAR data provided by the Slamware ROS SDK.

Main functions include:

- Directly subscribe to the `/scan` topic published by `slamware_ros_sdk_server_node`
- Implement a **scan-to-scan ICP front-end**
- Add **keyframes, loop closure detection, and pose graph optimization** based on the ICP front-end
- Save the trajectory, loop closure edges, optimized results, and plotting results into the project-level `result/` folder

## Recommended Documents

Please read the following documents first:

- `上机步骤.md`
- `代码功能说明.md`
- `README_上机步骤.md`
- `README_代码说明.md`
- `离线依赖清单.txt`

## Running Environment

This project is intended to run on the **Unitree Go1 robot** with an Ubuntu-based ROS environment.

The main running environment is:

- Robot platform: **Unitree Go1**
- Operating system: **Ubuntu with ROS1 Melodic**
- ROS version: **ROS1 Melodic**
- Python version: **Python 2.7**
- LiDAR SDK: **Slamtec Slamware ROS SDK**
- LiDAR data source: `/scan` topic from `slamware_ros_sdk_server_node`

The SLAM algorithm, LiDAR topic subscription, ICP front-end, loop closure detection, and pose graph optimization are all executed inside the Go1 robot’s Ubuntu system.

## Remote Connection Method

During the experiment, I used a **Windows laptop** to remotely connect to the Unitree Go1 robot.

The general workflow is:


```text
Windows laptop
    ↓ remote connection
Unitree Go1 Ubuntu system
    ↓ run ROS and Slamware SDK
slamware_ros_sdk_server_node publishes /scan
    ↓
this project subscribes to /scan and runs SLAM
```

## Some unimportant things

<details>
<summary>Click to read</summary>

编代码时，我心尚平；<br>
初试debug，亦可自持；一改、二改、三改，虽有疲惫，犹能修补。<br>
我本以为，历经风霜，已学会成人之冷静克制。<br>
岂料最后破我道心者，非算法，非代码，非环境依赖，竟是Go1的电池、雷达还有那该死的小米WiFi，靠。<br>
在我修炼情绪稳定的路途上，Go1，你当真是功德无量。<br>


往哪拜才能让你在上机的时候丝滑运行，人为什么要在没有真机的时候写上机代码，上机环境怎么能没有网，<br>
gpt你看看你训练出来的兵，竟无人会古法编程哈哈哈哈哈哈，<br>
还有你个小米wifi能不能不要老断了，那个薛定谔的雷达能不能别时好时坏了，<br>
最朴素的解决方法竟然是拔USB我真是服了（绝望研究生belike）<br>
</details>
