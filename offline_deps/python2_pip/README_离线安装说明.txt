这个文件夹用于保存 Python2.7 离线 pip 包。

注意：
rospy、tf、sensor_msgs、geometry_msgs、nav_msgs 不是普通 pip 包，
它们来自 ROS Melodic 的 apt/deb 包，需要提前准备 ROS 环境或对应 .deb。

在有网电脑下载：
python -m pip download -r requirements_python2.txt -d packages

如果是在 Windows 上给 Ubuntu 18.04 / Python2.7 下载 Linux 包，建议用：
python -m pip download -r requirements_python2_linux_wheels.txt -d packages --platform manylinux1_x86_64 --python-version 27 --implementation cp --abi cp27mu --only-binary=:all: --no-deps
python -m pip download -r requirements_python2_pure.txt -d packages --no-deps --no-binary=:all:

在 Go1 上离线安装：
cd ~/slamware_loop_slam_project/offline_deps/python2_pip
python2 -m pip install --no-index --find-links packages -r requirements_python2.txt

也可以直接执行：
cd ~/slamware_loop_slam_project/offline_deps/python2_pip
bash install_offline_python2_deps.sh

如果 Go1 上没有 pip：
优先用 apt 离线安装 python-pip，或先安装本目录里的 pip/setuptools/wheel。

补充：
本目录里 subprocess32 等少数包是源码包。如果现场用 pip 安装源码包失败，
优先改用 apt 离线安装 python-subprocess32，或提前准备 build-essential、python-dev。
