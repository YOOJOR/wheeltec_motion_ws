# 小车 workspace：当前部署与第二台车复刻

基线 v0.3.0，2026-09-26。本文的维护源在 wheeltec_motion_ws/docs/WORKSPACE_SETUP.md，部署副本为小车 `~/workspace/README.md`。后续任务先读 motion 仓库的 `docs/STAGE_SUMMARY.md`；本文负责安装复刻，不依赖历史日志。

## 1. 目录与锁定来源

环境：Ubuntu 22.04 / ROS 2 Humble，小车用户 wheeltec。以下命令假定 Humble 已安装，`~/workspace` 是本车工作区。第一次复刻在空目录执行，不覆盖已有 src、install 或配置。

| 目录 | 内容 | 获取方式 / 锁定版本 |
| --- | --- | --- |
| wheeltec_base_ws | 厂家通信、消息、serial 三包 | 从厂家原工作空间抽取；本地基线 fe99b01f8e047f686aa2d004ddb38a829f909a3b |
| vendor_ws/src/Livox-SDK2 | Livox C++ SDK2 | Livox-SDK/Livox-SDK2，08f523c930b2f0ba1e98a6afaa8d7476bf479908 |
| vendor_ws/src/livox_ros_driver2 | ROS2 驱动 | Livox-SDK/livox_ros_driver2，4a1def929e5b59c7a8122d19fce6efba581ce9f7 |
| conavGPT_ws/src/FAST_LIO_ROS2 | 当前定位源码 | Ericsii/FAST_LIO_ROS2，ros2 分支，2fffc570a25d0df172720bac034fbdb6a13d2162 |
| wheeltec_motion_ws | 自研闭环控制、Action、标定、启动脚本 | YOOJOR/wheeltec_motion_ws，v0.3.0 |
| archive/motion-before-v0.3.0 | 清理前资料备份 | 仅用户要求排查历史时读取 |

base 不上传 GitHub。vendor/FAST-LIO 本次不改、不搬；conavGPT_ws 目前没有接入 Co-NavGPT2。不要复制 build/install 到第二台车；绝对路径、架构和 underlay 都可能不同。

## 2. 本车实际改动审计

| 组件 | 相对锁定源码的实际变化 |
| --- | --- |
| base 三包 | 45 个源码文件逐一对比厂家来源，无差异。抽取时去掉嵌套 .git 和 Python 缓存，新增过本地 README、构建脚本与记录；通信代码、参数、协议、固件未改 |
| Livox 驱动 | 源码 config/MID360s_config.json：192.168.1.12 → 192.168.1.45；当前安装副本实际是 192.168.1.145。主机 IP 均沿用 192.168.1.5。源码与安装副本未同步 |
| FAST-LIO | config/mid360.yaml 的 extrinsic_est_en：true → false。另有 RViz 窗口/视角/强度显示变化、删除 PCD/1 占位文件；不影响定位算法，复刻不必照搬显示改动 |
| SDK2 | 源码 Git 干净。现有 build 缓存指向旧 Documents/navGPT_ws 目录，不能据此证明 /usr/local 二进制对应当前 HEAD；第二台车从锁定源码重新编译 |

FAST-LIO 的 extrinsic_T=[-0.011,-0.02329,0.04412]、单位 extrinsic_R 已来自该上游提交，并非我们另打补丁。它描述 LiDAR 相对内部 IMU，不是底盘中心偏移。当前脚本启动 MID360S 驱动并使用 FAST-LIO mid360.yaml；精确内部尺寸仍需按型号资料核对。

原第三方差异另保存为 `archive/motion-before-v0.3.0/fastlio-local-changes.patch` 和 `livox-driver-local-changes.patch`，不作为日常上下文。

## 3. 系统依赖与干净环境

已安装 Humble 的 Ubuntu 上安装构建/运行依赖（厂家可能已预装）：

```bash
sudo apt update
sudo apt install -y git rsync build-essential cmake pkg-config python3-dev \
  python3-colcon-common-extensions python3-rosdep python3-numpy \
  python3-matplotlib python3-pytest python3-yaml libeigen3-dev libpcl-dev \
  libapr1-dev libboost-all-dev ros-humble-ament-cmake-auto \
  ros-humble-rosidl-default-generators ros-humble-rosbag2-py \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-tf2-geometry-msgs \
  ros-humble-turtlesim ros-humble-nav2-msgs ros-humble-ackermann-msgs \
  ros-humble-rviz2 ros-humble-launch-ros gnome-terminal
```

每个工作空间在新干净 shell 中编译，避免 .bashrc 自动加载旧厂家大空间；保留 HOME，仅清除旧 ROS overlay：

```bash
env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin \
  LANG=C.UTF-8 bash --noprofile --norc
source /opt/ros/humble/setup.bash
mkdir -p ~/workspace
```

若另有 ROS 依赖缺失，可在每节源码准备好后使用 rosdep 补系统依赖；先初始化/更新 rosdep，再用 `--ignore-src`，不要为 serial、livox 等本地包寻找不存在的系统替代品。base 原厂头文件仍依赖 turtlesim 的 Spawn 服务；漏 wheeltec_robot_msg 或 serial 是拆包常见失败原因。

## 4. 独立 base：复制完整三包，再编译

选择一条获取路径。最容易是从第一台车只复制 src，或复制其整个工作空间后在第二台车重新生成 build/install。下列命令在第二台车执行，SSH 地址改成第一台车实际地址：

```bash
mkdir -p ~/workspace/wheeltec_base_ws/src
rsync -a --exclude=.git --exclude=__pycache__ \
  wheeltec@FIRST_ROBOT:~/workspace/wheeltec_base_ws/src/ \
  ~/workspace/wheeltec_base_ws/src/
```

如果第二台车也有同版厂家 `~/wheeltec_ros2`，可直接复刻原抽取步骤（与上述复制二选一）：

```bash
mkdir -p ~/workspace/wheeltec_base_ws/src
rsync -a --exclude=.git --exclude=__pycache__ \
  ~/wheeltec_ros2/src/turn_on_wheeltec_robot ~/workspace/wheeltec_base_ws/src/
rsync -a --exclude=.git --exclude=__pycache__ \
  ~/wheeltec_ros2/src/wheeltec_robot_msg ~/workspace/wheeltec_base_ws/src/
rsync -a --exclude=.git --exclude=__pycache__ \
  ~/wheeltec_ros2/src/depend/serial_ros2 ~/workspace/wheeltec_base_ws/src/
```

目录名 serial_ros2 对应包名 **serial**。本车车型/IMU 默认值已由厂家配置为 senior_mec_bs/stm32，不是抽取时改出来的。其他车核对 `turn_on_wheeltec_robot/config/wheeltec_param.yaml` 和 `config/imu.yaml`，不要把另一车型默认值当成本车值。

在只有系统 Humble 的干净 shell 编译：

```bash
cd ~/workspace/wheeltec_base_ws
colcon build --base-paths src --symlink-install --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
ros2 pkg prefix turn_on_wheeltec_robot
ros2 pkg prefix wheeltec_robot_msg
ros2 pkg prefix serial
```

三个路径应只指向独立 base。第一台车 `/dev/wheeltec_controller` 已指向 `/dev/ttyCH343USB0`，无需重装 udev；第二台车核对 `ls -l /dev/wheeltec_controller` 和串口权限，再按厂家对应设备规则设置。不要盲目把第一台的 tty 编号或 USB 标识复制过去。

轻量通信入口是 `base_serial.launch.py`，本车无需重复传默认参数；明确传参也可以。完整 turn_on_wheeltec_robot.launch.py 还依赖 EKF/URDF 等，未纳入此三包空间。

## 5. SDK2 与 Livox 驱动

在只有系统 Humble 的干净 shell：

```bash
mkdir -p ~/workspace/vendor_ws/src
git clone https://github.com/Livox-SDK/Livox-SDK2.git \
  ~/workspace/vendor_ws/src/Livox-SDK2
git -C ~/workspace/vendor_ws/src/Livox-SDK2 checkout \
  08f523c930b2f0ba1e98a6afaa8d7476bf479908
cmake -S ~/workspace/vendor_ws/src/Livox-SDK2 \
  -B ~/workspace/vendor_ws/sdk_build -DCMAKE_BUILD_TYPE=Release
cmake --build ~/workspace/vendor_ws/sdk_build -j2
sudo cmake --install ~/workspace/vendor_ws/sdk_build
sudo ldconfig
git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
  ~/workspace/vendor_ws/src/livox_ros_driver2
git -C ~/workspace/vendor_ws/src/livox_ros_driver2 checkout \
  4a1def929e5b59c7a8122d19fce6efba581ce9f7
cp ~/workspace/vendor_ws/src/livox_ros_driver2/package_ROS2.xml \
  ~/workspace/vendor_ws/src/livox_ros_driver2/package.xml
```

编辑 `vendor_ws/src/livox_ros_driver2/config/MID360s_config.json`：`Mid360s.host_net_info[0].host_ip` 填本车雷达网卡 IP，`lidar_configs[0].ip` 填该雷达 IP。第一台当前运行的安装配置分别为 192.168.1.5 / 192.168.1.145；源码中的雷达 IP 是 192.168.1.45。两份不同，本次没有改动第三方或重新构建它。复刻第一台当前部署时填写 192.168.1.145；第二台必须按本雷达实际地址填写，不能照抄尾号。使用独立直连网络或避免多车同网段地址冲突。

```bash
cd ~/workspace/vendor_ws
colcon build --base-paths src/livox_ros_driver2 \
  --packages-select livox_ros_driver2 --symlink-install --executor sequential \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DBUILD_TESTING=OFF
source install/local_setup.bash
ros2 pkg prefix livox_ros_driver2
```

编译后核对 `install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json`，这是启动实际读取的文件。第一台旧构建为安装副本；不能假定编辑 src 即刻生效。当前 src 与 install 的 IP 差异在重建前必须确认，直接重建现有第一台可能把运行地址改为 .45。

显式选驱动，SDK 由前面的 CMake 安装；不用 colcon 将 SDK 当 ROS 包重复构建。该提交 CMake 直接安装 launch_ROS2，无需额外复制 launch 文件夹。官方 build.sh humble 会删除工作空间 build/devel/install；未来 vendor 合入 FAST-LIO 后尤其不能用它做日常增量构建。

## 6. FAST-LIO：仍在 conavGPT_ws

新干净 shell，先 source Humble，再加载 vendor 的 local_setup：

```bash
source ~/workspace/vendor_ws/install/local_setup.bash
mkdir -p ~/workspace/conavGPT_ws/src
git clone --branch ros2 https://github.com/Ericsii/FAST_LIO_ROS2.git \
  ~/workspace/conavGPT_ws/src/FAST_LIO_ROS2
git -C ~/workspace/conavGPT_ws/src/FAST_LIO_ROS2 checkout \
  2fffc570a25d0df172720bac034fbdb6a13d2162
git -C ~/workspace/conavGPT_ws/src/FAST_LIO_ROS2 submodule update --init --recursive
```

此版本 ikd-Tree 子模块为 e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4。核对 `config/mid360.yaml`：话题 /livox/lidar、/livox/imu，lidar_type=1、scan_rate=10，extrinsic_T/R 使用锁定上游值；将 **mapping.extrinsic_est_en 设为 false**。不要再复制已知旧的错误内部外参，也不要把底盘偏移写在这里。

```bash
cd ~/workspace/conavGPT_ws
colcon build --base-paths src --packages-select fast_lio --symlink-install \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/local_setup.bash
ros2 pkg prefix fast_lio
```

RViz 外观和 PCD/1 占位文件无需修改；当前启动管理关闭 RViz，降低附加负担。`lclivox` / `lclocal` 是第一台 .bashrc alias，不是复刻所需依赖，明确启动命令见 motion 的 OPERATIONS。

## 7. 自研 motion：固定版本与本车参数

新干净 shell，仅加载系统 Humble。公开仓库用 HTTPS 下载，无需先配置 GitHub 写权限：

```bash
git clone https://github.com/YOOJOR/wheeltec_motion_ws.git \
  ~/workspace/wheeltec_motion_ws
git -C ~/workspace/wheeltec_motion_ws checkout v0.3.0
cd ~/workspace/wheeltec_motion_ws
cp src/wheeltec_motion_control/config/motion_control.example.yaml \
  src/wheeltec_motion_control/config/motion_control.yaml
bash scripts/build_and_test.sh
cp docs/WORKSPACE_SETUP.md ~/workspace/README.md
```

模板禁用输出与外参确认。测量第二台车的安装姿态、参考点、平移（可用 CALIBRATION 离线录包工具辅助 XY），填写 YAML，再启用；第一台实车数值见 STAGE_SUMMARY，**不是通用默认值**。启动管理配置 `scripts/robot_stack/config.yaml` 的路径、话题也必须与本车一致。路径支持 ~，不依赖 alias。

图形桌面需要 gnome-terminal；首次启动前关闭其他手动节点，按 OPERATIONS 做静止检查，再由用户试车。启动脚本不自动发运动目标，不会自动重放中断动作。第二台的 Git 分支开发可从标签创建 `git switch -c robot-2`；不要将本车标定参数意外覆盖第一台。

第一台升级代码前先 `git status`、保存本车配置；有本地改动先审阅提交，不能用 reset --hard/clean 强行清理。SSH push 显示 SpiritGit 时意味着 SSH 密钥映射到了另一账号，需指定 YOOJOR 对应密钥；clone/只读 HTTPS 不受此影响。

## 8. 未来把 FAST-LIO 放入 vendor（尚未执行）

保留锁定提交和配置差异；在新目录组装 driver+FAST-LIO，先构建 driver，再加载其环境构建 fast_lio（或使用正确依赖的 colcon 拓扑构建）。SDK 保持外部 CMake 安装。重新生成 install，不能搬旧绝对路径链接。

迁移时修改 robot_stack/config.yaml 的 fastlio 路径、.bashrc/alias，移除旧空间的自动 source，确保 `ros2 pkg prefix fast_lio` 只解析新目录；再次检查位姿、恢复机制和控制接口。验证前保留原 conavGPT_ws 作为回退。这个迁移不属于 v0.3.0 的已完成事项。
