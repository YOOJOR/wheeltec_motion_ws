# WHEELTEC FAST-LIO 闭环控制工作空间

版本：0.1.0（Git 标签 v0.1.0）。ROS 2 Humble / Ubuntu 22.04 / Python 3.10。

已验证：两包编译成功；22 项测试全部通过；安装后的启动文件、CLI 和正常退出检查通过。详细结果见 logs/colcon-test-result.log。

本工作空间包含两个新增包：`wheeltec_motion_interfaces`（动作消息）和 `wheeltec_motion_control`（Python 控制器、客户端）。只复用现有 `/cmd_vel → turn_on_wheeltec_robot → STM32` 链路，不修改固件或厂商驱动。定位使用 FAST-LIO `/Odometry`，本包不发布 TF。

**当前为电脑上的开发版本。安装外参待测量，实车精度、速度正方向、制动效果和实际固件行为均待用户验证。** 不包含避障或全局规划；不保证进程强制终止/串口断开后的停车时间。

## 目录和版本记录

- `src/`：两个 ROS 包及测试。
- `src/wheeltec_motion_control/config/motion_control.yaml`：集中参数及单位说明。
- `docs/DEVELOPMENT_LOG.md`：开发决策、失败、修复、执行记录。
- `logs/`：纳入 Git 的编译和测试原始输出。
- `scripts/build_and_test.sh`：Humble 环境编译和测试入口。
- 本目录有独立 `.git`；在本目录使用 `git log --oneline`、`git status`。未配置远端，也未上传。
- `build/ install/ log/` 是可重建产物，不纳入 Git。容器产物不可直接用于实车，请在目标环境重新编译。

## 编译

在已有 ROS 2 Humble 的机器人/Ubuntu 环境，将整个工作空间复制过去，在本目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
```

需要 `rclpy`、`nav_msgs`、`geometry_msgs`、`std_srvs`、`rosidl_default_generators`、`ament_python`、`ament_cmake`、`launch_ros`、`python3-pytest` 等依赖，详见各包 package.xml。只构建本工作空间，避免父目录两个同名 `fast_lio` 包参与扫描。

Fedora 开发机可以使用已下载的官方容器镜像，从本目录执行：

```bash
podman run --rm --network=none --security-opt label=disable \
  -e ROS_DOMAIN_ID=173 -e ROS_LOCALHOST_ONLY=1 \
  -v "$PWD:/ws" -w /ws ros:humble-ros-base-jammy \
  bash scripts/build_and_test.sh
```

该命令只运行合成底盘测试，不启动串口或 FAST-LIO；镜像具体版本记录在日志中。测试话题为 `/motion_test/*`，没有向 `/cmd_vel` 输出的测试。

## 外参：先测量，再启用

FAST-LIO 的 `body` 是 IMU 坐标系；底盘参考点应与实际运动控制参考点一致（原驱动叫 `base_footprint`，不是随意选择的雷达中心）。本包内部按以下关系计算：

```text
T_world_base = T_world_body × T_body_base
```

- `body_from_base_translation = [tx, ty, tz]`：**底盘参考点在 IMU/body 坐标系下的位置**，单位 m。
- `body_from_base_rpy = [roll, pitch, yaw]`：将底盘坐标轴转到 IMU/body 坐标轴的旋转，单位 rad，使用 Rz(yaw) Ry(pitch) Rx(roll)。
- 如果测得的是 IMU 在底盘中的安装位姿 `T_base_body`，必须取逆：`R_body_base=R_base_bodyᵀ`，`t_body_base=-R_base_bodyᵀ t_base_body`；不能只对平移取负而忽略旋转。
- 这里是 IMU 与底盘之间的安装外参，不是 FAST-LIO 中雷达与 IMU 的 `extrinsic_T/R`。
- 转换使用完整三维位姿，之后才取底盘 `x,y,yaw`。不广播 `body → base_footprint`，因此不会新增第二条 TF 父链。

将参数文件复制为你自己的实车配置（例如 `robot_motion.yaml`），填入测量值，再设置：

```yaml
control_enabled: true
extrinsics_calibrated: true
```

默认两项为 false：未启用时不发布任何速度；未确认外参时拒绝动作。模拟测试中的零外参只对应合成底盘，不能照搬为实车标定结果。

## 实车启动与调用

1. 启动现有底盘通信（若已经运行，不能再启动第二份串口节点）：

```bash
ros2 launch turn_on_wheeltec_robot base_serial.launch.py
```

沿用实车已验证的车型和串口配置；资料默认 `mini_mec` 不代表实际 R680 配置。底盘自己的 `imu_mode` 不改变 FAST-LIO 使用 MID360S 内置 IMU 的选择。

2. 按已有方式启动 Livox 和 FAST-LIO。确认 `/Odometry` 的 frame 为 `camera_init`、child 为 `body`。多机部署要同步系统时间，否则时间戳检查会拒绝数据。
3. 关闭键盘控制及其他速度发布程序。在新终端加载 Humble 和本工作空间后：

```bash
ros2 launch wheeltec_motion_control motion_control.launch.py \
  params_file:=/绝对路径/robot_motion.yaml
```

启动文件只启动本控制器，不会替你启动底盘或雷达。启用后控制器空闲时持续发零速度，运行时应是唯一速度发布者。本初版采用操作上互斥，不包含多控制源仲裁器。

4. 在另一个已加载本工作空间的终端调用：

```bash
# 前进 0.2 m；负数表示后退
ros2 run wheeltec_motion_control motion_client move 0.2
ros2 run wheeltec_motion_control motion_client move -0.2 --max-speed 0.08
# 左转 15°（输入单位为 rad）；负数右转
ros2 run wheeltec_motion_control motion_client rotate 0.261799 --max-speed 0.2
# 请求停止当前动作
ros2 run wheeltec_motion_control motion_client stop
```

标准 ROS 工具也可使用：

```bash
ros2 action send_goal /wheeltec_motion/execute wheeltec_motion_interfaces/action/ExecuteMotion \
  '{motion_type: 1, target: 0.2, max_speed: 0.08, timeout_sec: 20.0}' --feedback
ros2 service call /wheeltec_motion/stop std_srvs/srv/Trigger '{}'
```

动作类型：1=直行，2=旋转。目标相对每次动作开始时的底盘位姿；旋转保持目标符号指定的方向，支持跨越 ±π 甚至一整圈。`max_speed=0`、`timeout_sec=0` 使用全局默认；单动作速度只能降低，不能突破全局上限。直行的速度覆盖限制 x 轴，横移纠偏和航向保持仍分别使用各自全局上限。超时可在 `max_timeout` 内调整。

一次只接受一个动作，忙时拒绝新普通动作。取消使用标准 Action cancel，stop 服务会令当前动作返回 STOPPED。客户端退出/断网不等于动作取消；强制杀死客户端后，控制器仍按原目标执行，需从另一终端调用 stop。

结果码：0 成功、1 取消、2 位姿失效、3 超时、4 无进展、5 主动停止、6 内部异常。客户端成功退出码为 0，拒绝/失败为 1，键盘中断为 130。停止服务确认的是请求已处理，并不是实测静止反馈。

## 调参

所有运动参数、误差阈值、超时、话题、frame、外参均在 YAML 中有说明。

- 先保留 `max_linear_speed=0.15 m/s`、`max_angular_speed=0.35 rad/s`，小距离、小角度确认方向。
- `distance_gain/rotation_gain` 控制接近速度，`heading_gain` 控制直行航向，`linear_accel/angular_accel` 控制正常速度变化。终点、停止、取消、故障直接发零速度，不经过斜坡。
- 初版 `lateral_correction=false`；此时保持航向并检测横向误差，不保证主动消除侧向漂移。基本运动验证后开启该项，使用麦轮 y 轴纠偏；若偏移超差且无法恢复，动作会失败或超时。
- 原地旋转只闭环控制航向，不主动把旋转产生的位置漂移拉回原点。
- `distance_tolerance/cross_track_tolerance/angle_tolerance` 是软件到达判据，不等同于真实定位精度。
- `settle_time` 和静止速度阈值用于确认连续新位姿中的稳定停止；相邻位姿差分速度受定位噪声影响，需要实测调整。
- `pose_timeout` 是位姿过期阈值，不是“停车时限”；定位失效时结束当前动作并发零速度，恢复后不会自动续跑。
- `no_progress_timeout` 防止持续输出却没有进展；定位跳变或时间戳不前进会结束动作。FAST-LIO 重启导致时钟/坐标重置后应重启控制器，再明确发送新动作。

参数可在空闲时在线调整，例如：

```bash
ros2 param set /wheeltec_motion_controller max_linear_speed 0.10
ros2 param set /wheeltec_motion_controller rotation_gain 1.0
```

在线更改会清除位姿缓存，等待新数据。动作执行中拒绝调参，先停止。话题/服务名称、控制与反馈频率、`use_sim_time` 需要重启；重启恢复文件值，在线调整不会自动写回 YAML。长期参数请保存到你的配置并纳入版本控制。

## 验证范围

电脑测试覆盖几何外参、前后直行、转向跨界/多圈方向、横移纠偏、限速和加速度、过冲、位姿丢失/跳变/重复时间戳、无进展、超时、停止和取消；ROS 集成测试使用真实 Humble Action/Topic/Service 和合成底盘。

实车由用户完成：测外参；0.2 m/15°确认方向；1 m、左右90°各5次记录误差；再测动作序列。建议工程目标为位置误差 ≤10 cm、转角误差 ≤5°，尚未实测。保留定位、cmd_vel、动作结果和外部测量，不以 FAST-LIO 自身输出作为唯一真值。
