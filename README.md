# WHEELTEC FAST-LIO 闭环控制工作空间

版本：0.2.0（Git 标签 v0.2.0）。ROS 2 Humble / Ubuntu 22.04 / Python 3.10。

已验证：两包编译成功；29 项测试全部通过；安装后的启动文件、CLI 和正常退出检查通过。详细结果见 logs/calibration-test-result.log。

本工作空间包含两个新增包：`wheeltec_motion_interfaces`（动作消息）和 `wheeltec_motion_control`（Python 控制器、客户端）。只复用现有 `/cmd_vel → turn_on_wheeltec_robot → STM32` 链路，不修改固件或厂商驱动。定位使用 FAST-LIO `/Odometry`，本包不发布 TF。

**当前为初步实车验证版本。2026-09-25 用户已填写本车外参并完成一次低速直行动作；重复精度、转向和长期稳定性仍待验证。仓库默认 YAML 仍为禁用输出、零外参，不能直接作为实车配置。** 不包含避障或全局规划；不保证进程强制终止/串口断开后的停车时间。

## 阅读路线

- 第一次使用：阅读「控制链路与两个包」「编译」「外参」「实车启动与调用」。
- 已能运行：阅读「怎样读反馈和结果」「参数如何生效」。
- 出问题：从「独立调试：先判断故障在哪一层」按现象查找，不必执行全部命令。
- 标定工具单独见 [docs/CALIBRATION.md](docs/CALIBRATION.md)；开发记录见 [docs/DEVELOPMENT_LOG.md](docs/DEVELOPMENT_LOG.md)。

文中实车命令在**小车终端**执行，示例控制工作空间为 `~/workspace/wheeltec_motion_ws`。开发电脑上的 `/mnt/wheeltec_motion_ws` 是 NFS 文件视图，不代表电脑拥有小车正在运行的 ROS 节点或环境。读到 YAML 也不能证明运行中的节点已加载它。

## 控制链路与两个包

```text
上层程序 / motion_client
  └─ 一次 Action 目标 → /wheeltec_motion/execute
                           ↓
FAST-LIO /Odometry → motion_controller → /cmd_vel（Twist，默认 20 Hz）
                           ↑                       ↓
                    位置/朝向反馈         turn_on_wheeltec_robot
                                                   ↓ 串口
                                                 STM32 → 车轮
```

- `wheeltec_motion_interfaces` 定义 `ExecuteMotion.action` 的目标、反馈和结果格式，不驱动车轮。
- `wheeltec_motion_control` 包含 Action 服务端、命令行客户端和标定工具。运行节点名为 `/wheeltec_motion_controller`。
- 其他 ROS 2 Python/C++ 程序可创建该接口的 Action 客户端，调用 `/wheeltec_motion/execute`；需要接口包、可互相发现的 ROS 环境和相同通信域。请求发给运行节点，不是发给包目录。
- 用户的距离/角度目标只发送一次；控制器根据最新有效位姿，默认每 50 ms 重算速度，直到成功或终止。位置输入约 10 Hz、速度输出 20 Hz、客户端反馈 5 Hz 是不同频率。
- `/cmd_vel` 的 `linear.x` 是前后速度（m/s），`linear.y` 是横向速度（m/s），`angular.z` 是转向速度（rad/s）。底盘驱动将它编码为串口帧；本包不打开串口，也不直接设置电机 PWM。
- 直行采用剩余距离比例控制、限速、正常加速度限制，并保持起始航向。默认不主动横移纠偏；旋转只控制相对转角。进入到达范围后发零速度，持续检查位置及估计速度，满足稳定条件才成功。
- 启用后，即使空闲也持续发布零速度。因此不要同时运行键盘、导航或另一个 motion 控制器向同一话题发速度。

## 目录和版本记录

- `src/`：两个 ROS 包及测试。
- `src/wheeltec_motion_control/config/motion_control.yaml`：集中参数及单位说明。
- `docs/DEVELOPMENT_LOG.md`：开发决策、失败、修复、执行记录。
- `logs/`：纳入 Git 的编译和测试原始输出。
- `scripts/build_and_test.sh`：Humble 环境编译和测试入口。
- 本目录有独立 `.git`；在本目录使用 `git log --oneline`、`git status`。已配置 GitHub 远端；用 `git remote -v` 查看地址。提交与推送是两步，不能把本地提交当成已同步到小车。
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
podman build -t wheeltec-motion-test:humble -f scripts/Containerfile scripts

podman run --rm --network=none --security-opt label=disable \
  -e ROS_DOMAIN_ID=173 -e ROS_LOCALHOST_ONLY=1 \
  -v "$PWD:/ws" -w /ws wheeltec-motion-test:humble \
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
ros2 launch turn_on_wheeltec_robot base_serial.launch.py car_mode:=senior_mec_bs
```

本车车型为 `senior_mec_bs`；沿用实车已验证的串口和 IMU 配置，不要照搬资料中的 `mini_mec` 默认车型。底盘自己的 `imu_mode` 不改变 FAST-LIO 使用 MID360S 内置 IMU 的选择。

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
ros2 run wheeltec_motion_control motion_client move 0.2 --max-speed 0.05
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

实车验证由用户安排，不是启动后自动执行的步骤：先确认外参与方向，再按场地条件验证直行、转向及重复性。建议工程目标为位置误差 ≤10 cm、转角误差 ≤5°，尚未实测。保留定位、cmd_vel、动作结果和外部测量，不以 FAST-LIO 自身输出作为唯一真值。

## 离线旋转外参辅助标定

新增 `calibrate_rotation`：记录 FAST-LIO 位姿后离线计算水平偏移，输出左右转交叉验证、轨迹图和残差；不自动改配置，高度需单独测量。完整命令与结果解释见 [标定工具说明](docs/CALIBRATION.md)。


## 每次启动：确认加载的是哪份配置

以下命令以直接使用源码中的配置为例。先自行填写真实外参，确认需要启用，再设置两个开关；不要只为消除报错而把标定开关设为 true。私有实车 YAML 也可以使用，只需替换 `params_file`。

每个用于控制器/客户端的新终端都加载：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/wheeltec_motion_ws/install/setup.bash
ros2 pkg prefix wheeltec_motion_control
```

最后一条应指向这份工作空间的安装目录。如果指向旧工作空间，先纠正环境。启动厂家驱动、Livox、FAST-LIO 的终端还需加载它们各自的安装环境，沿用已验证的启动方式；本控制工作空间不包含这些包。

控制器终端（确认没有另一份控制器后执行）：

```bash
cd ~/workspace/wheeltec_motion_ws
mkdir -p runtime_logs
motion_run_id=$(date +%Y%m%d-%H%M%S)
ros2 launch wheeltec_motion_control motion_control.launch.py \
  params_file:="$PWD/src/wheeltec_motion_control/config/motion_control.yaml" \
  2>&1 | tee "runtime_logs/controller-${motion_run_id}.log"
```

这是启动命令，会在已启用时持续发零速度；不是纯检查命令。终端保持运行。启动日志中实车启用状态应为：

```text
Motion server ready; output enabled=True; extrinsics calibrated=True.
```

配置文件需有正确的节点名和层级（下面仅演示开关结构，外参等仍需保留）：

```yaml
wheeltec_motion_controller:
  ros__parameters:
    control_enabled: true
    extrinsics_calibrated: true
```

在客户端终端检查实际值，**运行值比磁盘上的 YAML 更能说明当前状态**：

```bash
ros2 param get /wheeltec_motion_controller control_enabled
ros2 param get /wheeltec_motion_controller extrinsics_calibrated
ros2 param get /wheeltec_motion_controller body_from_base_translation
ros2 param get /wheeltec_motion_controller use_sim_time
ros2 param get /wheeltec_motion_controller pose_timeout
ros2 action list -t
```

本车正常实车模式 `use_sim_time=False`，当前 `pose_timeout=0.5`。外参应与你确认的值一致。看到 `True/True` 仅表示开关启用，不证明定位健康、外参准确或底盘串口连接成功。

## 怎样读反馈和结果

以 `move 0.2 --max-speed 0.05` 为例，目标是相对起点、沿起始车头方向前进 0.2 m；它不是前往世界坐标 x=0.2，也不是固定发 4 秒速度。

| 输出 | 含义及判断方法 |
| --- | --- |
| `RUNNING` | 正在闭环计算速度；不保证此刻车轮确实在动 |
| `remaining` | 有符号剩余量；直行为 m，旋转为 rad。直行按起始朝向投影计算，并非二维直线距离 |
| `cross` | 直行时相对起始直线的横向偏差，m；不是航向角。旋转动作不以此作为位置保持指标 |
| `SETTLING` | 已进入到达容差，发零速度并等待稳定；如果又离开范围会返回 RUNNING |
| `elapsed` | 从动作开始累计秒数 |
| 结果 `pose: [x,y,yaw]` | 换算后的底盘参考点在配置世界坐标系中的最终位姿，单位 m、m、rad；不是本次位移 |
| `code: 0` | 满足软件到达和稳定条件，不等于外部尺测零误差 |

默认直行到达条件是：前后误差 ≤3 cm、横向误差 ≤5 cm、朝向误差 ≤0.035 rad（约 2°）。随后相邻定位估算的线速度 ≤0.025 m/s、角速度 ≤0.04 rad/s，并在新位姿持续到来时满足稳定条件至少 0.4 s。定位抖动会使稳定等待延长；这是估计静止，不是轮速反馈或物理制动保证。

2026-09-25 用户提供的一次实测输出：目标 0.2 m，最终 `remaining=0.0209739`，耗时约 6.50 s、返回成功。表示按 FAST-LIO 反馈沿目标方向前进约 0.179 m，符合 3 cm 容差；不是已经达到 20 cm 的精确尺测结果。该次前约 1.5 s 位移很小，起步死区、底盘响应等原因未证实。

## 参数如何生效，以及该调什么

编辑 YAML **不会让已运行的节点自动重载**。保存长期参数后重启控制器；若使用复制安装，配置变更需要重新构建，或用 `params_file` 直接指定修改后的源码文件。在线 `ros2 param set` 只改当前进程，重启后丢失。

| 目标/现象 | 相关参数 | 调整前先确认 |
| --- | --- | --- |
| 单次低速执行 | CLI `--max-speed` | 直行单位 m/s、旋转 rad/s；0 使用默认，不能超过全局上限 |
| 全局速度上限 | `max_linear_speed`、`max_angular_speed` | 单次限速不修改全局值；旋转速度与角度不能混淆 |
| 起步/普通速度变化更缓慢 | `linear_accel`、`angular_accel` | 减小加速度会延长起步；主动停止、故障和到达时直接发零 |
| 靠近目标时的速度 | `distance_gain`、`rotation_gain` | 先确认定位延迟与底盘低速响应；增大增益可能增加过冲 |
| 直行车头偏转 | `heading_gain` | 核对轴向、转向符号和外参；不要靠增益补偿错误坐标 |
| 横向偏差 | `lateral_correction`、`lateral_gain` | 默认 false，只监测偏差并保持航向；开启需另行验证麦轮横移方向 |
| 距目标还有几厘米就成功 | `distance_tolerance` | 当前默认 3 cm 是主动设计；收紧前先确认定位噪声、外部测量和低速能力 |
| 一直 SETTLING | `settle_time`、`stopped_*` | 查看定位是否抖动、车是否缓慢滑移，不先放宽阈值 |
| 位姿旧或中断 | `pose_timeout` | 先测实际延迟/更新情况。不是停车时限，不是用于掩盖秒级积压 |
| 无进展或超时 | `no_progress_timeout`、`default_timeout`、CLI `--timeout` | 先确定底盘是否收到非零速度、有无运动，而非直接延长等待 |

一次改一个参数，记录旧值、新值、运行配置和结果。只有检查、参数读取不会导致运动；本 README 中 `move/rotate/send_goal` 示例会实际请求动作，不要作为查看状态的命令执行。

## 独立调试：先判断故障在哪一层

先保留报错和控制器终端输出，再按以下顺序查。ROS 检查命令也需要在正确环境中执行；出现 `Package not found` 或消息类型无法加载，优先检查 `source` 和编译安装，不应先改控制算法。

### A. 客户端连接不上：`action server unavailable`

```bash
ros2 node list
ros2 action list -t
ros2 action info /wheeltec_motion/execute
ros2 pkg prefix wheeltec_motion_control
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION
```

应能看到控制节点和 Action 服务端。看不到时依次检查：控制器是否仍在运行、是否加载正确工作空间、是否修改了 Action 名称、两个终端的 ROS 通信环境是否一致。未设置的环境变量可能不输出，不等于报错。跨机器还需网络可达；本地 NFS 文件可访问不代表 ROS 通信可达。若出现重复控制节点或多个 Action 服务端，先关闭重复实例。

### B. `goal rejected; see controller log`

客户端这一句没有完整原因，去控制器终端或保存的 `runtime_logs/controller-*.log` 找同一时刻的 `Goal rejected: ...`。

| 控制器原因 | 怎样处理 |
| --- | --- |
| `control_enabled is false` / `extrinsics_calibrated is false` | 用上文 `ros2 param get` 核对运行值；检查 YAML 节点名、params_file、旧进程；外参未确认则先完成确认 |
| `no pose received` / `pose is stale` | 检查 `/Odometry` 是否发布、控制器订阅的话题是否正确；再看 C 节 |
| `odometry source timestamp is stale` | 消息时间戳比当前 ROS 时间旧得过多；按 C 节测时间差，不靠提高速度或改外参解决 |
| `odometry source timestamp is in the future` | 检查多机时钟、实车 use_sim_time 和上游时间戳，不要直接改成当前时间掩盖问题 |
| `unexpected odometry world frame` / `unexpected odometry child frame` | 实际消息与 world_frame/body_frame 不匹配；核实物理坐标含义后再统一配置 |
| `controller busy or shutting down` | 当前有动作或正在退出；等待完成或明确 stop，不会自动排队 |
| `goal speed exceeds configured limit` | 减小单次 --max-speed，或在明确需要时调整全局限速 |
| `target exceeds configured bound` / `goal timeout exceeds configured maximum` | 核对单位、max_distance/max_rotation/max_timeout，特别注意角度使用 rad |

### C. 定位新鲜度：区分“来得勤”和“来得及时”

以下连续运行命令每项观察约 10～15 秒后 Ctrl+C，尽量不要同时开多个点云测量。

```bash
ros2 topic info /Odometry -v
ros2 topic echo /Odometry --once --field header
ros2 topic echo /Odometry --once --field child_frame_id
ros2 topic hz /Odometry
ros2 topic delay /Odometry
```

预期坐标名为 `camera_init` / `body`；用 topic info 核对发布者、订阅者与 QoS。`hz` 统计该工具收到消息的频率，`delay` 比较接收时间与消息时间戳。10 Hz 也可以持续落后 1 秒，频率正常不等于定位可用于控制。检查时短暂出现“尚未发现话题”，随后持续有数据，不能单凭初始警告判断节点故障。

本控制器同时检查：①消息源时间戳年龄；②距上次有效位姿到达的时间。默认两者都受 0.5 s 门槛约束。`/Odometry` 的 twist 全零不妨碍本控制器运行，它使用 pose，并用相邻 pose 估算停止状态。

如果 Odometry 时间差明显过大，再分段测：

```bash
ros2 topic delay /livox/imu
ros2 topic delay /livox/lidar
```

`/livox/lidar` 使用自定义消息时，终端须加载实际 Livox 驱动工作空间的 install/setup.bash。路径以小车实际安装位置为准，不是控制工作空间。点云大，Python 命令行测量本身可能跟不上，测得的延迟和频率不能直接作为驱动内部耗时。驱动点云时间戳为帧起始，FAST-LIO 输出为扫描结束；不同阶段、不同时间段的平均数不能直接相减当作处理耗时。

| 观察 | 下一步 |
| --- | --- |
| IMU 和点云都明显旧 | 核对驱动时间基准、主机时钟/同步、传输和公共上游延迟 |
| IMU 新鲜，点云明显旧 | 排查组帧、点云传输/队列、驱动负载，同时考虑测量工具开销 |
| 原始输入新鲜，Odometry 旧 | 优先排查 FAST-LIO 接收队列、计算耗时与等待 IMU 同步 |
| 只有 RViz/录包/点云测量开启后变差 | 逐个停止非必需负载做对照，观察是否恢复；不要一次改多个配置 |

查看资源占用（输出完整保存，不用 head 截掉后续采样）：

```bash
mkdir -p ~/workspace/wheeltec_motion_ws/runtime_logs
top -b -d 2 -n 3 -w 160 > ~/workspace/wheeltec_motion_ws/runtime_logs/top.txt
```

看 CPU 空闲、I/O 等待、可用内存及各进程；总 CPU 空闲不能排除单线程瓶颈，第一屏也不代表持续负载。不要仅凭这些数据就增大 pose_timeout、开启 time_sync_en 或修改雷达/IMU 时间偏移。

**需要用重启诊断积压时：**先停止当前动作，确认车静止，退出控制器，再只重启 FAST-LIO 并保持驱动运行做对照。重启会重置定位原点、重建当前局部地图，需要的地图应提前保存。观察从启动开始的延迟：先小后增长支持积压方向；一开始就很大则继续查输入、传输及时间。确认定位正常后重新启动控制器，不续接旧动作。不要在运动中用重启测试，也不要自动重试被拒绝的动作。

2026-09-25 案例（用户提供的运行输出）：

- 控制器开关 True/True、use_sim_time=False、pose_timeout=0.5，但 Odometry 平均约 1.4 s，因此拒绝动作。
- 分段观测 IMU 约 2 ms、点云命令行测量约 0.7 s；停止额外测量/可视化后的 Odometry 仍约 1.4 s。资源快照无整机持续耗尽的直接证据。
- 用户报告重新运行后 Odometry 平均 31 ms、最大 59 ms，随后低速直行动作成功。具体重启组件组合未记录清楚，不能据此断言只重启某一个节点就一定解决。
- 结论：本次数据过期的保护按预期生效；重启后恢复。**积压具体位置和根因未确认，不能视为永久修复。** 若复现，应先保存日志与配置，再做单变量对照。

### D. 动作已接受，但车不动或走走停停

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /cmd_vel
```

只观察，不使用 `topic pub` 绕过控制器试车。检查速度发布者是否只有预期控制器、底盘是否订阅该话题。空闲/SETTLING/结束时零速度是正常的；RUNNING 也可能因误差或加速限制给出很小速度。

- 没有非零速度：检查控制器是否仍有动作、是否已终止、是否处于 SETTLING，以及发布话题配置。
- 非零速度与零速度交替：检查键盘、导航、重复控制器等多个发布者；启用但空闲的本控制器也会持续发零。
- 持续非零速度但车不动：查底盘驱动终端的串口错误、供电/使能状态、车型 `senior_mec_bs`、串口配置、底盘自身限制及低速死区。收到 Twist 不等于 STM32 已执行，先定位哪一层未响应。
- 车实际在动但 remaining 不变：看 FAST-LIO 位姿是否跟随、坐标轴和外参是否正确；不要只延长 no_progress_timeout。
- remaining 变小但 cross 增大：默认没有横移纠偏；核对安装轴向和实测轨迹，不要仅凭成功码认定路径精确。

### E. 已开始，后来失败或一直不能完成

| 结果码 | 含义 | 先查什么 |
| --- | --- | --- |
| 0 | 成功 | 结合实际尺测/角度测量判断精度，保留剩余量 |
| 1 | 取消 | 是否由 Action 客户端请求 cancel |
| 2 | 位姿失效 | message 中是否源时间旧、流中断、时间戳不前进、位置/角度跳变；重启定位后重启控制器 |
| 3 | 动作超时 | 是否确实太慢、到达容差过紧、横向误差未恢复或稳定判据长期不满足 |
| 4 | 无可测进展 | 底盘不动、低速死区、定位不更新，或误差未持续改善 |
| 5 | 主动停止 | stop 服务请求已处理 |
| 6 | 内部异常 | 查看控制器日志；`control loop timing gap` 表示控制调度间隔超过默认 0.3 s，检查负载/阻塞 |

`goal rejected` 发生在接单前，没有上述完整动作结果；不能把客户端进程退出码 1 当成 Action 的“取消码 1”。`--server-wait` 是客户端等待服务器/响应的时间，`--timeout` 是动作执行时间，两者也不同。

停止使用 `motion_client stop` 或 stop 服务。客户端普通 Ctrl+C 在已经拿到有效目标句柄时会尝试取消；断网、强制 kill、尚未取得句柄时不能依靠它停止。服务返回成功表示处理了请求，不证明车轮已经物理静止。故障动作不会因定位恢复而自动续跑。

## 留存一次可复查的调试记录

控制器启动时使用上文 tee 保存运行输出。`log/` 通常是 colcon 编译/测试日志，**不是当前 ROS 控制器的运行日志**；ROS 默认运行日志在 `~/.ros/log`，可通过启动终端显示的路径定位。本工作空间的 `runtime_logs/` 便于通过 NFS 查看，避免混淆历史测试与当前实车错误。

在已启动节点的客户端终端保存快照：

```bash
cd ~/workspace/wheeltec_motion_ws
motion_capture_id=$(date +%Y%m%d-%H%M%S)
mkdir -p "runtime_logs/$motion_capture_id"
ros2 param dump /wheeltec_motion_controller > "runtime_logs/$motion_capture_id/controller-params.yaml"
ros2 topic info /Odometry -v > "runtime_logs/$motion_capture_id/odometry-info.txt"
ros2 topic info /cmd_vel -v > "runtime_logs/$motion_capture_id/cmd-vel-info.txt"
git rev-parse HEAD > "runtime_logs/$motion_capture_id/commit.txt"
git status --short > "runtime_logs/$motion_capture_id/git-status.txt"
```

把客户端输出、延迟测量、实际现象和启动命令放在同一目录；记录“改了什么、是否重启了哪些节点、重启前后分别是什么结果”。需要录包时可另用 `ros2 bag record -o <尚不存在的输出目录> /Odometry /cmd_vel`，Ctrl+C 正常结束；该包不含完整 Action 结果，仍需客户端日志。录包也增加负载，诊断延迟时需记录是否开启。

原始 runtime_logs 默认不进 Git，避免大量实车记录自动入库；有价值的结论整理到 docs，必要的原始证据另行选择归档。开发和参数长期变更应在本独立仓库提交：先 `git diff` 审核，按文件 `git add`，再 `git commit`；用 `git log --oneline -5` 查看版本。小车与开发机各有本地修改时，先核对 `git status`，不要用强制重置覆盖实车参数。更新代码后在小车重新编译、重新加载环境并重启相关节点；Git 同步文件不会自动更新运行进程。


## 同时测量 IMU、点云帧起止与位姿延迟

若 1 秒以上的位姿延迟复现，先保持机器人静止和现有定位进程运行，再采样，避免立即重启丢失异常状态。此脚本仅订阅，不发送动作：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/vendor_ws/install/setup.bash
mkdir -p ~/workspace/wheeltec_motion_ws/runtime_logs
python3 ~/workspace/wheeltec_motion_ws/scripts/diagnose_pose_delay.py --seconds 15 \
  | tee ~/workspace/wheeltec_motion_ws/runtime_logs/delay-probe.log
```

`imu/odom age` 是消息时间戳到接收时刻的秒数；`lidar age` 是点云帧头年龄，`end_age` 是末点年龄，`span` 是首末点间隔。10 Hz 组帧约 0.1 s，因此帧头比末点旧约 0.1 s 是正常现象，不能把这段采样窗口都当作处理延迟。每行 `n` 是该采样窗口收到的数量。

脚本用 raw 订阅避免逐点解码；只支持此项目标准 Livox CustomMsg 的 CDR 布局，并在采样结束与一帧 ROS 反序列化进行核对。它测的是本诊断订阅端，不能单独给出 FAST-LIO 内部队列长度或控制器回调耗时。全部 no data 时检查进程、环境和 ROS_DOMAIN_ID。最终 PASS 只证明消息解析正确，不代表小车运动精度。

2026-09-26 SSH 对照：用户重启后的位姿延迟约 40 ms，标准工具也约 42 ms；异常时约 1.4 s 的运行状态已结束，根因仍未确认。保留异常时与正常时的采样才能继续定位。已归档输出见 logs/20260926-delay。

## 一键启动通信、雷达、定位与控制接口

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_robot.sh
```

等待 `STACK READY` 后发送现有 Action 请求即可。默认不启动 RViz；脚本不发送动作。退出/挂起、数据持续中断或位姿持续过期时按依赖关系自动恢复，恢复后只接收新目标，不续跑旧动作。Ctrl+C 退出全部受管进程。

开始前退出原来手动启动的四个 launch，脚本会拒绝重复节点。仅检查配置用 `bash scripts/start_robot.sh --check`。详细就绪条件、恢复规则、日志、后台停止方式及可调参数见 [统一启动说明](scripts/robot_stack/README.md)。

## 可选：四个终端标签页管理

在小车图形桌面执行 `bash scripts/start_robot_tabs.sh`，新增窗口的四个标签页分别运行通信、Livox、FAST-LIO、控制接口，在原页内自动恢复。原版 `start_robot.sh` 保留供对比，两者不能同时运行。

`bash scripts/start_robot_tabs.sh --status` 查看状态；`--stop` 停止新版。在任一受管标签页按 Ctrl+C 或关闭该页会停止整套服务。详细切换方式、日志及使用说明见 [标签页版本](scripts/robot_stack/TABS.md)。
