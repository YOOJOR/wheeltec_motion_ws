# 使用与调试

当前上下文见 STAGE_SUMMARY。所有命令在小车 Ubuntu 22.04 / Humble 上运行；开发机没有原生 ROS。`motion_control.yaml` 是第一台车已启用输出的配置，新车先使用禁用的 example。

地图/探索 RViz 已有预设入口：`bash scripts/start_debug_rviz.sh`。自动配置俯视地图、前沿/短期目标、IMU 位姿/轨迹和奥比原始 RGB/深度；只打开可视化，不启动设备或运动。四终端启动顺序、颜色与坐标含义、故障排查见 [VISUALIZATION](VISUALIZATION.md)。

## 当前 Co-NavGPT2 探索入口

当前默认 ASTRA S，采用随机可达前沿，不接 VLM。新增程序为只读预览，不启动设备、不发布速度：

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_exploration_preview.sh --check
bash scripts/start_exploration_preview.sh --demo --steps 300
```

以上可在不进行实车调试时执行。真实相机/定位已由用户启动后，使用 `--start --duration 60` 查看真实数据预览；静止投影、地图检查和故障 reset 的完整步骤见 [EXPLORATION_PREVIEW](EXPLORATION_PREVIEW.md)。相机外参暂定未确认，预览没有自动导航模式启用功能。下面的运动命令属于既有控制操作，不能用来替代探索验收。

## 启动与停止

三个工作空间按 WORKSPACE_SETUP 已编译后，在小车桌面终端：

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_robot_tabs.sh --check
bash scripts/start_robot_tabs.sh
```

等待控制接口页显示就绪。四页各自执行 ROS launch 并实时输出；出错按依赖自动重启。启动入口返回后四页继续运行，重复启动只提示已运行。

```bash
bash scripts/start_robot_tabs.sh --status
bash scripts/start_robot_tabs.sh --stop
```

任一受管标签 Ctrl+C / 关闭该页停止整套。发控制指令、调试另开普通终端。SSH 没有图形会话时只能查看/停止标签版；启动前台版用 `bash scripts/start_robot.sh`，Ctrl+C 停全部。两版共用锁，不可同时运行；切换前停止原版并等待节点退出。

后台原版停止：

```bash
stack_pid=$(cat ~/workspace/wheeltec_motion_ws/runtime_logs/robot_stack/supervisor.lock)
ps -p "$stack_pid" -o pid,args
# 确认是本项目 robot_stack/supervisor.py 后再执行：
kill -INT "$stack_pid"
```

恢复规则：通信故障保留定位地图；Livox 故障重启驱动和定位；FAST-LIO 故障重新定位；motion 故障只重启接口。上游异常先撤下 motion，持续 3 s 后重试；启动未就绪等 45 s；退避 3–30 s。重启不续跑旧动作，定位重启后地图原点会变。

### 手动分步启动（排查时使用）

先退出受管脚本和旧手动节点，四个独立终端分别运行：

```bash
# 通信
source /opt/ros/humble/setup.bash
source ~/workspace/wheeltec_base_ws/install/local_setup.bash
ros2 launch turn_on_wheeltec_robot base_serial.launch.py
```

```bash
# 雷达；当前使用 MID360S，不是 MID360 的启动文件
source /opt/ros/humble/setup.bash
source ~/workspace/vendor_ws/install/local_setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

```bash
# 定位；vendor 是 FAST-LIO 编译/运行依赖
source /opt/ros/humble/setup.bash
source ~/workspace/vendor_ws/install/local_setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
```

```bash
# 控制接口；明确使用源码中的当前 YAML
source /opt/ros/humble/setup.bash
source ~/workspace/wheeltec_motion_ws/install/local_setup.bash
ros2 launch wheeltec_motion_control motion_control.launch.py \
  params_file:=$HOME/workspace/wheeltec_motion_ws/src/wheeltec_motion_control/config/motion_control.yaml
```

ROS 环境加载有叠加性：source 不会清除已有旧路径。包解析错误时使用新终端/干净 shell，或优先使用会清理子环境的启动脚本。

## 调用动作与接口

普通客户端终端只需 source Humble 与 motion：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/wheeltec_motion_ws/install/local_setup.bash
ros2 run wheeltec_motion_control motion_client move 0.2 --max-speed 0.05
ros2 run wheeltec_motion_control motion_client move -0.2 --max-speed 0.05
ros2 run wheeltec_motion_control motion_client rotate 1.570796 --max-speed 0.1
ros2 run wheeltec_motion_control motion_client rotate -1.570796 --max-speed 0.1
ros2 run wheeltec_motion_control motion_client stop
```

这些是独立示例；不要在上一动作未完成时并行发送。move 单位 m，rotate 单位 rad（90°≈1.570796）；正号前进/逆时针左转。`--timeout 60` 设置单动作超时，`--server-wait 10` 设置客户端等待接口时间，不会改变控制器位姿门槛。速度超过直行 0.15 / 转向 0.35 会被拒绝。

原生 ROS Action 调用（也会实际运动）：

```bash
ros2 action send_goal /wheeltec_motion/execute wheeltec_motion_interfaces/action/ExecuteMotion \
  '{motion_type: 1, target: 0.2, max_speed: 0.05, timeout_sec: 45.0}' --feedback
ros2 service call /wheeltec_motion/stop std_srvs/srv/Trigger '{}'
```

其他程序使用 ActionClient 加载 ExecuteMotion；请求字段/结果码见 STAGE_SUMMARY 及 action 文件。先检查 accepted，再等待结果 code=0 才继续。客户端 Ctrl+C 尝试 cancel；直接终止客户端并不能代替明确的 stop。

RUNNING=闭环输出；SETTLING=进入容差、请求零速度并观察是否稳定。直行 remaining 为 m，转向为 rad（角度=rad×180/π），cross 为横向偏差。返回成功只表示当前容差与稳定判据通过，最终 pose 是 camera_init 中的底盘坐标，不是相对移动量。

## 持续速度导航（Co-NavGPT2 接入）

完整坐标/时间戳契约、模式表、Python 适配示例和失效处理见 [NAVIGATION.md](NAVIGATION.md)。本接口负责速度请求与失效停止，路径规划和方向纠偏由上层完成；没有实现闭环路径跟踪。

```bash
ros2 run wheeltec_motion_control navigation_client status
ros2 run wheeltec_motion_control navigation_client enable
# 上层只向 /wheeltec_motion/navigation/cmd_vel 发布新的 TwistStamped；不要直接发 /cmd_vel。
ros2 run wheeltec_motion_control navigation_client disable
ros2 run wheeltec_motion_control navigation_client stop
```

启用等待新请求时为零；Action 与导航互斥。默认底盘控制 frame=`wheeltec_base`，只支持 linear.x / angular.z。本车速度上限仍为 0.15 / 0.35，示例用 0.12 / ±0.30，不使用上游 0.3 / ±0.5。导航请求默认双重超时 0.3 s；正常每约 0.1 s 处理新观测更新，不能定时重发旧决策掩盖卡顿。进入阻塞 VLM/长等待前 stop；超时、定位异常、停止或退出后须显式启用并发新请求，不自动恢复缓存。MotionStatus 为软件模式/输出状态，不是物理停车确认。

## 配置及生效检查

控制参数在 `src/wheeltec_motion_control/config/motion_control.yaml`；新车模板 example。路径/四条启动命令/恢复时间在 `scripts/robot_stack/config.yaml`。

```bash
ros2 param get /wheeltec_motion_controller control_enabled
ros2 param get /wheeltec_motion_controller extrinsics_calibrated
ros2 param get /wheeltec_motion_controller body_from_base_translation
ros2 param get /wheeltec_motion_controller pose_timeout
ros2 param get /wheeltec_motion_controller navigation_command_timeout
ros2 param get /wheeltec_motion_controller max_linear_speed
ros2 param get /wheeltec_motion_controller max_angular_speed
ros2 param get /wheeltec_motion_controller use_sim_time
```

通过 source + 单独 launch 时默认读取 install 配置；管理脚本明确读取源码配置。修改源码并不等于所有现有节点都已重载；停止并重新启动、再查询参数。普通脚本/源码 YAML 更新一般无需 colcon，但源码/接口或安装内容更改后需重新编译。

空闲时部分数值参数可以 `ros2 param set`，变化后等待新位姿；Action 或导航模式中参数更新会拒绝，先 stop/退出。topic/Action/service 名称、navigation_frame、control_rate_hz、feedback_rate_hz、use_sim_time 需要节点重启。navigation_command_timeout 独立于 pose_timeout，新参数也已加入新车 example；稳定配置应写回 YAML 并提交，不只保留临时 param set。

## 当前响应参数试调与回退

2026-09-28 第一档试调（dc6a68d）已部署，仅应用于第一台车的 motion_control.yaml；最高速度、到达容差、静止速度阈值、定位门槛与频率不变。example 保留原保守值。尚未收到本档实车运动反馈，参数更积极不代表底盘实际延迟已消除。

| 参数 | 保守值 | 当前试调值 |
| --- | --- | --- |
| linear_accel | 0.15 | 0.30 m/s² |
| angular_accel | 0.5 | 1.0 rad/s² |
| distance_gain | 0.8 | 1.2 s⁻¹ |
| rotation_gain | 1.2 | 1.6 s⁻¹ |
| settle_time | 0.4 | 0.25 s |

先用相同的动作目标和 --max-speed 对比起步/收尾时间；再单独提高请求速度，避免把速度变化误认为调参效果。观察是否增加过冲、来回修正或停稳等待。反馈打印频率仍为 5 Hz，SETTLING 输出零速度。

分别记录实际运动耗时与 SETTLING 等待。当前停稳速度由相邻位姿差估计：两帧间隔 0.1 s、位置抖动 3 mm 会得到 0.03 m/s，高于 0.025 m/s 静止阈值，可能重置等待。这是计算示例，本车噪声幅度尚未测量；缩短 settle_time 没有改变这一算法。持续反馈也不表示还在要求移动。

本车旧配置备份：runtime_logs/tuning/20260928-responsive-1/motion_control.before.yaml。保守版本也保存在 Git 提交 ff29eb7 中。需要回退时，在小车桌面执行：

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_robot_tabs.sh --stop
cp runtime_logs/tuning/20260928-responsive-1/motion_control.before.yaml \
  src/wheeltec_motion_control/config/motion_control.yaml
bash scripts/start_robot_tabs.sh
```

这是整套重启，FAST-LIO 地图原点会重置；等待重新就绪后再发新动作。回退后 YAML 会显示为 Git 修改，保留这个事实，不要强制覆盖。日后有其他参数修改时先比较备份，只恢复上表五项。

### 若仍迟滞：对比手柄与 ROS 指令

用户体感观察为手柄到车体动作约 0.25 s、每隔约 0.4 s 切换左右转向时车体几乎不动，尚未同步测量。先完成同条件调参对比，再同步记录手柄事件、最终速度指令和独立车体运动参考；保持速度幅度、地面与负载一致，区分手柄链路、指令滤波、底盘加减速限制和机械响应。仅凭位姿反馈或体感不能定位全部延迟来源。

自动控制器空闲时也持续发零速度。测试手柄前必须退出自动控制接口；仅停止一个受管 motion 进程会触发自动重启，先按上文退出受管启动管理，再按“手动分步启动”仅启动需要的底盘、雷达和定位节点，保留 motion 关闭。手柄启动后检查实际底盘速度话题的发布者，默认话题检查为：

```bash
ros2 node list
ros2 topic info /cmd_vel -v
```

确认自动控制器已经退出，最终速度话题只有当前测试使用的发布者；若手柄另经中间节点转发，也检查其输入与输出话题。试车由用户执行，先确定迟滞所在环节，再决定控制参数或算法的后续调整。

## 自主排查顺序

```bash
ros2 node list
ros2 topic hz /Odometry
ros2 topic echo /Odometry --once
ros2 topic delay /Odometry
ros2 topic info /Odometry -v
ros2 topic info /cmd_vel -v
ros2 action list -t
```

| 现象 | 先检查 |
| --- | --- |
| action server unavailable | 控制接口是否启动；管理脚本可能正在等定位恢复；ROS 域/环境是否相同 |
| goal rejected | 控制器终端/日志的 `Goal rejected:`，不要只看客户端通用提示 |
| control_enabled / extrinsics_calibrated false | 当前加载的 YAML 是否是正确车辆的已确认配置 |
| odometry source timestamp is stale | header.stamp 与当前时间，源年龄需 ≤0.5 s；脚本会等待或恢复，勿直接放宽阈值 |
| wrong frame / future / jump | camera_init、body、use_sim_time=false、时钟及定位是否重置 |
| speed exceeds limit | 请求速度是否超过全局上限；max-speed 不能绕过上限 |
| controller busy | 等上一动作结果，或明确 stop/cancel；不要并发动作 |
| NO_PROGRESS | 是否真的移动；串口、PowerVoltage、速度源冲突、底盘响应与打滑 |
| TIMEOUT | 单动作时间和进度；不能把超时当成功 |
| 发了速度但车不动 | `/dev/wheeltec_controller` 权限/唯一节点、通信回包、电源/底盘；控制器不直接拥有串口 |
| 意外方向/横向偏差 | 正负号、底盘和 IMU 轴、偏移定义；不要先盲目增大增益 |
| existing nodes / another supervisor | 停止旧手动节点或另一份脚本；不要重复占用串口 |

`Motion server ready` 仅表示节点创建成功；手动启动时仍可能没有可用位姿。管理脚本的 STACK READY 才表示当前完整链路检查通过。

`topic delay` 减的是扫描结束时间 stamp，包含输入、处理与订阅端回调调度，不等于网络耗时。需要用户指定进一步排查时，可使用现有只读工具：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/vendor_ws/install/local_setup.bash
python3 ~/workspace/wheeltec_motion_ws/scripts/diagnose_pose_delay.py --seconds 15
```

工具订阅原始 IMU、CustomMsg、Odometry，不发速度；age 是帧头年龄，end_age 是末点年龄，span 是扫描窗口。10 Hz 时帧头比末点旧约 0.1 s 很常见。末尾 PASS 只验证 CDR 解析与消息反序列化一致，不是定位性能结论。全部 no data 时先查节点、环境、域；初始发现阶段短暂 no data 不等于断流。

## 相机单独使用

在独立终端执行 `bash ~/workspace/wheeltec_motion_ws/scripts/start_astra_rgbd.sh --check`，再用 `--start` 打开相机；Ctrl+C 关闭本次相机链。它不启动底盘、定位或运动控制，不使用厂家 sensors/nav/follower 整车入口，不直接加载整个厂家 overlay。

另一个已 source 系统 Humble 与本仓库 install/local_setup.bash 的终端运行 `ros2 run wheeltec_rgbd rgbd_capture --duration 5 --output ~/workspace/wheeltec_motion_ws/runtime_logs/camera/latest`。消费接口为 /wheeltec_camera/rgbd（原子 RGB、米制深度、内参），诊断为 /wheeltec_camera/status。现有 YAML 明确标记设备归属未核验，raw 深度内参仍可能 NaN；完整参数、外参边界与 Python 示例见 [CAMERA_INTERFACE](CAMERA_INTERFACE.md)。

## 日志位置

管理脚本每轮目录 `runtime_logs/robot_stack/日期时间[-tabs]-PID/`，`--status` 可显示标签版当前目录。看 `supervisor.log` 的 UNHEALTHY/RECOVER 原因，再看 base/livox/fastlio/motion.log。标签版另有 *-tab.log 和 bootstrap.log；ros/ 保存 ROS 日志，配置快照记录本轮参数。

手动 ROS launch 默认日志路径显示在启动终端，通常为 `~/.ros/log/`；colcon 的 `log/` 是构建日志。若保存手动控制器屏幕输出：

```bash
mkdir -p ~/workspace/wheeltec_motion_ws/runtime_logs
ros2 launch wheeltec_motion_control motion_control.launch.py \
  params_file:=$HOME/workspace/wheeltec_motion_ws/src/wheeltec_motion_control/config/motion_control.yaml \
  2>&1 | tee ~/workspace/wheeltec_motion_ws/runtime_logs/controller.log
```

复现问题至少记录 commit、当前参数、客户端结果与实际现象。原始日志/录包/地图不进 Git；保留有价值的摘要到当前维护文档，长期实验资料存工作空间外的 archive。无需为普通问答追加逐次日志。

### RGB-D 与定位的只读接入

上层可使用 `wheeltec_rgbd.posed.PosedRGBDReader` 同时读取相机观测和 FAST-LIO 对应时刻位姿。必须先提供并确认相机光学坐标系到车体的实测外参；目前不提供用占位值启动真实建图的命令。构造参数、坐标链、定位故障后的地图清理及重置要求见 [CAMERA_INTERFACE](CAMERA_INTERFACE.md) 的位姿衔接部分。该接口只订阅数据，不启动运动、相机或定位进程。

### D455 相机入口

连接 D455 后运行 `bash ~/workspace/wheeltec_motion_ws/scripts/start_d455_rgbd.sh --start`，只启动相机及 RGB-D 适配器。先退出占用同一 D455 的 Viewer/调试工具，Ctrl+C 关闭。采集检查传 `--camera d455`；奥比为默认 baseline 相机。完整步骤见 [D455_INTERFACE](D455_INTERFACE.md)。D455 安装外参须单独确认，不沿用 Astra 厂家值。
