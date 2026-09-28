# 使用与调试

当前上下文见 STAGE_SUMMARY。所有命令在小车 Ubuntu 22.04 / Humble 上运行；开发机没有原生 ROS。`motion_control.yaml` 是第一台车已启用输出的配置，新车先使用禁用的 example。

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

## 配置及生效检查

控制参数在 `src/wheeltec_motion_control/config/motion_control.yaml`；新车模板 example。路径/四条启动命令/恢复时间在 `scripts/robot_stack/config.yaml`。

```bash
ros2 param get /wheeltec_motion_controller control_enabled
ros2 param get /wheeltec_motion_controller extrinsics_calibrated
ros2 param get /wheeltec_motion_controller body_from_base_translation
ros2 param get /wheeltec_motion_controller pose_timeout
ros2 param get /wheeltec_motion_controller use_sim_time
```

通过 source + 单独 launch 时默认读取 install 配置；管理脚本明确读取源码配置。修改源码并不等于所有现有节点都已重载；停止并重新启动、再查询参数。普通脚本/源码 YAML 更新一般无需 colcon，但源码/接口或安装内容更改后需重新编译。

空闲时部分数值参数可以 `ros2 param set`，变化后等待新位姿；动作执行中参数更新会拒绝。topic/Action/service 名称、control_rate_hz、feedback_rate_hz、use_sim_time 需要节点重启。稳定配置应写回 YAML 并提交，不只保留临时 param set。

## 当前响应参数试调与回退

2026-09-28 第一档试调只应用于第一台车的 motion_control.yaml；最高速度、到达容差、静止速度阈值、定位门槛与频率不变。example 保留原保守值。参数更积极不代表底盘实际延迟已消除，实车效果待用户比较。

| 参数 | 保守值 | 当前试调值 |
| --- | --- | --- |
| linear_accel | 0.15 | 0.30 m/s² |
| angular_accel | 0.5 | 1.0 rad/s² |
| distance_gain | 0.8 | 1.2 s⁻¹ |
| rotation_gain | 1.2 | 1.6 s⁻¹ |
| settle_time | 0.4 | 0.25 s |

先用相同的动作目标和 --max-speed 对比起步/收尾时间；再单独提高请求速度，避免把速度变化误认为调参效果。观察是否增加过冲、来回修正或停稳等待。反馈打印频率仍为 5 Hz，SETTLING 输出零速度。

本车旧配置备份：runtime_logs/tuning/20260928-responsive-1/motion_control.before.yaml。保守版本也保存在 Git 提交 ff29eb7 中。需要回退时，在小车桌面执行：

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_robot_tabs.sh --stop
cp runtime_logs/tuning/20260928-responsive-1/motion_control.before.yaml \
  src/wheeltec_motion_control/config/motion_control.yaml
bash scripts/start_robot_tabs.sh
```

这是整套重启，FAST-LIO 地图原点会重置；等待重新就绪后再发新动作。回退后 YAML 会显示为 Git 修改，保留这个事实，不要强制覆盖。日后有其他参数修改时先比较备份，只恢复上表五项。

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
