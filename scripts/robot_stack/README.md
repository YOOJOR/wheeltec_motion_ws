# 一键启动与自动恢复

小车入口：

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_robot.sh
```

脚本清除继承的旧 overlay 路径，然后依次加载系统 Humble 和四个独立工作空间的 local_setup.bash；保留当前 ROS_DOMAIN_ID、RMW 和网络偏好。不依赖 `.bashrc` alias，不需要手动 source。路径以 [config.yaml](config.yaml) 为准。

前台运行，看到 `STACK READY: accepting NEW motion goals` 后即可在另一个已加载 motion 环境的终端使用现有 motion_client。脚本不会发送移动/转向目标，也不会在恢复后重放旧请求。

## 正常启动顺序

1. 底盘串口通信：收到新的 `/PowerVoltage` 数据，表明确实收到通信回包；不只检查进程存在。
2. Livox：收到新鲜的 `/livox/imu` 和 `/livox/lidar`。用 raw 订阅点云头，避免 Python 逐点解码。
3. FAST-LIO：连续收到至少 3 条时间新鲜、时间戳递增、frame 和四元数有效的 `/Odometry`。默认不启动 RViz。
4. Motion 控制接口：Action 服务可用，控制器持续输出速度消息，并且 `/cmd_vel` 没有其他发布者。

运动配置明确使用 workspaces.motion 下源码中的 motion_control.yaml，不依赖安装目录里可能过期的副本。必须已经启用 control_enabled、extrinsics_calibrated；脚本不会替用户设置它们。base_serial 沿用已有配置中的 senior_mec_bs 和 stm32，不改厂家配置。

开始前不要同时保留手动启动的四个节点。脚本检测到已存在的相关进程就报错退出，不会抢占串口或自动杀死其他控制来源。检查但不启动：

```bash
bash scripts/start_robot.sh --check
```

## 自动恢复规则

| 故障组件 | 自动退出/重启的组件 | 保留的组件 |
| --- | --- | --- |
| 底盘通信 | 先退出 motion，再重启底盘，通信恢复后重启 motion | Livox、FAST-LIO；保留当前地图 |
| Livox | 先退出 motion 和 FAST-LIO，重启 Livox，再等定位健康并重启接口 | 底盘通信 |
| FAST-LIO | 先退出 motion，重启 FAST-LIO，定位健康后重启 motion | 底盘通信、Livox |
| Motion | 仅重启 motion | 通信与定位 |

子节点退出但 ros2 launch 仍活着的情况，通过数据/服务健康检查处理。数据中断、持续旧时间戳和控制器无输出也会触发恢复，不只监测 launch 进程退出。

已运行的通信/定位不健康时先退出 motion；若在 unhealthy_grace_sec 内恢复，直接重新准备 motion，否则重启异常组件。启动期间一直没有合格数据，则 startup_timeout_sec 后重试。重试间隔从 3 s 起，指数增加到最多 30 s；连续健康 60 s 后重置退避。重新启动始终等数据到齐才开放接口。

定位的新鲜度使用 motion YAML 的 pose_timeout 和 future_stamp_tolerance，没有放宽原来的 0.5 s 门槛。FAST-LIO 重启会重建地图并重置 camera_init 坐标；上层程序应结束旧任务、重新获取定位并明确发新请求。接口恢复不代表之前的动作完成，客户端可能收到失败或失联；不要自动认为重启后仍在执行旧目标。

退出 motion 时先 SIGINT，使控制器正常退出并请求零速度；仍不退出时再按配置升级到 SIGTERM/SIGKILL，只处理脚本自己启动的进程组。串口失联或下位机故障后的物理停车能力仍取决于原底盘，脚本不修改 STM32，也不提供固定物理停车时间保证。

## 停止与后台运行

前台按 Ctrl+C，按 motion → FAST-LIO → Livox → base 退出，不再重启。退出整个脚本后，不等于后台守护服务仍在运行。要保持它运行，可以在小车本地终端或 tmux 中启动。

如果通过后台方式运行，先确认不存在另一份脚本：

```bash
cd ~/workspace/wheeltec_motion_ws
mkdir -p runtime_logs
nohup bash scripts/start_robot.sh > runtime_logs/robot-stack-console.log 2>&1 &
```

后台停止（锁文件记录的 PID 是 supervisor；先确认命令，再发 SIGINT）：

```bash
stack_pid=$(cat runtime_logs/robot_stack/supervisor.lock)
ps -p "$stack_pid" -o pid,args
# 确认显示的是 robot_stack/supervisor.py 后，再执行：
kill -INT "$stack_pid"
```

不要重复启动，也不要在后台脚本仍运行时手动启动一份底盘或控制器。脚本未注册为开机服务；本次要求是统一启动与运行期间自动恢复。

## 日志与调整

每次运行生成 `runtime_logs/robot_stack/日期时间-PID/`：

- supervisor.log：启动、就绪、故障原因、恢复顺序和退出。
- base.log / livox.log / fastlio.log / motion.log：各启动程序的原始输出（重启追加）。
- ros/：ROS 节点日志。
- stack-config.yaml、motion-config.yaml：本次配置快照。

看到 `RECOVER` 先看具体组件和原因，再查看对应日志；看到 `cmd_vel publisher conflict` 应退出额外键盘/导航速度发布者。通讯一直不就绪检查串口和回包；Livox 不就绪检查雷达/网络；定位旧检查原始数据和 FAST-LIO。启动超时不自动允许忽略健康门槛。

路径、启动 argv、日志目录、健康/启动/退出时间、重试退避都在 config.yaml。编辑后重启脚本生效。调整 motion 参数仍按工作空间 README 的方法；不要用重试或门槛放宽掩盖持续故障。

可用其他配置：

```bash
bash scripts/start_robot.sh --config /绝对路径/robot-stack.yaml
```

普通脚本更新无需 colcon build，但依赖的四个 ROS 工作空间必须已编译。
