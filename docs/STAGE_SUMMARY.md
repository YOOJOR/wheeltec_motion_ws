# 阶段总结：基础闭环运动控制已可用

基线：**v0.3.0，2026-09-26，ROS 2 Humble / Ubuntu 22.04**。本文是后续开发的统一上下文，可直接据此继续工作；无需阅读早期聊天、逐次开发日志或原始测试输出。只有用户明确要求追查历史问题时才读取旧记录。总体规划仍为开发机 `document/Co-NavGPT2-baseline复现路线图.md`。

## 1. 当前结论与工作边界

当前已完成“FAST-LIO 位姿反馈 → 相对运动 Action → 原厂底盘通信”的基础闭环，并经用户初步实车验证。能够前进/后退、原地左右转向、停止，提供进度与结果、参数配置、离线底盘偏移标定和两种启动/恢复方式。

这不是完整自主导航系统：没有路径规划、障碍检测、避障、旧地图重定位或多车协作。尚未完成总体规划 M1 的长期定位验收，也未完成 M2 的完整 TF/重复精度验收。当前是 **M2 控制适配部分已实现、整体仍进行中**，不能把基础动作成功写成 Co-NavGPT2 baseline 已跑通。

用户已经明确决定：

- 不修改 STM32 固件；原厂通信协议和串口节点保持原样。
- 暂缓“0.5 秒内物理停车”要求；`pose_timeout=0.5` 是位姿有效性门槛，不是制动保证。总体规划早期的该项门槛当前不作为已通过项。
- 偶发定位延迟先依靠自动恢复继续推进；保留原因未确认的事实，暂不继续深挖。
- 实车运动验证由用户执行；自动开发验证采用合成话题/独立 ROS 域，不发实车动作。
- 只将自己的 wheeltec_motion_ws 上传 GitHub；厂家 base 不上传、不改源码。
- FAST-LIO 暂留 conavGPT_ws，迁入 vendor_ws 是后续事项，本版本不搬动。

## 2. 硬件、环境与目录

第一台车：WHEELTEC **senior_mec_bs** 麦克纳姆底盘，支持原地旋转；MID360S 使用内置 IMU，雷达 X 朝车头。底盘通信的 `imu_mode=stm32` 表示厂家底盘自身 IMU 来源，并不改变 FAST-LIO 使用 Livox IMU。

小车用户 wheeltec；项目根 `~/workspace`。开发机 NFS `/mnt` 对应小车 `/home/wheeltec/workspace`。NFS 源码可读，但 install 内指向小车绝对路径的链接在开发机可能无法解析，不代表小车安装损坏。

| 工作空间 | 内容与职责 | 本次处理 |
| --- | --- | --- |
| wheeltec_base_ws | 厂家通信、消息、串口库三包 | 保持原样，源码已核对 45 文件无差异；只记录复刻过程 |
| vendor_ws | Livox ROS 驱动及 SDK2 源码；SDK 安装在 /usr/local | 不修改；锁定来源和实际配置差异 |
| conavGPT_ws | 当前仅 FAST_LIO_ROS2；名字不代表已接入 Co-NavGPT2 | 不修改；只关闭过在线外参估计和调整过 RViz 显示 |
| wheeltec_motion_ws | 自研 Action、控制、标定、启动管理 | v0.3.0 正式维护仓库 |

来源锁定：

| 组件 | 来源 | 当前源码提交 |
| --- | --- | --- |
| 运动控制 | github.com/YOOJOR/wheeltec_motion_ws，main | 用 v0.3.0 标签定位本阶段版本 |
| 厂家底盘抽取 | 小车 /home/wheeltec/wheeltec_ros2/src | 独立本地基线 fe99b01f8e047f686aa2d004ddb38a829f909a3b |
| FAST-LIO | github.com/Ericsii/FAST_LIO_ROS2，ros2 | 2fffc570a25d0df172720bac034fbdb6a13d2162 |
| Livox 驱动 | github.com/Livox-SDK/livox_ros_driver2，master | 4a1def929e5b59c7a8122d19fce6efba581ce9f7 |
| Livox SDK2 | github.com/Livox-SDK/Livox-SDK2，master | 08f523c930b2f0ba1e98a6afaa8d7476bf479908 |

SDK 源码干净；现有 SDK 构建缓存仍记录早先 Documents/navGPT_ws 路径，不能仅凭当前源码 HEAD 证明 /usr/local 二进制当时构建自同一提交。第二台车按锁定源码重新编译安装，避免复制这个旧构建目录。完整步骤见 WORKSPACE_SETUP。

## 3. 数据流与坐标约定

```text
MID360S → livox_ros_driver2 → /livox/lidar + /livox/imu
                              ↓
                           FAST-LIO
                              ↓ /Odometry
                   wheeltec_motion_control
                              ↓ /cmd_vel（Twist，20 Hz）
                 turn_on_wheeltec_robot → 串口 → STM32
```

| 接口 | 类型/约定 |
| --- | --- |
| /Odometry | nav_msgs/msg/Odometry，约 10 Hz；world=camera_init，child=body（IMU） |
| /livox/lidar | livox_ros_driver2/msg/CustomMsg，启动配置 10 Hz |
| /livox/imu | sensor_msgs/msg/Imu，观测约 200 Hz |
| /cmd_vel | geometry_msgs/msg/Twist，linear.x/y 为 m/s，angular.z 为 rad/s |
| /PowerVoltage | std_msgs/msg/Float32，用新回包判断底盘通信就绪 |
| /wheeltec_motion/execute | wheeltec_motion_interfaces/action/ExecuteMotion |
| /wheeltec_motion/stop | std_srvs/srv/Trigger |

底盘节点使用 `/dev/wheeltec_controller`，115200 baud。闭环控制器不打开串口、不发布 TF。不要把 body 改名当成 base_link，也不要向 FAST-LIO 的 body→地图链额外发布重复 TF。当前控制器内部换算底盘位姿；供相机融合/导航使用的完整 TF 链仍需后续明确唯一发布者。

计算关系：`T_world_base = T_world_body × T_body_base`，先做完整三维变换，再取平面 X/Y/yaw。`body_from_base_translation` 是底盘参考点在 IMU/body 轴下的坐标；`body_from_base_rpy` 把底盘轴旋转到 body 轴。当前 base 选为地面上的有效原地旋转中心，而不是外壳某个角点。

FAST-LIO 的 `mapping.extrinsic_T` 是雷达原点在 IMU 中的位置，与底盘偏移是两个独立参数。当前 FAST-LIO 沿用上游 T=[-0.011,-0.02329,0.04412]、R=单位阵，在线估计 false；没有使用相关 fork 提到的旧 T=[0.05512,0.02226,-0.0297]。MID360S 精确内部尺寸仍需按其型号资料核对，不能凭与 MID360 配置相同就声称已验证。

## 4. 控制原理和上层调用契约

`wheeltec_motion_interfaces` 只生成 Action 类型；`wheeltec_motion_control` 提供服务端、CLI 和离线标定。任何具备该接口类型、处于同一 ROS 域/可达网络的 ROS2 程序都可调用；这里不是 HTTP API。

目标从**接受请求时的底盘位姿**定义。直行沿当时底盘 X 轴计算已走距离/横向偏差，用比例控制、航向修正、限速和加速度限制输出 Twist。转向累计连续 yaw，避免跨 ±π 的方向错误。进入容差后发布零速度并等待新的位姿满足稳定条件，才返回成功。

Action 定义位于 `src/wheeltec_motion_interfaces/action/ExecuteMotion.action`：

- Goal：motion_type=1（MOVE_LINEAR）/2（ROTATE）；target 的单位分别 m/rad；正数前进/左转，负数后退/右转。
- max_speed=0 使用配置上限；正数只能降低该动作速度，超过全局上限会拒绝。timeout_sec=0 使用默认动作超时。
- Feedback：phase、当前底盘 x/y/yaw、remaining、cross_track_error、elapsed_sec；直行 remaining 为 m，转向为 rad。
- Result：code、message、final_x/y/yaw、remaining、elapsed_sec；code=0 SUCCEEDED、1 CANCELED、2 POSE_INVALID、3 TIMEOUT、4 NO_PROGRESS、5 STOPPED、6 INTERNAL_ERROR。
- 同时只接受一个动作；忙碌、输出未启用、外参未确认、位姿无效、参数越界均可能拒绝。拒绝原因记录在控制器 `Goal rejected: ...` 日志中，客户端通用提示不包含完整原因。
- 上层先确认 accepted，再等 Result；只有 code=SUCCEEDED 才进入下一步。接受成功不等于运动完成，接口重新出现不等于旧动作成功。
- Ctrl+C 客户端会尝试取消已接受目标；直接 kill 客户端或客户端失联不等于停止请求。明确停止用 Trigger/`motion_client stop`。
- 不提供独立横移目标、避障或多速度源仲裁；lateral_correction 当前关闭。当前约定 `/cmd_vel` 仅有控制器一个发布者，键盘/导航不要并行发布。

## 5. 第一台车参数基线

`src/wheeltec_motion_control/config/motion_control.yaml` 是第一台车已使用的配置，输出启用；example 是新车的禁用模板。仓库不再假称实车 YAML 为零偏移占位值。

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| control_enabled / extrinsics_calibrated | true / true | 第一台车已启用并确认底盘偏移 |
| body_from_base_translation | [-0.10718,-0.02237,-0.20388] m | 底盘旋转中心在 IMU 中的位置 |
| body_from_base_rpy | [0,0,0] rad | 当前按 IMU 与底盘轴一致配置 |
| control_rate_hz / feedback_rate_hz | 20 / 5 | 速度更新与 Action 反馈频率 |
| max_linear_speed / max_angular_speed | 0.15 m/s / 0.35 rad/s | 全局上限，不是每个动作一定达到的速度 |
| linear_accel / angular_accel | 0.15 m/s² / 0.5 rad/s² | 加速限制；停止直接请求零速度 |
| distance_tolerance / angle_tolerance | 0.03 m / 0.035 rad | 后者约 2° |
| cross_track_tolerance / lateral_correction | 0.05 m / false | 当前不启用麦轮横移纠偏 |
| settle_time | 0.4 s | 进入容差后持续稳定判据 |
| pose_timeout / future_stamp_tolerance | 0.5 s / 0.1 s | 位姿源时间与接收新鲜度检查 |
| default_timeout / no_progress_timeout | 45 s / 5 s | 单动作超时、无可测进展超时 |
| max_distance / max_rotation | 5 m / 2π rad | 单次相对目标上限 |

XY 来自原地左右旋转录包拟合；Z 采用已测雷达底部距地面 0.201 m，加此前使用的 IMU 相对固定面 0.00288 m，得到 IMU 高度 0.20388 m，对应底盘点 Z=-0.20388 m。该机械/坐标假设不是所有车辆通用值。第二台车必须重新确认参考点、轴向和安装高度。

`calibrate_rotation` 是 control 包内的独立离线命令，不在在线控制器启动时自动运行。可读 rosbag2 `/Odometry` 或导出 CSV，按左右转分别拟合固定旋转中心；只提供平移建议，不能从平面旋转可靠求出未知高度，也不自动修改 YAML。操作见 CALIBRATION。

## 6. 日常启动、恢复和停止

推荐小车桌面 `bash ~/workspace/wheeltec_motion_ws/scripts/start_robot_tabs.sh`；没有图形桌面时使用前台 `start_robot.sh`。两者共用配置与单实例锁，不可同时运行。脚本自行清除旧 overlay 子环境并 source 系统 Humble 与四空间 local_setup，不依赖 .bashrc alias。

启动顺序：收到底盘 PowerVoltage 新回包 → Livox 点云/IMU 新鲜 → FAST-LIO 连续至少 3 条新鲜、递增且 frame/姿态有效的 Odometry → 控制 Action 可用且只有一个控制速度发布者。默认不启动 RViz。运行中的控制器空闲时持续发零速度，停止其他速度发布者后再使用。

| 异常 | 恢复策略 |
| --- | --- |
| 通信 | 先退出 motion，再重启 base；保留 Livox/FAST-LIO 地图 |
| Livox | 退出 motion、FAST-LIO，重启 Livox→定位→motion |
| FAST-LIO | 退出 motion，重启定位，数据合格后重启 motion |
| motion | 仅重启 motion |

运行中上游不健康时先撤下 motion；持续约 3 s 异常后恢复相应组件。启动阶段 45 s 未就绪重试；重试间隔 3 s 指数增加，最多 30 s，健康 60 s 后重置。恢复总时间还取决于驱动、ROS 发现和定位，不承诺几秒内完成。

标签页版本 `--status` 查看、`--stop` 停止；任一受管标签 Ctrl+C 或关闭它会停止整套服务，不再重启。普通启动终端返回后标签页继续运行。原版前台 Ctrl+C 同样退出全部；后台原版需先核对 supervisor.lock 的 PID/命令后发 SIGINT。脚本不保证自己被 SIGKILL 后仍作为系统服务存活，也没有注册开机启动。

FAST-LIO 重启会重建地图/重置 camera_init 原点；不会续跑旧 Action。后续任务编排必须结束旧任务、重新确认定位和目标，再由上层明确发新目标。

## 7. 已验证事实、限制和已知问题

- 用户已成功执行 0.2 m 低速直行及 90°转向。直行示例结果剩余约 0.02097 m，符合当前 0.03 m 容差；不是毫米级实测精度证明。
- 运动/标定已有 29 项 Humble 测试覆盖几何变换、速度/加速度、前后直行、跨角转向、停止/取消、位姿异常、超时及离线标定；启动管理 14 项测试覆盖依赖恢复、同页重启、并发退出和进程清理。本版本复验结果见 VALIDATION。
- 实车已验证四组件退出恢复、Livox 重启保持原标签、主动中断退出全链路，未由开发端发送实车移动/转向目标。
- 正常 Odometry 时间年龄常约 30–40 ms；偶发启动后约 1.3–1.4 s，单独重启 FAST-LIO 有时恢复。目前只认定“症状可通过重启恢复”，不认定已修复根因。
- `ros2 topic delay` 是测量订阅回调时刻减 header.stamp；FAST-LIO stamp 是点云扫描结束时间，包含输入、配对/处理、发布及测量端调度，不能直接当作发布到控制器的网络延迟，不能单凭此值证明队列积压。
- 相同现象参考：Taeyoung96/FAST_LIO_ROS2 #13（偶发 1.3 s、重启约 40 ms）；Livox 驱动 #229（CustomMsg 与 IMU 时间差）、#254（组帧时间间隔异常）。截至本阶段未确认与本车同一根因。用户已选择等待 issue 回复，暂缓排查。
- 未完成：长期运行、不同距离/地面/负载重复精度、实际停车时间、全项目 TF 验收、RGB-D 相机融合、目标检测/规划、双车通信与协作。

## 8. 后续工作顺序与更新规则

1. 先按已固定版本复刻第二台车：IP、串口、车型、IMU/底盘偏移按本车填写，重复编译与基本功能检查。
2. 用户进行前后直行/左右转向重复测试，用尺子或地面标记作独立参考，记录误差与时间；调整容差/增益须说明理由。
3. 上层运动任务编排：每步等 Result；失败、取消、定位重置时终止整段；禁止无条件重放旧请求。先完成短序列再接入规划器。
4. 结合总体规划推进 RGB-D 相机、TF 和时间配对；按原 Co-NavGPT2 流程做单车融合/规划，再谈多车，不把当前接口当作已经完成导航。
5. 将 FAST-LIO 合入 vendor_ws 时重新构建干净 install、调整 robot_stack/config.yaml 的 fastlio 路径及 .bashrc/alias，并复验唯一包解析。只记录方案，本次未迁移。

新任务开始读取本文；使用细节按需读 OPERATIONS，部署按需读 WORKSPACE_SETUP。旧记录归档在小车 `~/workspace/archive/motion-before-v0.3.0/`、开发机 `archive/motion-before-v0.3.0/`，不是默认阅读入口；Git 旧提交仍保留过程可追溯。

后续开发提交时更新本文的事实/边界与 CHANGELOG，验证摘要更新 VALIDATION，原始运行数据放忽略的 runtime_logs。不能把“编译成功”“一次动作成功”替代尚未完成的量化验收。
