# 持续速度导航接口与 Co-NavGPT2 接入

本接口在现有运动控制节点内接收最新速度请求，负责限速、加速度限制、请求有效性和失效停止。它不规划路径、不做方向纠偏，也不是闭环路径跟踪。相对运动 Action 保留原有到达容差、停车、停稳后返回的语义。当前未接 D455，本次不包含 RGB-D 建图、FMM、识别、避障或多车算法。

## 实际接口

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/wheeltec_motion/navigation/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 最新导航速度请求；输入 QoS 为 BEST_EFFORT / VOLATILE / depth=1，兼容可靠发布者 |
| `/wheeltec_motion/navigation/set_enabled` | `std_srvs/srv/SetBool` | data=true 启用导航；false 退出；必须检查 success/message |
| `/wheeltec_motion/status` | `wheeltec_motion_interfaces/msg/MotionStatus` | 5 Hz 与状态变化时发布；RELIABLE / TRANSIENT_LOCAL / depth=1，可订阅查询最近状态 |
| `/wheeltec_motion/stop` | `std_srvs/srv/Trigger` | 停止当前 Action 或导航；导航退出且清空缓存 |
| `/wheeltec_motion/execute` | `wheeltec_motion_interfaces/action/ExecuteMotion` | 保留相对直行/转角 Action |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 本节点唯一输出，20 Hz；上层不得直接发布此话题 |

`MotionStatus.mode` 为 IDLE=0、ACTION=1、NAVIGATION=2；包含 phase/reason、输出和外参启用状态、pose_ready、导航 session/启用时间、请求接收年龄及请求/输出速度。年龄为 -1 表示无有效请求。输出速度是软件命令，不是测量速度；状态零速度和 stop.success 均不证明物理停车。查询时同时确认状态时间戳新鲜，不能只信旧的瞬态本地样本。

## 坐标系与时间

- `header.frame_id` 必须严格为配置 `navigation_frame`（默认 `wheeltec_base`）；速度定义在当前底盘控制参考点：地面有效旋转中心，x 朝前、z 朝上。`linear.x` 单位 m/s，正前进、负后退；`angular.z` 单位 rad/s，正左转、负右转。两者可同时非零。
- `linear.y/z`、`angular.x/y` 必须全部为零，即使本底盘具备麦轮横移能力，首版导航也不接受横移。六个分量均须有限。
- 这个 frame 名称只标明输入语义。控制器不广播 TF，填写 `wheeltec_base` 或 `base_link` 不会建立 TF 链。
- `header.stamp` 是上层新鲜请求的原始 ROS 时间，不能为空，须严格递增且晚于本次启用时刻。客户端与服务器用同一 ROS 时钟；分机运行须同步系统时间，不能填写单调时钟。客户端示例保留新观测/决策的原始时间，禁止给缓存决策换一个“现在”的时间戳。
- 主源码Agent分别记录图像采集时间 source_stamp_ns 与新决策开始时间 decision_stamp_ns；速度请求 header 使用后者，在开始处理一组新的有效观测时取得，不能在计算结束或重发时重新打戳。图像年龄和处理耗时仍独立各限0.3s，过期输入不允许发送运动。这样请求时效与传感器时效有各自明确的起点。
- 请求在接收时和每次输出时同时检查源年龄；收到后再用单调时钟检查有效期。允许源时间最多超前 0.1 s；源年龄或接收年龄超过 0.3 s 则停止。延迟到达的请求不会获得额外完整寿命。ROS 时钟大幅后退或前跳也会导致失效停止。
- 源时间不晚于本次启用时间，或不大于最近已接收源时间的请求，被视为旧包/重复包：不执行、不替换、不刷新有效期，状态 reason 报告忽略。空/零时间戳也属于旧包。负秒或非法纳秒字段属于非法时间戳。

相机建图参考点、FAST-LIO 的 IMU `body`、底盘控制参考点是不同位置。已有控制器使用完整 `T_world_base=T_world_body*T_body_base` 后提取平面位姿。本车安装平移 `[-0.10718,-0.02237,-0.20388] m` 保留。相机外参不能替代底盘外参；世界坐标 FMM 目标必须由上层结合已标定相机/IMU/底盘变换和当前底盘朝向，选择前进或转向。相机偏离旋转中心时旋转会产生相机位移，不能把相机航向/位置误当底盘点。完整 TF 链及唯一发布者仍待后续接入验收。

## 模式、失效与恢复

| 当前模式 | 请求 | 行为 |
| --- | --- | --- |
| IDLE | 启用导航 | 要求 control_enabled、extrinsics_calibrated 为 true，位姿源/接收均新鲜且至少两帧连续有效；进入 NAVIGATION/WAITING，立即零速度，清除缓存 |
| NAVIGATION | 新有效速度 | 替换唯一目标；独立输出定时器按已有加速度逐步更新，不等待 Action 或停稳 |
| NAVIGATION | 再次启用 | 幂等，保留 session，不清零当前运动，也不刷新请求寿命 |
| NAVIGATION | 新零速度 | 立即零速度，保持 NAVIGATION/ZERO，可接受下一条新鲜请求；不会进入 SETTLING |
| NAVIGATION | 相对 Action | 拒绝 |
| ACTION（含接受至结果返回的预留期） | 启用或退出导航 | 拒绝，不影响该 Action；要停车调用 stop 或取消 Action |
| NAVIGATION | 退出、stop、指令/定位/控制失效 | 立即零速度、清空请求、转 IDLE 并报告原因 |
| IDLE | 导航消息 | 忽略，不隐式启用 |

无请求的 NAVIGATION/WAITING 持续零速度；首条请求前没有指令超时，但定位与控制调度检查仍生效。相同速度的新请求不会导致每轮停车。速度变化和反向请求复用 linear_accel/angular_accel；显式零请求、stop、退出和失效绕过平滑限制，直接请求零速度。

新请求含 NaN/Inf、非零不支持分量、错误 frame、过期/异常超前时间，或超出当前全局速度上限，均**整条拒绝并退出导航**，不截断后继续运动。旧/重复包采用前述忽略策略。不会接受待执行队列。即使新的接收回调先于迟到的控制定时器执行，也不能用新包掩盖已经发生的请求超时或控制调度缺口。

位姿检查复用原有帧、四元数、外参、接收/源时间及跳变门槛；定位跳变或断流在恢复帧到达时仍触发停机。调度间隔超过 max_control_gap 或控制异常也退出导航。定位恢复、程序重启或 ROS 接口重新出现不会续跑；必须重新确认状态、显式启用，再从新的观测产生晚于启用时间的新请求。节点重启初始模式总是 IDLE。

软件停止在故障回调或下一个可执行的控制 tick 发布；调度卡死期间不能执行发布，进程被强制终止后也不能继续输出。底盘制动、串口执行和节点失联后的硬件行为须实测。当前没有导航 NO_PROGRESS/停稳成功判据，也没有障碍检测。

## 配置与生效检查

已核对本次启动配置和构建后安装配置：第一台车 cap 为 **0.15 m/s、0.35 rad/s**，linear_accel=0.30、angular_accel=1.0。原有外参、限速、五项试调值及定位门槛保持；新车 example 保持禁用输出和保守值。上游 0.3 m/s、±0.5 rad/s 不适用于本车。

本轮默认 ROS 域只读 `ros2 node list --no-daemon` 未发现运行节点，未启动实车链路；本轮没有声称读回正在运行的实车参数。已完成本机源码/安装产物构建，后续启动读取新接口与配置，启动后按下述命令核对运行值。

| 新参数 | 默认/本车值 | 作用 |
| --- | --- | --- |
| navigation_command_timeout | 0.3 s | 导航源时间与单调接收年龄上限，独立于 pose_timeout=0.5 s |
| navigation_future_stamp_tolerance | 0.1 s | 导航请求源时间允许超前量，独立于位姿 future_stamp_tolerance |
| navigation_frame | wheeltec_base | 底盘控制参考点语义标签 |
| navigation_topic | /wheeltec_motion/navigation/cmd_vel | 输入话题 |
| navigation_mode_service | /wheeltec_motion/navigation/set_enabled | 模式服务 |
| status_topic | /wheeltec_motion/status | 状态话题 |

max_linear_speed、max_angular_speed、linear_accel、angular_accel、max_control_gap 和现有定位门槛共同生效。数值参数仅可在 IDLE 修改，随后需要新位姿；导航或 Action 期间拒绝参数修改。话题/服务/frame、频率及 use_sim_time 需要重启。

构建后按 OPERATIONS 重启自己的控制接口，再只读核对**运行节点**，不能把 YAML 与旧进程视为自动同步：

```bash
ros2 param get /wheeltec_motion_controller max_linear_speed
ros2 param get /wheeltec_motion_controller max_angular_speed
ros2 param get /wheeltec_motion_controller navigation_command_timeout
ros2 param get /wheeltec_motion_controller navigation_frame
ros2 topic info /cmd_vel --verbose
ros2 topic echo /wheeltec_motion/status wheeltec_motion_interfaces/msg/MotionStatus --once
```

## 最小接入示例

先加载 Humble 和本工程 install/local_setup.bash。模式和停止 CLI 不发布运动速度：

```bash
ros2 run wheeltec_motion_control navigation_client status
ros2 run wheeltec_motion_control navigation_client enable
ros2 run wheeltec_motion_control navigation_client disable
ros2 run wheeltec_motion_control navigation_client stop
# 原生等价调用
ros2 service call /wheeltec_motion/navigation/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /wheeltec_motion/navigation/set_enabled std_srvs/srv/SetBool '{data: false}'
ros2 service call /wheeltec_motion/stop std_srvs/srv/Trigger '{}'
```

Python 适配器 `wheeltec_motion_control.navigation_client.NavigationClient` 已安装；前进默认 0.12 m/s、左/右转 ±0.30 rad/s。它没有重发定时器；每次 update 只发布一个新决策。同步方法用于上层主循环，不放在 ROS 回调或已有 executor 中；已有 executor 的项目可照此契约改用异步 service future。

服务明确拒绝启用（例如 Action 占用）时，适配器直接报错，不调用 stop 干扰该 Action。若启用请求已发出但响应丢失或新鲜状态无法确认，适配器尝试 stop 并停止发布，以处理启用结果不确定的情况；不会自动重试启用。

以下接入结构中 `upstream` 是待实现的本车感知/规划适配层，不是本次已完成的 Co-NavGPT2 入口：

```python
import rclpy
from wheeltec_motion_control.navigation_client import NavigationClient

rclpy.init()
nav = NavigationClient()  # 默认本车限速内；其他车核对配置后再设置
try:
    nav.enable()           # 检查服务与新鲜 NAVIGATION 状态；无运动请求
    while upstream.running():
        if upstream.will_block_for_vlm_or_other_wait():
            nav.stop()    # 先确认 stop 服务成功；模式已退出
            upstream.run_blocking_work()
            upstream.discard_cached_decisions()
            # 明确确认定位、地图与控制恢复；不是故障后的自动重试。
            upstream.confirm_ready_to_resume()
            nav.enable()
        # 每轮新观测计算一次新决策；保留其原始 ROS 源时间。
        decision, source_stamp_ns = upstream.observe_and_choose_new_action()
        if decision == 'stop':
            nav.stop()
            break
        nav.update(decision, source_stamp_ns)  # forward / left / right / reverse
except (RuntimeError, ValueError):
    # 接口失效/状态过期/决策过期：终止旧任务，不自动重新启用。
    try:
        nav.stop(timeout=.3)
    except RuntimeError:
        pass             # 不再发布；服务失联时依靠服务端请求失效停止
    raise
finally:
    try:
        nav.stop(timeout=.3)
    except RuntimeError:
        pass
    nav.destroy_node()
    rclpy.shutdown()
```

正常运动时上层应持续处理新观测、产生新决策并更新请求；名义周期 0.1 s 只是期望。不能每 0.1 s 给上一次速度换新时间戳重发来掩盖感知或 VLM 卡住，不能用无限延长 timeout 保持运动。准备进入同步 VLM、等待人工输入或已知长感知处理前先 stop；如果不可预期的处理超过 0.3 s，导航会退出，之后不能仅补发速度恢复。适配器另外检查状态新鲜度（默认 0.6 s）和会话，失效后拒绝发布，但停止保障仍由控制端 watchdog 执行。客户端 timeout 必须与服务器参数一致。

同一动作连续多个新观测选择为 forward 是允许的；它们是有新时间戳的新决策，不是定时重放。STOP 后新的观测必须在重新启用之后取得，不能重用阻塞前的世界目标、位姿或速度。

## 本轮验证与待试车项

完整记录见 VALIDATION。自动验证只使用独立 ROS 域 173/174、合成位姿和 `/motion_test/*`，未发送实车运动指令。实车仍需用户验证：持续运动/换向响应、上层实际处理周期与 watchdog 命中、主动/失效后的物理停车、真实定位异常恢复，以及接相机后的坐标变换。软件接口完成不代表 Co-NavGPT2 自主探索已跑通。
