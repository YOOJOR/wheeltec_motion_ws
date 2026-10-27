# CoNavBot 实机部署与 WHEELTEC 运动工作区

本仓库负责路径跟踪、相对运动、D455 启动、基础栈管理和可视化。感知建图与 FMM 规划见相邻 [conavgpt2_ws](../conavgpt2_ws/README.md)。开发约束见 [AGENTS.md](AGENTS.md)，历史见 [CHANGELOG.md](CHANGELOG.md)。

## 当前状态

当前阶段：**探索预览版（2026-10-27）**。用户确认本轮策略修正后仿真探索初步正常，两个工作区配套使用；实机完整探索尚未验收。

- 桌面仿真环境已部署，支持 RViz 点目标与 Agent 探索入口、0.20 s 指令延迟及四轮 PID 力矩驱动。已完成编译、静止图形/传感器启动和 Agent 只读建图启动；运动表现由用户交互观察，操作见下方“桌面交互仿真”。
- Agent 输出前方约1 m路径；普通探索目标使用0.10 m到达容差，物体目标保持原版判定；Navigation 以20 Hz根据实时定位跟踪。motion 不处理地图、障碍判断或局部绕行。
- motion接口、控制与conavbot_sim已在x86_64 ROS Humble容器构建，关闭BUILD_TESTING；图形、传感器、状态显示完成静止检查，运动由用户交互观察。车载构建与实机路径跟踪待确认。
- D455 安装外参尚未配置，世界建图入口会拒绝启动。下一步补齐外参与 TF，再确认图像、定位配对及实机跟踪。
- 两端须同步部署并重建 motion 接口包；旧速度导航话题已删除。

## 目录与数据流

```text
D455 → RealSense驱动 → RGB/深度/内参 ─┐
MID360S → Livox → FAST-LIO → 位姿 ├→ conavgpt2_ws 原版Agent → 新鲜路径请求
                               └→ wheeltec_motion_control → 厂家底盘 /cmd_vel
```

| 位置 | 用途 |
| --- | --- |
| src/wheeltec_motion_control | node.py 集中完成运动控制；导航客户端、动作命令行和离线标定各自单文件 |
| src/wheeltec_motion_interfaces | ExecuteMotion Action、NavigationPath、MotionStatus |
| scripts/d455.launch.py、config/d455.yaml | 独立D455驱动；导航端直接接收和配对 |
| scripts/robot_stack | 基础栈依赖检查、启动与恢复 |
| scripts/start_conav_exploration.sh | 相邻导航仓库的主Agent入口 |
| config/rviz | 地图及传感器显示布局 |

## 坐标、参数与接口

当前参数源为 [motion_control.yaml](src/wheeltec_motion_control/config/motion_control.yaml)；新车使用禁用输出的 [example](src/wheeltec_motion_control/config/motion_control.example.yaml)。

- 底盘参考点是厂家四轮几何中心 `base_link`，`base_footprint` 在其下方 0.026 m。
- MID360S 后装：`T_body_base` 平移 `[-0.10718,-0.02237,-0.17788] m`、零RPY；XY暂保留旧旋转拟合作安装估计，未独立量到几何中心。不要使用厂家 laser_link/radar 代替。
- 完整三维计算 `T_world_base=T_world_body*T_body_base`，相机再乘 `T_base_optical`；地面点为 `T_world_base*[0,0,-0.026,1]`，倾斜时也如此。
- 最高速度0.15 m/s、0.35 rad/s；请求源年龄和接收年龄各3 s，位姿3 s，默认动作超时45 s。速度按上限截断，正常运动使用加速度限制；stop、取消和超时直接归零。

| 接口 | 契约 |
| --- | --- |
| /Odometry | camera_init/body，FAST-LIO IMU位姿；不是底盘点 |
| /wheeltec_motion/navigation/path | NavigationPath：camera_init 下的 nav_msgs/Path（米）及正的线/角速度上限 |
| /wheeltec_motion/navigation/set_enabled | SetBool；必须检查success，启用不直接产生运动 |
| /wheeltec_motion/execute | 相对直行/转角Action，与导航互斥；先检查accepted，再等成功Result |
| /wheeltec_motion/stop | Trigger；退出导航/停止Action并请求零输出 |
| /wheeltec_motion/status | 软件状态；须检查时间新鲜度，不代表实测速度或物理停稳 |
| /cmd_vel | 由运动控制器唯一发布；上层不得直发或与手柄并行发布 |

请求时间取决策开始，须递增且晚于导航启用；旧包不续期。空路径立即停车并保持导航；非法输入、超时或 stop 清除路径并退出导航，恢复后须重新启用。

路径以本次决策开始时间标记，路径和各点 frame_id 均为 camera_init。每条新路径替换旧路径，不重置输出速度；相邻重复点合并，单点作为位置目标，姿态字段不参与控制。Navigation 先投影到最近路径段，再向前取 `path_lookahead=0.25 m`；航向比例控制使用 heading_gain，偏角≥60°时立即停止平移、原地转向，偏角较小时按 cos(偏角) 降低前进速度。路径末端按 distance_gain 减速，进入 distance_tolerance 后归零。状态为 WAITING_PATH / TRACKING / REACHED；到达后仍受3 s路径超时约束。状态 requested_linear_x/angular_z 表示跟踪器加速度限制前的目标速度。

启动扫描异步执行旋转2π的 Action（45 s超时、使用 --turn-speed），期间持续建图。成功后才启用 Navigation，等待启用后的新观测生成路径。只读运行不提交扫描，--skip-initial-scan 跳过扫描。数据故障、预算耗尽或退出均 stop；新鲜路径显示在 RViz 的 Agent navigation path（/conavbot/path）。

相对运动的 move 单位 m、rotate 单位 rad，正号前进/左转。直行使用起始航向投影计算距离，并进行航向纠偏；距离与航向都进入容差即归零并返回成功。横向偏差只作反馈。旋转按累计转角控制，支持跨越 ±π；进入角度容差即完成。动作请求可指定速度和超时，速度仍受车辆上限约束。直接结束客户端不能替代 stop。

运动控制集中在 `node.py`。YAML 分为启用与安装、速度与加速度、增益与容差、超时；参数修改后重启生效。控制20 Hz、反馈5 Hz，源时间超前容许量0.1 s。

## 相机与时间契约

D455 与 FAST-LIO 独立发布感知和定位。相机由 [start_d455.sh](scripts/start_d455.sh) 启动，参数集中在 [d455.yaml](config/d455.yaml)：640×480、15 Hz、RGB8/Z16、同步和深度对齐；关闭点云、IMU及红外输出。单独执行 `bash scripts/start_d455.sh --start`，可追加 `serial_number:=设备序列号`；已有统一栈时不要重复启动。

直接发布 `/camera/d455/raw/d455/` 下 `color/image_raw`、`aligned_depth_to_color/image_raw` 及各自 `camera_info`。

导航端后台接收四路消息，待配对缓存每路两条，完整观测两组。图像与内参同戳，RGB/深度差≤30 ms；按深度时间取最近位姿，差≤50 ms，不外推。仅解码最新可配对的新观测，毫米转米，0.4–6 m外及无效深度置NaN；原版Agent仍有4 m截断和边缘处理。标定变化时重新检查及生成去畸变映射。定位失效仍锁存，显式重置清除旧地图与缓存。

基础栈监控四路数据更新，不判断像素有效率；全零深度由导航端报告，不重启相机。D455安装外参为空，采集可独立运行，世界建图入口拒绝缺失外参。

## 获取与部署

### 桌面交互仿真（与实车隔离）

`conavbot_sim` 使用 Podman 中的 Ubuntu 22.04 / ROS Humble / Gazebo Fortress。
房间为 8×6 m，包含箱体与隔断；四轮麦克纳姆模型按约 0.313 m 轴距、0.355 m 轮距简化。
每轮带被动滚子，驱动使用轮速 PID 输出力矩；指令默认延迟 0.20 s，电机力矩有响应惯性和转速限制。
这些动力学参数为手动调试起点，未按实车辨识。

在本工作区运行：

```bash
bash scripts/start_sim.sh build    # 首次安装镜像和编译；更新源码可使用 compile
bash scripts/start_sim.sh up       # 打开 Gazebo、RViz；等待用户目标，默认静止
```

在 RViz 顶部选择 **2D Goal Pose**，在空闲区域点击并拖动后松开；当前位置到目标的 FMM 路径交给现有 Motion 跟踪。
第一版仅要求到达目标位置，拖动方向不作为终态朝向要求。障碍附近、地图外和不可达目标会被拒绝。

```bash
bash scripts/start_sim.sh explore  # 切换到现有 Agent，持续探索；无前沿时随机选目标，会运动
bash scripts/start_sim.sh goal     # 停止探索，回到等待 RViz 点目标
bash scripts/start_sim.sh stop     # 停止任务；普通零指令保留延迟和电机制动
bash scripts/start_sim.sh estop    # 清空指令队列并锁存急停；仍有物理制动过程
bash scripts/start_sim.sh reset    # 重建场景、清地图/时钟/PID；回到等待点目标
bash scripts/start_sim.sh down     # 关闭所有仿真进程和容器
```

急停后使用 `goal` 或 `explore` 显式重新开始任务；暂停或重置 Gazebo 时钟后使用 `reset` 重建会话。
关闭 Gazebo 或 RViz 会关闭本次仿真栈。两种导航模式互斥，不允许额外运行导航客户端竞争控制权。
仿真使用 `continuous_exploration: true`，无时间、位移和累计转角预算。前沿筛选保持官方最大轮廓及超过20格门槛，无候选时恢复全图随机目标回退。手动停止或传感器、定位和控制故障时停车并保存地图。
仿真位姿来自物理世界真值，不模拟 FAST-LIO；Agent 只读取 RGB-D 与定位，不读取点目标模式的已知地图。

参数入口：[simulation.yaml](src/conavbot_sim/config/simulation.yaml)。修改后执行 `compile`，再 `reset`。
`motor.command_delay` 控制纯延迟；`kp/ki/kd` 为轮速 PID，`max_torque` 为轮端堵转力矩，
`max_wheel_speed` 为空载转速，`torque_time_constant` 为力矩一阶响应时间。车体、相机、房间与障碍也在同一文件。
仿真默认速度由 `navigation.forward_speed=0.25 m/s`、`navigation.turn_speed=0.50 rad/s` 控制，两种模式共用；Motion上限为0.30 m/s、0.60 rad/s。仿真生成独立 YAML，不覆盖实车速度参数。
仿真启动脚本统一加载 `scripts/sim/fastdds.xml`：Fast DDS 每个参与者的共享内存段为64 MiB。
默认512 KiB小于640×480 RGB/浮点深度单帧，会造成大图像丢失与间歇卡顿；容器的1 GiB `/dev/shm` 总容量不能替代这个DDS配置。
修改该XML后执行 `reset` 重新加载，无需编译；配置仅作用于仿真入口。

RViz 显示规划路径、Agent 地图、图像、轨迹和电机遥测。遥测顺序为左前、左后、右后、右前；
`/sim/wheel_target` 和 `/sim/wheel_measured` 为 JointState，`/sim/delayed_cmd_vel` 为延迟后的车体目标，
`/sim/ground_truth` 为物理位姿/速度。原始 `/sim/motor_state` 的 20 项依次为：仿真秒数、急停标记、
4 项目标轮速、4 项实际轮速、4 项实际力矩、收到的 vx/vy/wz、延迟后的 vx/vy/wz。

`status` 查看模式和进程；`logs` 查看日志；`record` 录制关键话题（Ctrl+C 结束录包）。

实时状态在单独终端查看，2 Hz刷新中文阶段、前沿数量、真实目标/距离、观测与计算耗时、扫描进度及Motion输出；Ctrl+C只退出查看，不停车：

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
bash scripts/start_sim.sh watch
```

再开一个终端开始探索，随时用stop结束：

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
bash scripts/start_sim.sh explore
```

需要停车时单独执行：

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
bash scripts/start_sim.sh stop
```

RViz的 `Exploration status and goals` 显示青色候选点、橙色实际缓存目标和小车上方英文阶段；Motor dynamics默认关闭，可按需勾选。随机目标回退明确显示为继续探索；短暂等待观测保留最近任务阶段，另显示等待时间。状态超过3 s失联会标记，旧速度不当作当前输出。

每次任务使用独立run_id，旧消息不会串入新任务；结束后保留停止原因，清除候选与目标。关键事件写入 `runtime_logs/sim/status/<run_id>/events.jsonl`。查看器不订阅图像、不发运动请求。
状态接口为 `/sim/session_status`（会话生命周期）和 `/sim/exploration_status`（2 Hz聚合JSON），均为可靠、transient_local的std_msgs/String；RViz使用 `/sim/exploration_markers` MarkerArray。`/conavbot/decision`增加实际前沿/目标坐标、目标来源、run_id和最终退出报告；规划与控制行为不变。
日志、生成模型、构建产物及实际依赖版本保存在 `runtime_logs/sim/`，不提交 Git。
临时卡顿诊断：`bash scripts/start_sim.sh diagnose --duration 120 --label rviz-on`。
记录期间自行切换图像显示或启动探索；该命令只订阅，不控制车辆，不保存像素、不做图像转换。
原始/适配后图像与内参的逐帧时间、定位、仿真时钟速率、Agent决策、Motion状态和结束时日志保存在
`runtime_logs/sim/diagnostics/时间目录/`；Ctrl+C可提前结束。记录器仍会接收图像数据，建议同时停用其他图像话题监测。
`events.jsonl`保留时序，`summary.json`汇总到达间隔；健康记录中的计时器间隔可辅助识别记录器自身调度卡顿。
容器仅共享本地 X11 与渲染设备，使用独立网络、ROS Domain 71，不挂载底盘串口。
2026-10-26 用户交互运行后确认探索仿真初步可用。已完成编译、静止图像链路对照和用户手动探索；未做自动运动验收、动力学辨识或实车验收。

### 仿真调试结论与实机迁移参考（2026-10-26）

#### 已交付的结构与边界

- 实车文档车型为 WHEELTEC `senior_mec_bs`。用户提供的 `/home/ujiu/Downloads/senior_mec_bs_robot.urdf` 给出四轮几何和关节，不能单独确定麦轮滚子接触、电机参数、减速比和控制板PID。仿真采用近似模型。
- 车体0.36×0.28×0.12 m、3 kg；轴距0.313 m、轮距0.355 m；轮半径0.05 m、宽0.04 m，每轮总成0.224 kg。每轮8个45°被动滚子，X型布局，轮地接触驱动车体。
- 执行链：Motion `/cmd_vel` → 固定0.20 s延迟队列 → 麦轮逆运动学 → 100 Hz四轮速度PID → 轮端力矩 → 物理接触。PID为Kp=0.08、Ki=0.10、Kd=0；堵转力矩0.8 N·m、空载转速20 rad/s、力矩响应时间常数0.05 s，均为待辨识假设。
- 新指令不会重置旧指令的延迟计时；普通零指令也经过队列，急停清空队列并归零目标，仍需物理制动。场景重置清除旧任务、延迟队列与PID状态。
- `goal` 使用房间配置生成的已知地图、膨胀障碍和FMM；`explore` 使用原Agent的RGB-D建图、前沿和FMM。两模式只由一个路径发布者占用Motion；切换会停止旧任务，位置保留，重新探索创建新地图；`reset` 才恢复初始场景。
- 两模式共用Motion的20 Hz路径跟踪、0.25 m前视与约1 m短路径。Motion不建图、不识别动态障碍，也没有独立雷达避障层。仿真目标导航忽略最终朝向。

#### 环境与接口

- Fedora宿主运行rootless Podman；容器为Ubuntu22.04、ROS Humble、Gazebo Fortress、RViz，独立网络/IPC、ROS Domain 71、`IGN_PARTITION=conavbot_sim`，不挂底盘串口。Intel Mesa渲染可用；工具看不到显卡曾是工具访问范围问题，不能据此判断宿主缺GPU。
- Python为x86_64 CPU环境，Torch2.2.2、Open3D0.18、OpenCV4.10；锁定依赖见 `scripts/sim/requirements.txt`，实际镜像和系统/Python版本见 `runtime_logs/sim/{image-id,apt-versions,python-versions}.txt`。这套环境不能直接替代Jetson环境。
- 构建中已修复Ignition插件开发包名、OpenCV headless版本冲突、SDF visual/collision重名、RViz Marker话题属性、目标节点入口及容器退出/缓存路径问题。复现使用现有Containerfile和脚本，不重建旧失败配置。
- 相机640×480、15 Hz、地面高度0.30 m，输出D455四路契约：RGB `rgb8`、对齐深度毫米 `16UC1`、两路匹配内参；光学帧 `d455_color_optical_frame`，图像与内参同戳。
- `/sim/odometry` 提供真实底盘位姿，再换算 `/Odometry` 的body参考点；`T_world_body = T_world_base × inverse(T_body_base)`。模拟外参来自模型，实车D455外参仍未确认；FAST-LIO、传感器噪声和真实定位漂移未模拟。
- Gazebo时间经固定起点映射到ROS墙钟，启动等待渲染预热及接近实时速率后才固定起点；后续不持续重打戳。暂停、倒退或重置需要结束旧任务并重建会话。实车应保留真实采集时间和同一时刻位姿配对，不照搬仿真时钟映射。

#### 图像卡顿根因：必须保留的DDS配置

原先RGB连续显示一两秒后卡顿，关闭RViz图像后有所缓解；原始图像约12 Hz，适配后约5 Hz，小消息/定位基本连续。关闭显示只能减轻负载，最终根因是Fast DDS默认512 KiB共享内存段过小：640×480 RGB约900 KiB，浮点深度约1.17 MiB。

现已固定 `scripts/sim/fastdds.xml` 每参与者64 MiB，`inside.sh` 设置 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 和 `FASTRTPS_DEFAULT_PROFILES_FILE`。容器1 GiB `/dev/shm`总容量与DDS段大小是两个配置，必须分别满足。仅修改DDS配置的静止对照如下，图像分辨率、QoS和适配逻辑均未改变：

| 指标 | 修复前25秒 | 修复后30秒 |
| --- | --- | --- |
| 适配后RGB接收频率 | 5.36 Hz | 15.1 Hz |
| 适配后深度接收频率 | 4.92 Hz | 15.1 Hz |
| RGB最长接收间隔 | 0.860 s | 0.091 s |
| 深度最长接收间隔 | 0.798 s | 0.092 s |

证据目录为 `runtime_logs/sim/diagnostics/20261026-010223` 和 `20261026-010308`；初次120秒诊断在 `20261026-005658`，当时仿真实时倍率0.984～1.003，排除了持续整体降速作为主要原因。[Fast DDS 2.6官方说明](https://fast-rtps.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html)记录了默认段大小及小段引发数据丢失的风险。

实机若同样使用Fast DDS传输大图像，应按实际RMW和传输拓扑配置发布与订阅进程；当前XML仅由仿真入口加载，实车入口尚未接入。跨机器通信还要单独检查网络传输，不能套用同机共享内存结论。

#### 停车原因与当前时效设置

| 本次出现的原因 | 已确认含义与处理 |
| --- | --- |
| `PROCESSING_EXPIRED` | 曾因输入年龄190 ms加计算123 ms，共313 ms超过300 ms而退出；该规则是10月12日WHEELTEC适配层加入，非官方探索算法要求。已按用户要求放宽。 |
| `navigation request timeout` | 路径时间戳取计算开始；143 ms计算已消耗原300 ms有效期的一半。客户端和Motion两端现统一3 s。 |
| `NO_NEW_OBSERVATION` | 无新的完整可用观测持续到等待上限；不等同于相机停止发布，可能是丢帧或配对失败。DDS修复解决了本轮主要图像传输问题。 |
| `duration completed` | 一次正常运行满原60秒预算退出；该仿真预算已取消。 |
| `MOTION_BUDGET_REACHED` | 提速后一次距起点4.014 m超过4 m上限，累计转角19.961 rad；该仿真预算已取消。原位移预算是距起点距离，不是累计路程。 |
| `NO_FRONTIERS` | 曾新增为空候选即完成，817帧后触发；随后按用户要求撤销，当前恢复官方随机目标回退。 |

当前图像接收/观测年龄、单轮处理、Agent子进程响应、无新观测等待、位姿有效期、导航请求和客户端状态有效期均为3 s，服务调用等待5 s；历史位姿缓存1024条。RGB/深度配对30 ms、图像/位姿配对50 ms、未来时间容忍100 ms保持不变；启动扫描Action仍为2π、45 s超时。

**这些3 s修改已写入共用Agent源码、Motion默认值及实车 `motion_control.yaml`/example，并非全部仅限仿真。** 但速度提高只在仿真配置生效：实车仍为0.15 m/s、0.35 rad/s上限。3 s时效并不是实车避障保证；例如0.15 m/s持续3秒对应0.45 m运动。实机部署须核对控制板失联停车、实时定位和雷达是否真正能约束运动输出。

#### 前沿策略：保留官方基线，避免把筛选为空当作探索完成

- 官方参考是 `ybgdgh/Co-NavGPT2`，提交 `891ba09959278b666ef01a7f8b6c64f8fe8436cc`，本地 `../conavgpt2_ws/upstream/Co-NavGPT2`。前沿工具文件与该参考相同。
- 已探索地图5×5闭运算 → 最大轮廓 → 去除7×7膨胀障碍和地图边缘 → 八邻域连通段必须超过20格 → 质心按FMM距离排序 → 最近候选。没有候选时在整个地图随机选目标再规划，不预先保证可达；24 m地图可能抽到8×6 m房间外。
- 一次保存地图中，最大轮廓经障碍过滤只剩15和18格两段，均被20格门槛过滤。保留全部轮廓时有6段超过20格，但尚未证明全部可达。不能仅凭这次结果把最大轮廓策略判为错误，或把空候选等同全图完成。
- 当前保留最大轮廓与20格门槛，恢复随机回退；不实施降低至10格或保留全部轮廓的优化。用户要求优先还原官方行为，优化后置。
- 仍有既存适配：当前保持前沿/随机目标直到到达或无可用路径，每次建图更新通向该目标的路径；官方单Agent入口每次建图重新选目标。FMM约1 m路径输出、底盘坐标换算与Motion跟踪也属于适配，不能称整个系统与官方逐行一致。

#### 实机落地顺序与未完成项

1. 固定D455/MID360安装，确认相机到底盘、FAST-LIO body到底盘外参及时间戳契约，再接真实观测。用采集时刻的历史位姿投影延迟图像；旧图像可以用于静态建图，但不代替实时障碍制动。
2. 先核对实机图像频率、到达间隔、采集年龄及DDS配置，再启用Agent。一次只运行必要的只读记录器；多个图像`hz/delay`订阅会增加大消息负载。小消息正常而图像丢失时优先检查大消息链路。
3. 运动仍由Motion发 `/cmd_vel`，实车控制板执行电机PID；不把仿真轮端PID参数复制到控制板。实际辨识指令延迟、刹停距离、电机限幅和闭环响应，再调整仿真模型。
4. 用户提到实车有雷达快速更新障碍，但当前项目Motion没有雷达避障输入。独立局部避障/截停链是否存在、是否真正控制输出，仍需核对与接入。当前FMM对未知区的处理也不能替代这条链。
5. 仿真探索已由用户确认初步可用，覆盖完整性、复杂动态障碍与实机运行未验收。不要自动发送实车运动请求。
6. “终端中文实时状态＋RViz当前目标/前沿＋事件日志”已实现：使用 `watch`，见上方实时状态说明。完成静止显示、实际观测只读Agent字段、Ctrl+C和手动停止状态检查；运动中的显示由用户启动探索观察。

后续需要用户操作时，给出从完整工作目录开始的全部可复制命令，不要求回翻聊天。运行中的故障先看最新 `agent/report.json` 的 `exit_reason` 和 `motion_status_before_stop`；`task.log`为追加文件，末尾附近可能包含旧运行错误，不能混作最新原因。`agent/report.json`和`map.npz`会被后续运行覆盖，重要现场应先归档；诊断目录有独立时间戳和结束时日志副本。

桌面重新登录后，旧容器可能引用已失效的 `.mutter-Xwaylandauth.*`。使用当前桌面终端重建容器，源码和挂载日志保留：

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
bash scripts/start_sim.sh down
bash scripts/start_sim.sh compile
bash scripts/start_sim.sh up
```

编译只更新安装文件，运行进程仍需重启；已有会话用 `reset`，无会话用 `up`，避免对不存在的会话执行reset。实机运行时需同时部署两个工作区的对应源码与接口；本轮改动尚未提交或推送。

两个仓库放在同一父目录，已有目录先检查Git状态和远端，不覆盖修改：

```bash
git clone https://github.com/YOOJOR/wheeltec_motion_ws.git
git clone https://github.com/YOOJOR/conavgpt2_ws.git
```

车载环境为 Ubuntu22.04 / ROS2 Humble，Jetson 依赖见导航 README。部署前检查 `scripts/robot_stack/config.yaml`、导航 `conavbot/config.yaml` 及 `CONAVBOT_CONAV_ROOT` 中的路径。

外部工作区：`wheeltec_base_ws` 为厂家通信包，`vendor_ws` 为 Livox / FAST-LIO。部署版本：

| 组件 | 锁定提交/注意事项 |
| --- | --- |
| 厂家三包 | 本地基线fe99b01；不上传厂家源码，按车型和设备配置 |
| Livox-SDK2 | 08f523c930b2f0ba1e98a6afaa8d7476bf479908 |
| livox_ros_driver2 | 4a1def929e5b59c7a8122d19fce6efba581ce9f7 |
| Ericsii/FAST_LIO_ROS2 | 2fffc570a25d0df172720bac034fbdb6a13d2162；ikd-Tree e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4 |

本车Livox主机IP为192.168.1.5；历史src雷达IP为.45、实际install为.145，重建前必须核对，不能假定源码等于运行配置。FAST-LIO已有部署关闭内部外参在线估计，内部LiDAR/IMU变换与底盘偏移分开。SDK旧构建缓存无法证明安装二进制来源；新车重新构建，不复制build/install/.venv。vendor增量构建显式选包，不能使用会清整个install的厂家build脚本。新车重新测量外参并从禁用模板开始。

准备 Humble 及依赖后，执行 `bash scripts/build.sh` 构建；部署更新后重启节点。

## 现场操作指南

以下在已部署的小车图形桌面执行，需要 GNOME Terminal。统一入口默认打开五个标签页：底盘、Livox 雷达、FAST-LIO 定位、运动控制接口、D455 驱动；默认不启动 RViz。

```bash
bash scripts/start_robot.sh                       # 默认 D455，无 RViz
bash scripts/start_robot.sh --rviz                 # D455 并开启 RViz
bash scripts/start_robot.sh --status               # 查看各组件状态
bash scripts/start_robot.sh --stop                 # 关闭整套受管组件
```

统一入口仅保留 `start_robot.sh`；`--config /路径/config.yaml` 指定部署路径与默认设置，`--check` 只做环境检查。`--no-rviz` 可覆盖配置中的启用设置。默认值见 [config.yaml](scripts/robot_stack/config.yaml)。不要同时另开相机或基础栈；脚本会启动硬件，空闲控制器持续发零速度，不发送探索目标。

组件进程退出或就绪超时后按依赖恢复；相机单独恢复，不重启定位。相机标签显示驱动数据流状态；导航报告 `NO_USABLE_DEPTH` 不触发相机重启，`STACK READY` 不等于深度有效或允许探索。关闭任意受管标签页或在其中按 Ctrl+C 会停止整套组件；总控先停止运动接口。

Agent 单独启动；D455外参仍为空，配置并核验前拒绝世界建图。RViz 使用 `bash scripts/start_debug_rviz.sh`，`--minimal` 关闭图像/点云显示。地图/决策为 `/conavbot/*`。

FAST-LIO提供camera_init→body，RViz安装TF读取共同配置提供底盘链，Agent提供camera_init→conav_map。控制与建图不依赖RViz；不要叠加厂家整车TF。迁移坐标或定位重启后结束旧任务、退出旧进程并清地图。只读reset为 `/conavbot/reset`；运动中reset终止本次运行。

有限运动示例仅在明确授权、现场观察和可随时停止时执行；会实际移动：

```bash
bash scripts/start_conav_exploration.sh --start --motion --allow-provisional-camera --duration 45 --map-size-cm 600 --input-stride 4 --forward-speed 0.05 --turn-speed 0.25 --max-distance 0.2 --max-rotation 8.0
```

临时允许相机几何不等于标定通过。预算为定位估计距起点位移和累计绝对转角，不是精确物理边界。当前持续探索未通过，不直接扩大范围。

停止：运行终端Ctrl+C，或加载环境后明确调用stop，并观察车体：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
python3 -m wheeltec_motion_control.navigation_client stop
```

确认IDLE/零输出及实际停稳后，用 `bash scripts/start_robot.sh --stop` 关闭受管栈。受管 RViz 标签页也参与统一停止；单独启动的 RViz 独立关闭。手柄测试先退出受管控制栈，避免自动重启控制器及空闲零输出冲突。

## 诊断与代码核查

- NO_USABLE_DEPTH：先看原始深度、范围内比例及输入计数，不能等同于传感器停流。
- PROCESSING_EXPIRED：分别查看input_age_sec、agent_sec、source_age_sec和processing_sec，不凭名称认定FMM慢。
- 请求超时：核对新观测/新请求是否持续，当前有效期3 s，不重发旧决定。
- 定位异常：核对camera_init/body、时钟、源年龄和重启；`topic delay`包含扫描/处理/调度，不等于网络耗时。
- 启动失败：看 `runtime_logs/robot_stack/` 下supervisor及组件日志。通信故障保留定位；Livox/FAST-LIO故障按依赖重启，旧任务必须结束。

只读记录器（需要已启动的输入）不保存像素、不启用运动；额外订阅可能增加负载：

```bash
python3 scripts/record_conav_acceptance.py --duration 60 --sensor-timing --output runtime_logs/conav/acceptance/new-run.json
```

离线旋转拟合：`ros2 run wheeltec_motion_control calibrate_rotation --bag /绝对录包目录 --output /新的输出目录`；也支持 `--csv`。工具不发速度、不改配置；平面旋转不能独立确定高度或厂家几何中心，`--body-z`须来自测量及坐标换算，建议不得直接覆盖车辆YAML。

软件测试及临时模拟代码禁止新增或运行，具体约束见 AGENTS.md。日志、录包和地图不提交 Git。
