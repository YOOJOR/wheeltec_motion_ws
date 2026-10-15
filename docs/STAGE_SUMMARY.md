# 阶段总结：运动接口与原版 Co-NavGPT2 探索适配

## 2026-10-15：厂家 base_link 坐标迁移

当前控制参考点为厂家四轮几何中心 base_link，base_footprint 在其下方0.026m。相机采用厂家 senior_mec_bs URDF 的 camera_joint，base_link 下平移[0.16083451891873,0.000745202407130424,0.089704]m、零RPY，组合离地0.115704m。光学轴转换保持显式；confirmed=false表示物理投影仍待验收，不再使用人工高度覆盖。

MID360S为后装：body_from_base=[-0.10718,-0.02237,-0.17788]m、零RPY；XY经用户同意暂作安装估计，来自旧旋转拟合，未独立测量到厂家几何中心。厂家laser_link和通用radar不用于FAST-LIO。旧记录中的wheeltec_base/地面原点及-0.20388为历史配置，以本节为准。没有改变速度/加速度/超时、深度范围、相机内参或FAST-LIO内部外参。


控制基线：**v0.3.0，ROS 2 Humble / Ubuntu 22.04**；部署更新：**2026-09-28，FAST-LIO 已迁入 vendor_ws**；**2026-10-01，持续速度导航**；**2026-10-12，原版 Agent 有限实车探索链路通过**。本文是后续开发的统一上下文，可直接据此继续工作；无需阅读早期聊天、逐次开发日志或原始测试输出。只有用户明确要求追查历史问题时才读取旧记录。总体规划仍为开发机 `document/Co-NavGPT2-baseline复现路线图.md`。

## 1. 当前结论与工作边界

**2026-10-13 最新测试：原始图像持续，但视野内的原始深度读数全部落在当前奥比接受范围外，导致RGB-D停止输出。** 用户在场，限定20分钟内结束；沿用60s/0.5m/10rad预算及0.05m/s、0.25rad/s。首次未在2s内发现导航服务，未发送运动请求；只读接口检查正常后重启测试，实际发出26条新鲜左转请求，随后控制器navigation receive timeout，上层MOTION_INTERFACE_LOST并stop成功。没有完成一圈或进入前进，持续探索仍未通过；最终IDLE/零，用户确认实际停稳无异常，本轮硬件全部关闭。

新增只读record_conav_acceptance.py --sensor-timing，按源时间关联原始彩色/深度与原子RGB-D到达，不保存像素。断更后的0.5s内旁路仍收到16帧彩色、15帧深度，而RGB-D为0；适配器accepted停在4881、NO_USABLE_DEPTH拒绝持续增加。停车后的3帧原始16UC1深度约24–25%像素有正值，全部大于3m，0.4–2m内比例为0；现配置depth_max=2.0因此过滤所有点。远距离数值尚未独立测距验证，未修改深度范围、运动参数或失效门槛。此次原因不等同于10月12日263ms输入延迟，也不能认定所有既往问题均已解释。下一步优先核实远距有效性与2m截断策略，并区分“新帧没有近处有效点”和“传感器断流”，再验证持续探索。

**2026-10-12 进一步验收：持续实车探索尚未通过，当前优先解决运动时的图像迟到。** 用户重新在场，测试限制60s、距起点0.5m、累计转角10rad，速度仍0.05m/s和0.25rad/s。两次进入运动均因PROCESSING_EXPIRED停止，中间一次在导航启用服务响应阶段退出、未发速度。最后一次95条新请求后，第96帧进入Agent时图像已0.263s旧，计算0.054s，完成时图像年龄0.317s，触发原0.3s门槛。定位年龄最大约0.081s；不能把问题归为FMM计算慢，尚需分清相机采集、传输与配对延迟。最终停止服务成功、输出零、IDLE；本轮栈和相机已关闭，用户确认最后一轮实际停稳、无其他异常。

修复了两项接口问题：服务发现与响应原共用2s，发现耗尽后仍发送却零等待；现每阶段各有有限等待，不重发请求、不改速度超时。文档中的python -m navigation_client stop原缺少main入口，现可实际执行并返回失败码，隔离及真实停车调用已验证。新增过期帧完整诊断；探索消费者不再强制相机发布端DDS配置，相机32MiB配置保留。静止25/45/40s均通过；2000步数值房间252次扫描后1748次规划、607次前进，无模拟碰撞拦截；边界/故障重置验证通过。详细证据与未通过结果见VALIDATION最新节。下一步先同步记录运动时原始彩色/深度、原子RGB-D、位姿与配对队列年龄，再决定修复，不放宽保护或盲目加速。

**前一轮结论：主开发已迁到原版 Co-NavGPT2 源码，完成有人现场观察的有限运动探索链路验收。** 开发目录为 `~/workspace/conavgpt2_ws/development/Co-NavGPT2`，分支 `wheeltec-adaptation`，官方基线 `891ba09959278b666ef01a7f8b6c64f8fe8436cc`；upstream 保留参考，旧 wheeltec_exploration 仅作测试。当前直接调用原版 ROS_Agent 的点云建图、前沿、FMM 与动作方法，目标识别/VLM关闭。硬件坐标、时间配对、启动扫描与持续速度接口在边界适配，源码差异逐项见 WHEELTEC_ADAPTATION.md。

源码交付提交 `a1cc638`；部署仓库保存development.lock.json与可重建补丁，适配源码及本车教程/验证均有本地Git版本，不自动推送。

实车使用奥比 ASTRA S，6 m 地图、像素采样步长4、相机接口发布上限20 Hz；前进0.05 m/s、转向0.25 rad/s。本轮最终完成定位累计6.296 rad的启动扫描，随后原版产生170次 PLANNED，新鲜导航请求556条；达到有限运动预算后stop成功、控制状态IDLE、最终输出零。定位估计前后底盘位移约0.215 m；用户确认左转一圈、转向、前进均发生，最终实际停稳且无异常。不是尺测位移或物理停车时间测量，也不是完整房间覆盖、独立避障或语义任务验收。

此前两次实车尝试因请求源时间过期退出，失败记录保留。当前请求时间使用开始处理一组新观测的时间，图像采集时间独立保留；禁止慢计算后刷新时间、重发旧决定。图像年龄/处理各0.3 s、控制器请求0.3 s、定位0.5 s及50 ms配对门槛没有放宽。独立ROS域的合成测试覆盖源码扫描→规划→控制输出、断流停止、恢复不重放、延迟与过期图像、预算退出及控制权；相机限频也已验证。见 VALIDATION 的最新记录。

运动控制配置未改；奥比地面参考高度仍0.1157 m，完整外参未标定确认，confirmed=false。D455独立接口保留，本轮未使用。收尾时已关闭本轮基础栈、相机、探索、记录与RViz，不继续运动。下一步是有人现场的更长探索、地图与地面障碍投影检查、边界/避障和物理停止验证；检测/VLM、多机仍未接通。当前操作入口以 [SOURCE_AGENT](SOURCE_AGENT.md) 为准；下列日期段落保留此前阶段事实，不能当作最新运行状态。

2026-10-12 论文策略收尾验证完成：独立 ROS 域188、合成输入的最终探索回归通过；前沿提取直接对比锁定原版方法一致。300 步数值房间先输出252次扫描建议，位姿累计6.30 rad后切到48次有效规划，没有把静止或旧帧计作扫描完成。修正已到达前沿重复选择问题，实际ROS验证地图层/标记发布、扫描→规划、故障清图与显式reset。全部实车程序保持关闭，本轮没有启动硬件或发运动。当前软件仍为只读探索，完整自主运动未接通；静止调试用 `--skip-initial-scan`。证据与局限见 VALIDATION。

2026-10-10 探索策略更新：按用户核对论文后的要求，默认改为 paper_frontier：新增点投影 explored 层，按最大轮廓减膨胀障碍提取前沿，随机选择候选替代 VLM；无有效前沿时全图随机候选兜底，允许未知参与 FMM，并保留障碍/连通/限次检查。新增按新鲜位姿累计一圈的启动扫描（45 s 超时），不照搬原版处理步数。默认启动仍只读，不会驱车，静止调图使用 `--skip-initial-scan`；RViz 新增前沿和扫描进度标记。此前 known_free 策略可配置对照。完整对应关系与保留差异见 EXPLORATION_POLICY，验证见 VALIDATION。本轮开始已按用户要求关闭全部实车程序，开发期间保持关闭。

2026-10-10 新增只读可视化入口 `bash scripts/start_debug_rviz.sh` 和仓库内 RViz 布局：camera_init 俯视地图、前沿/FMM 目标、FAST-LIO IMU 位姿/轨迹、奥比原始 RGB/深度，雷达点云默认关闭。可选 `--camera d455` 只切换独立图像输入，不提供相机安装 TF。入口不启动硬件、规划或运动，无需构建，使用与调试见 VISUALIZATION；验证见 VALIDATION。

2026-10-10 最新修复结果：奥比大图像传输改为相机进程专用 Fast DDS 32 MiB 共享内存段，图像后端在订阅前初始化；RGB-D/位姿配对新增一组新鲜等待帧，并及时替换已无法配对的等待帧。最终 5 分钟真实静止整链 RGB-D 约 28.81 Hz、最大接收间隔 0.123 s，无 STALE；探索持续处理 2480 组、推进最大间隔约 0.331 s，未冻结、地图保留，配对差仍 ≤50 ms。最终相机与探索回归通过，速度采样全零，未发运动。本轮相机/配对连续性通过；当前场景仍无可达前沿、未产生 PLANNED，后续检查自由空间/视野并接实机速度出口。不是完整自主探索验收。此前 70 s 和中间实现的失败记录均保留，下文是阶段历史，当前以此段及 VALIDATION 为准。

2026-10-10 最新整链测试：基础动作验收后，真实奥比 RGB-D + FAST-LIO + 随机前沿/FMM 只读预览运行 70 s。实际处理 48 组，均未找到可达前沿；随后观测源年龄超过 0.5 s 触发 STALE，清地图并锁存，虽然后续新帧/配对恢复也未自动重启探索。定位年龄最大约 80 ms、处理耗时最大约 52 ms，采样速度全零。本轮未产生 PLANNED、未接自主运动桥、未用 VLM，完整项目仍未通过连续性验收。下一步先处理观测间歇和场景前沿，再接实机运动出口；见 VALIDATION。

2026-10-10 用户恢复雷达后完成三段低速实车相对动作：前进目标 10 cm / 左右各 15°，定位估计完成约 8.39 cm / 左 13.79° / 右 16.21°，均在当前容差内返回成功，用时约 3.73 / 5.11 / 6.19 s。结束显式停止，连续采样输出零并关闭本轮服务。首次启动定位延迟经受管重启恢复；动作期间位姿年龄最大约 58 ms。非零指令到定位可见明显运动约 1.9–2.4 s，远长于本轮指令斜坡约 0.1–0.15 s，具体来源尚未区分。相机/探索没有参与，11.57 cm 外参和地图运动一致性仍待验证，不代表完整 baseline 已通过。细节及指标定义见 VALIDATION。

2026-10-10 运动验收尝试：用户授权立即开始。底盘通信就绪，但雷达网口 DOWN / carrier=0，Livox 初始化失败，没有定位输入，尚未发送运动目标。已关闭本轮服务，等待恢复雷达连接后继续低速前进/转向验收；不是运动测试通过。

2026-10-10 当前相机配置：按用户决定保留地面有效旋转中心 `wheeltec_base`，奥比预览 Z 采用 **0.1157 m（11.57 cm）**，XY/轴向及运动控制外参保持原值。来源为假定厂家相机高度相对 base_link 后的 `0.0897 + 0.026`；实际厂家父坐标系及完整组合未核实，保留 `confirmed=false`，不宣称物理标定完成。下文 0.115 m 为前一轮诊断时使用的历史值。

2026-10-09 固定目标补充复查：用户说明上一轮板子为手持，现已靠架固定；尺测镜头离地约 0.115 m、板距约 0.57 m。只启动奥比相机采样约 45 s，997 组、约 22.47 Hz，中心深度 0.610–0.611 m，下沿约 0.565 m，上部约 0.67 m；需按板面同一测点比较，不能把中央与尺测距离之差直接定为传感器误差。5 个快照中板体→阴影的彩色边缘约第 395 行、深度边缘约 393 行，较大偏移未复现。单相机采样最大帧间隔 0.222 s，未见 STALE，但本轮未带定位/探索负载，不能宣布此前间歇已修复。预览相机 Z 改为用户实测 0.115 m，保留 factory XY/轴向和 confirmed=false；固定板下沿投影仍约 -0.0255 m，安装倾角/参考点/内参需继续核验。无运动控制节点或运动请求，详细记录见 VALIDATION。

2026-10-09 用户在实验室明确授权代理执行本轮现场验证，包括静止合格后低速前进 0.2 m、左右各 15°。实际停留在静止阶段：发现相机观测间歇导致预览两次 STALE 锁存、清地图，以及相机投影/配准疑点，因此没有发送任何运动目标。启动定位曾出现约 1.396 s 源年龄，受管重启后恢复；随后约 9.6 分钟监测中位姿年龄中位数 34.5 ms、最大 90.6 ms，11565 条采样速度全零。原场景有效深度约 2.27%，放入纸箱后约 54.39%；单帧箱体边界及相对地面高度存在疑点，外参仍未确认。静止验收未通过，移动验收未执行。停止请求已发送，结束时关闭本轮启动的服务；结果及下一步见 VALIDATION 的 2026-10-09 节。

2026-10-02 后续推进：新增 `wheeltec_exploration` 只读单车预览，接自研 RGB-D / FAST-LIO 时间配对、暂定厂家相机矩阵、世界 XY 栅格、随机可达前沿和锁定官方 FMM，发布地图/目标/方向建议；入口没有运动发布者或客户端，不启动硬件、不接 VLM。默认奥比，D455 安装未确定，世界预览明确拒绝空外参。准备好的 DecisionBridge 仅以合成位姿和测试话题验证，尚未接到现场探索入口。本轮没有读取真实相机或让车移动。可选本地 YOLO-World/MobileSAM 接口及目标深度投影已准备，新分支真实模型链/识别性能待验收；默认仍为几何随机探索。

软件证据：合成房间 300 帧产生地图、前沿和 FMM 建议，观察到模拟墙；隔离 ROS 输入验证断流/跳变/计算超时、清地图、显式 reset 与真实控制节点测试出口。全仓构建和已列回归通过，但不证明真实投影、避障、物理停车或完整 baseline 已跑通。使用与现场验收以 [EXPLORATION_PREVIEW](EXPLORATION_PREVIEW.md) 为准，验证意义以 VALIDATION 的输入→输出→局限表为准，数量仅作漏跑检查。

2026-10-02 D455 已接入 Orin（USB 5 Gbps），SDK/ROS 驱动/Viewer 工具齐全，已验证图形窗口并添加桌面快捷方式。新增独立 D455 话题与光学帧，相机消费端支持 camera=astra_s/d455，默认仍为奥比；用户决定先用奥比跑通 baseline，再换 D455。D455 最终独立接口静止读取 109 组、约 13.82 Hz，0 拒绝、结束 READY；49 项相机相关回归测试通过，安装外参/世界建图仍待验收，见 D455_INTERFACE。

2026-10-01 后续接入：新增只读 PosedRGBDReader，将 RGB-D 深度时间与 FAST-LIO 位姿配对，并通过显式已确认外参构造世界相机/底盘变换；定位异常锁存、清缓存，恢复要求上层清除旧地图并显式重置。已编译安装，43 项相机相关测试通过（含新增 17 项），完成合成配对、坐标轴和 ROS 数据链验证，真实相机外参仍待确认，未建立实车世界地图。详见 CAMERA_INTERFACE。

2026-10-01 相机接口已新增并编译安装：仅复用 Astra 底层驱动，自研 wheeltec_rgbd 提供原子 RGB-D、内参、状态和最新观测读取；不使用厂家完整导航/跟随栈。当前 SDK 返回 NaN 内参，适配层显式加载本机现有 rgb_camera.yaml/ir_camera.yaml 并去畸变，标记文件与设备归属尚未核验。最终 8 秒静止取流接收 128 组、约 21 Hz，最大源年龄约 0.171 s，结束 READY；87 项包测试（61 控制＋26 相机）通过，15 项启动管理通过。相机光学坐标系的真实观测已通过 Co-NavGPT2 点云函数检查，尚未接世界位姿或完成自主探索。操作与标定边界见 [CAMERA_INTERFACE](CAMERA_INTERFACE.md)。

总目标为复现 Co-NavGPT2，基础运动接口服务于后续视觉语义导航。2026-09-29 用户调整优先级：暂缓运动参数试调，先推进 Co-NavGPT2 软件部署；车上当前未连接 D455。官方源码、独立实车依赖和三组模型已准备在 `~/workspace/conavgpt2_ws`，通过无硬件离线检查与 GPU 合成输入推理。2026-10-01 在现有 motion 节点新增持续速度导航接口和最小适配器，完成隔离软件验证；完整探索仍待相机、TF、上游出口适配与用户实车验收。软件部署说明为 conavgpt2_ws/README.md，速度接入契约见本仓库 NAVIGATION.md。

当前已完成“FAST-LIO 位姿反馈 → 相对运动 Action → 原厂底盘通信”的基础闭环，并经用户初步实车验证。能够前进/后退、原地左右转向、停止，提供进度与结果、参数配置、离线底盘偏移标定和两种启动/恢复方式。

2026-10-01 相机检查：本机已连接 **Orbbec ASTRA S**（2bc5:0402，USB 2.0），D455 仍未连接。已有 OpenNI SDK 可读彩色＋深度 VGA 约 29 fps，HD 彩色也可输出；短距结构光的标称范围为 0.4–2 m。可先推进室内 RGB-D 软件接入，但真实覆盖、内参/配准/时间同步及相机外参/TF 尚未验收，不能直接套用 D455 启动脚本或参数。详情见 [CAMERA_ASSESSMENT](CAMERA_ASSESSMENT.md)。

2026-10-01 厂家功能链路追踪：桌面相机命令读取 camera_mode=astra，默认 RGB-D VGA/30 fps，未开启深度到彩色配准和设备同步；单独相机入口不接通底盘 TF。厂家存在二维 SLAM/Nav2、RTAB、RRT 和跟随入口，RTAB pure3d 仍使用轮式 EKF，导航使用深度转 LaserScan。厂家速度输出、定位链和更高限速配置不能直接叠加现有控制栈。只读审计与复用步骤见 [FACTORY_CAPABILITIES](FACTORY_CAPABILITIES.md)，未启动厂家整车或运动功能。

当前已通过原版 Agent 的有限实车探索链路验收，整体仍进行中：没有独立低层避障、旧地图重定位或多车协作；尚未完成总体规划 M1 的长期定位验收、M2 的完整 TF/重复精度验收及完整房间覆盖。不能把本次短程链路成功写成完整 Co-NavGPT2 语义任务已跑通。

用户已经明确决定：

- 不修改 STM32 固件；原厂通信协议和串口节点保持原样。
- 暂缓“0.5 秒内物理停车”要求；`pose_timeout=0.5` 是位姿有效性门槛，不是制动保证。总体规划早期的该项门槛当前不作为已通过项。
- 偶发定位延迟先依靠自动恢复继续推进；保留原因未确认的事实，暂不继续深挖。
- 常规自动验证采用合成话题/独立 ROS 域；用户在车边明确授权后，代理可在该次约定的低速、有限时长/位移/转角范围内试车，结束显式停止。离车期间不得自动移动。
- 只将自己的 wheeltec_motion_ws 上传 GitHub；厂家 base 不上传、不改源码。
- 相机仅复用底层 Astra/OpenNI 与现有标定文件，开发自己的观测接口；厂家完整功能包不作为自主探索运行栈。
- 用户于 2026-09-28 授权将 FAST-LIO 迁入 vendor_ws；迁移与静止接口验证由开发端完成，随后运动验证由用户执行。
- 用户于 2026-09-29 要求优先部署 Co-NavGPT2，当前不继续调整运动参数；响应问题保留为待办，不阻挡相机与导航软件准备。
- 用户于 2026-10-02 明确离车期间禁止车辆运动，D455 随意放置不用于调试；先完善只读软件、随机选择和文档，现场先用奥比验收。
- 2026-10-09 用户在车边授权本轮由代理操作验证；静止故障触发中止，实际未发运动目标。这项现场授权不等于今后无人监护时允许自动试车。

## 2. 硬件、环境与目录

第一台车：WHEELTEC **senior_mec_bs** 麦克纳姆底盘，支持原地旋转；MID360S 使用内置 IMU，雷达 X 朝车头。底盘通信的 `imu_mode=stm32` 表示厂家底盘自身 IMU 来源，并不改变 FAST-LIO 使用 Livox IMU。

小车用户 wheeltec；项目根 `~/workspace`。开发机 NFS `/mnt` 对应小车 `/home/wheeltec/workspace`。NFS 源码可读，但 install 内指向小车绝对路径的链接在开发机可能无法解析，不代表小车安装损坏。

| 工作空间 | 内容与职责 | 本次处理 |
| --- | --- | --- |
| wheeltec_base_ws | 厂家通信、消息、串口库三包 | 保持原样，源码已核对 45 文件无差异；只记录复刻过程 |
| vendor_ws | Livox 驱动、SDK2 源码、FAST-LIO；SDK 安装在 /usr/local | FAST-LIO 已迁入并重新构建；Livox 安装和 SDK 未重建 |
| archive/fastlio-migration-20260928/conavGPT_ws | 迁移前完整定位工作空间 | 仅用于回退，不自动 source |
| wheeltec_motion_ws | 自研 Action、控制、标定、启动管理与 RGB-D 接口 | v0.3.0 正式维护仓库 |
| conavgpt2_ws | Co-NavGPT2 官方源码、独立依赖环境、权重、部署检查 | 2026-09-29 完成无相机软件准备；尚未接通实车导航 |

来源锁定：

| 组件 | 来源 | 当前源码提交 |
| --- | --- | --- |
| 运动控制 | github.com/YOOJOR/wheeltec_motion_ws，main | v0.3.0 固定控制基线；main 含 2026-09-28 迁移部署 |
| 厂家底盘抽取 | 小车 /home/wheeltec/wheeltec_ros2/src | 独立本地基线 fe99b01f8e047f686aa2d004ddb38a829f909a3b |
| FAST-LIO | github.com/Ericsii/FAST_LIO_ROS2，ros2 | 2fffc570a25d0df172720bac034fbdb6a13d2162 |
| Livox 驱动 | github.com/Livox-SDK/livox_ros_driver2，master | 4a1def929e5b59c7a8122d19fce6efba581ce9f7 |
| Livox SDK2 | github.com/Livox-SDK/Livox-SDK2，master | 08f523c930b2f0ba1e98a6afaa8d7476bf479908 |

Livox 主机 IP=192.168.1.5；当前实际启动读取 install 配置的雷达 IP=192.168.1.145，而 src 配置是 192.168.1.45。两份未同步，本版本仅记录，不改第三方。重建驱动前先核对本雷达地址，不能假定源码等于当前运行配置。

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
| /wheeltec_motion/navigation/cmd_vel | geometry_msgs/msg/TwistStamped，最新底盘速度输入 |
| /wheeltec_motion/navigation/set_enabled | std_srvs/srv/SetBool，显式启用/退出导航 |
| /wheeltec_motion/status | wheeltec_motion_interfaces/msg/MotionStatus，模式/原因/请求与软件输出 |

底盘节点使用 `/dev/wheeltec_controller`，115200 baud。闭环控制器不打开串口、不发布 TF。不要把 body 改名当成 base_link，也不要向 FAST-LIO 的 body→地图链额外发布重复 TF。当前控制器内部换算底盘位姿；供相机融合/导航使用的完整 TF 链仍需后续明确唯一发布者。

计算关系：`T_world_base = T_world_body × T_body_base`，先做完整三维变换，再取平面 X/Y/yaw。`body_from_base_translation` 是底盘参考点在 IMU/body 轴下的坐标；`body_from_base_rpy` 把底盘轴旋转到 body 轴。当前 base 选为地面上的有效原地旋转中心，而不是外壳某个角点。

FAST-LIO 的 `mapping.extrinsic_T` 是雷达原点在 IMU 中的位置，与底盘偏移是两个独立参数。当前 FAST-LIO 沿用上游 T=[-0.011,-0.02329,0.04412]、R=单位阵，在线估计 false；没有使用相关 fork 提到的旧 T=[0.05512,0.02226,-0.0297]。MID360S 精确内部尺寸仍需按其型号资料核对，不能凭与 MID360 配置相同就声称已验证。

## 4. 控制原理和上层调用契约

`wheeltec_motion_interfaces` 生成 Action 和 MotionStatus 类型；`wheeltec_motion_control` 提供服务端、CLI、导航适配器和离线标定。任何具备该接口类型、处于同一 ROS 域/可达网络的 ROS2 程序都可调用；这里不是 HTTP API。

目标从**接受请求时的底盘位姿**定义。直行沿当时底盘 X 轴计算已走距离/横向偏差，用比例控制、航向修正、限速和加速度限制输出 Twist。转向累计连续 yaw，避免跨 ±π 的方向错误。进入容差后发布零速度并等待新的位姿满足稳定条件，才返回成功。

SETTLING 阶段继续发送 Action 反馈，但速度输出为零；终端持续反馈不等于持续要求移动。上层逐步等待 Result 时仍会等待停稳，影响序列耗时。Co-NavGPT2 持续探索使用下述导航速度模式，不反复提交/取消短 Action；本次没有实现连续轨迹跟踪。

持续导航契约（详见 [NAVIGATION](NAVIGATION.md)）：

- IDLE / ACTION / NAVIGATION 互斥；Action 预留或执行时拒绝导航切换，导航时拒绝 Action。唯一 20 Hz 输出定时器协调空闲零速度、Action 和导航，导航输入 depth=1，不积累目标队列。
- 启用后 WAITING 输出零速度；新请求直接替换速度目标，相同速度的新请求保持连续运动，不等待停稳。只接受底盘控制点 `wheeltec_base` 的 linear.x / angular.z，其他分量须为零；标签不代表 TF 已建立。方向纠偏/FMM 目标变换由上层负责。
- 源 ROS 时间严格递增且晚于本次启用；单调接收年龄与源年龄均不得超过 navigation_command_timeout=0.3 s，允许超前 navigation_future_stamp_tolerance=0.1 s。旧/重复包忽略且不续期；新的非法/越界请求退出导航并立即零速度。
- 位姿源/接收新鲜且连续有效，复用既有定位门槛；跳变、断流、超时、控制调度超过 max_control_gap 或异常均清空缓存并转 IDLE。导航 stop/退出也如此；必须显式重新启用并接收新的有效请求才能恢复，节点重启不续跑。
- 保留 cap=0.15 m/s、0.35 rad/s、当前加速度与安装外参；新零速度请求立即停车但保持 NAVIGATION/ZERO。命令零速度不等于实测物理停车。正常上层持续产生新决策，进入阻塞 VLM/长等待前先 stop，禁止换新时间戳循环重发旧决策或无限延长 timeout。

Action 定义位于 `src/wheeltec_motion_interfaces/action/ExecuteMotion.action`：

- Goal：motion_type=1（MOVE_LINEAR）/2（ROTATE）；target 的单位分别 m/rad；正数前进/左转，负数后退/右转。
- max_speed=0 使用配置上限；正数只能降低该动作速度，超过全局上限会拒绝。timeout_sec=0 使用默认动作超时。
- Feedback：phase、当前底盘 x/y/yaw、remaining、cross_track_error、elapsed_sec；直行 remaining 为 m，转向为 rad。
- Result：code、message、final_x/y/yaw、remaining、elapsed_sec；code=0 SUCCEEDED、1 CANCELED、2 POSE_INVALID、3 TIMEOUT、4 NO_PROGRESS、5 STOPPED、6 INTERNAL_ERROR。
- 同时只接受一个动作；忙碌、输出未启用、外参未确认、位姿无效、参数越界均可能拒绝。拒绝原因记录在控制器 `Goal rejected: ...` 日志中，客户端通用提示不包含完整原因。
- 上层先确认 accepted，再等 Result；只有 code=SUCCEEDED 才进入下一步。接受成功不等于运动完成，接口重新出现不等于旧动作成功。
- Ctrl+C 客户端会尝试取消已接受目标；直接 kill 客户端或客户端失联不等于停止请求。明确停止用 Trigger/`motion_client stop`。
- 不提供独立横移目标、避障或多速度源仲裁；lateral_correction 当前关闭。当前约定 `/cmd_vel` 仅有控制器一个发布者，键盘/导航不要并行发布。

## 5. 第一台车当前参数（响应试调）

2026-09-28 用户要求改善起步和收尾，第一档响应试调提交 dc6a68d 已部署到小车；用户确认已推送 GitHub。尚未收到本档实车运动反馈，不能认定响应已改善。迁移后的保守配置基线为提交 ff29eb7；五项参数的前后对比及回退方法见 OPERATIONS。最高速度、到达容差、静止速度阈值和定位门槛均未放宽；比较时先保持相同动作目标和 --max-speed。

调参背景还包括用户实际观察到的手柄迟滞：体感估计手柄到车体动作约 0.25 s，每隔约 0.4 s 切换左右转向时车体几乎不动。这些尚未同步测量；需区分手柄链路、指令滤波、底盘加减速限制和机械响应，不能直接归因于自动控制代码或硬件机动性，也不能把手柄迟滞与已知 FAST-LIO 时间年龄异常视为同一问题。

`src/wheeltec_motion_control/config/motion_control.yaml` 是第一台车当前配置，输出启用；example 保留原保守参数，作为新车的禁用模板。仓库不再假称实车 YAML 为零偏移占位值。

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| control_enabled / extrinsics_calibrated | true / true | 第一台车已启用并确认底盘偏移 |
| body_from_base_translation | [-0.10718,-0.02237,-0.20388] m | 底盘旋转中心在 IMU 中的位置 |
| body_from_base_rpy | [0,0,0] rad | 当前按 IMU 与底盘轴一致配置 |
| control_rate_hz / feedback_rate_hz | 20 / 5 | 速度更新与 Action 反馈频率 |
| max_linear_speed / max_angular_speed | 0.15 m/s / 0.35 rad/s | 全局上限，不是每个动作一定达到的速度 |
| linear_accel / angular_accel | 0.30 m/s² / 1.0 rad/s² | 加速限制；停止直接请求零速度 |
| distance_tolerance / angle_tolerance | 0.03 m / 0.035 rad | 后者约 2° |
| cross_track_tolerance / lateral_correction | 0.05 m / false | 当前不启用麦轮横移纠偏 |
| distance_gain / rotation_gain | 1.2 / 1.6 s⁻¹ | 提高末端速度请求，减少低速拖行；实际过冲需试车检查 |
| settle_time | 0.25 s | 进入容差后持续稳定判据 |
| pose_timeout / future_stamp_tolerance | 0.5 s / 0.1 s | 位姿源时间与接收新鲜度检查 |
| navigation_command_timeout / navigation_future_stamp_tolerance | 0.3 s / 0.1 s | 导航指令双重有效期与源时间超前量，独立于位姿门槛 |
| default_timeout / no_progress_timeout | 45 s / 5 s | 单动作超时、无可测进展超时 |
| max_distance / max_rotation | 5 m / 2π rad | 单次相对目标上限 |

XY 来自原地左右旋转录包拟合；Z 采用已测雷达底部距地面 0.201 m，加此前使用的 IMU 相对固定面 0.00288 m，得到 IMU 高度 0.20388 m，对应底盘点 Z=-0.20388 m。该机械/坐标假设不是所有车辆通用值。第二台车必须重新确认参考点、轴向和安装高度。

`calibrate_rotation` 是 control 包内的独立离线命令，不在在线控制器启动时自动运行。可读 rosbag2 `/Odometry` 或导出 CSV，按左右转分别拟合固定旋转中心；只提供平移建议，不能从平面旋转可靠求出未知高度，也不自动修改 YAML。操作见 CALIBRATION。

## 6. 日常启动、恢复和停止

推荐小车桌面 `bash ~/workspace/wheeltec_motion_ws/scripts/start_robot_tabs.sh`；没有图形桌面时使用前台 `start_robot.sh`。两者共用配置与单实例锁，不可同时运行。脚本自行清除旧 overlay 子环境并 source 系统 Humble 与三个工作空间 local_setup（共享 vendor 只加载一次），不依赖 .bashrc alias。

启动顺序：收到底盘 PowerVoltage 新回包 → Livox 点云/IMU 新鲜 → FAST-LIO 连续至少 3 条新鲜、递增且 frame/姿态有效的 Odometry → 控制 Action 可用且只有一个控制速度发布者。默认不启动 RViz。运行中的控制器空闲时持续发零速度，停止其他速度发布者后再使用。

| 异常 | 恢复策略 |
| --- | --- |
| 通信 | 先退出 motion，再重启 base；保留 Livox/FAST-LIO 地图 |
| Livox | 退出 motion、FAST-LIO，重启 Livox→定位→motion |
| FAST-LIO | 退出 motion，重启定位，数据合格后重启 motion |
| motion | 仅重启 motion |

运行中上游不健康时先撤下 motion；持续约 3 s 异常后恢复相应组件。启动阶段 45 s 未就绪重试；重试间隔 3 s 指数增加，最多 30 s，健康 60 s 后重置。恢复总时间还取决于驱动、ROS 发现和定位，不承诺几秒内完成。

标签页版本 `--status` 查看、`--stop` 停止；任一受管标签 Ctrl+C 或关闭它会停止整套服务，不再重启。普通启动终端返回后标签页继续运行。原版前台 Ctrl+C 同样退出全部；后台原版需先核对 supervisor.lock 的 PID/命令后发 SIGINT。脚本不保证自己被 SIGKILL 后仍作为系统服务存活，也没有注册开机启动。

FAST-LIO 重启会重建地图/重置 camera_init 原点；不会续跑旧 Action 或导航速度。后续任务编排必须结束旧任务、重新确认定位和目标，再由上层明确启用导航并发新请求。

## 7. 已验证事实、限制和已知问题

- 2026-09-29 Co-NavGPT2 官方源码锁定为 `891ba09959278b666ef01a7f8b6c64f8fe8436cc`；23 个 Python 文件语法与实车 import 链检查通过，合成 RGB-D 点云、前沿/FMM、CvBridge 转换及 Orin CUDA/NMS 检查通过。YOLO-World、CLIP 与 MobileSAM 本地模型完成 GPU 合成输入推理。D455、真实视觉导航和云 API 本轮未验证。
- 上游实车入口含写死的 Go2 ZeroMQ 地址、相机安装外参与 camera_init_1/body_1，单车还无条件开启 GUI；没有运行这些入口。2026-10-01 已提供持续导航速度接口与最小适配器；尚需在独立代码中适配本车 TF、相机同步与上游控制出口，不能仅重命名话题就认定接入完成。
- 2026-10-01 导航扩展：61 项控制/标定/ROS 测试及 15 项启动管理测试通过；ROS 域 173/174 合成验证连续更新、互斥、旧包过滤、超时/定位异常/停止、限速加速度、节点重启不重放、适配器与原 Action。适配器启用被拒绝时不停止现有 Action；安装入口单独 smoke 验证禁用输出与状态/服务。实车未发送运动指令，详见 VALIDATION。
- 2026-09-28 迁移静止验收通过：新 FAST-LIO 来自 vendor，15 秒 150 条位姿，年龄均值 32.5 ms、最大 45.9 ms；Action/stop 就绪，空闲速度全零。迁移后运动测试待用户执行。
- 用户已成功执行 0.2 m 低速直行及 90°转向。直行示例结果剩余约 0.02097 m，符合当前 0.03 m 容差；不是毫米级实测精度证明。
- 控制与标定测试共 23 项（16 核心控制＋7 标定）；加上 6 项合成 ROS 集成测试，Humble 基线共 29 项。启动管理另有 15 项测试覆盖依赖恢复、同页重启、并发退出和进程清理；不能把本轮仅复查 23 项写成重跑了全部 29 项。各轮范围见 VALIDATION。
- 当前停稳速度由相邻位姿差估计，对定位噪声敏感：假设两帧间隔 0.1 s、位置变化 3 mm，即算成 0.03 m/s，超过 stopped_linear_speed=0.025 m/s，可能重置停稳等待。这是判据敏感性的计算示例，不是本车已测噪声幅度；算法尚未改进，缩短 settle_time 不能彻底解决。
- 实车已验证四组件退出恢复、Livox 重启保持原标签、主动中断退出全链路，未由开发端发送实车移动/转向目标。
- 正常 Odometry 时间年龄常约 30–40 ms；偶发启动后约 1.3–1.4 s，单独重启 FAST-LIO 有时恢复。目前只认定“症状可通过重启恢复”，不认定已修复根因。
- `ros2 topic delay` 是测量订阅回调时刻减 header.stamp；FAST-LIO stamp 是点云扫描结束时间，包含输入、配对/处理、发布及测量端调度，不能直接当作发布到控制器的网络延迟，不能单凭此值证明队列积压。
- 相同现象参考：Taeyoung96/FAST_LIO_ROS2 #13（偶发 1.3 s、重启约 40 ms）；Livox 驱动 #229（CustomMsg 与 IMU 时间差）、#254（组帧时间间隔异常）。截至本阶段未确认与本车同一根因。用户已选择等待 issue 回复，暂缓排查。
- 未完成：长期运行、不同距离/地面/负载重复精度、实际停车时间、全项目 TF 验收、真实 RGB-D 世界地图验收、本地目标检测及 FMM 的实车效果、探索控制出口现场接通、双车通信与协作。相应软件接口和只读预览已有实现，不能与上述现场结果混为一谈。

## 8. 后续工作顺序与更新规则

当前优先推进 Co-NavGPT2 总体部署与接入，不继续调运动参数。第一档响应试调及其未验收状态保留；底盘固件未改。

1. 维护 conavgpt2_ws 的锁定源码、独立实车环境和离线验证；本阶段软件准备已完成。原版仿真环境和 HM3D 数据集未部署。
2. 优先使用已连接 ASTRA S：核验已有内参文件归属及配准误差，测量相机光学坐标系外参，接入已完成的只读位姿配对接口；明确 TF 唯一发布者。D455 已连接且独立数据接口可用，用户决定奥比 baseline 跑通后再换 D455；分别确认安装外参，不能套用奥比外参。
3. 先按 EXPLORATION_PREVIEW 完成真实静止图像/投影/地图验收，再接准备好的 DecisionBridge 到 navigation/cmd_vel + set_enabled/stop/status（见 NAVIGATION），由用户现场小范围试车。只读随机/FMM 链已做隔离验证，当前不接 VLM；正常运动周期性产生新决策，未来 VLM 长等待前明确 stop，失败或定位重置终止旧任务并重新确认；有限距离/转角才使用 Action 并等 Result。
4. 按固定版本复刻第二台车并重新标定，随后验证多车命名空间、地图配准、VLM 分配与通信；不能把当前离线模型测试当作多车导航已完成。
5. 运动响应仍为待办：恢复验证时先保持相同目标和 --max-speed，记录误差、运动耗时及 SETTLING 等待；若仍迟滞再同步比较 ROS 指令与手柄响应，排除自动控制零速度和受管重启造成的控制源冲突。
6. FAST-LIO 已迁入 vendor_ws；后续 vendor 构建显式选择包，避免 Livox 官方 build.sh 清掉整个 install。旧空间回退步骤见 WORKSPACE_SETUP。

新任务开始读取本文；使用细节按需读 OPERATIONS，部署按需读 WORKSPACE_SETUP。旧记录归档在小车 `~/workspace/archive/motion-before-v0.3.0/`、开发机 `archive/motion-before-v0.3.0/`，不是默认阅读入口；Git 旧提交仍保留过程可追溯。

后续开发提交时更新本文的事实/边界与 CHANGELOG，验证摘要更新 VALIDATION，原始运行数据放忽略的 runtime_logs。不能把“编译成功”“一次动作成功”替代尚未完成的量化验收。
