# 当前验证摘要

## 2026-10-10：雷达恢复后三段低速实车动作

用户确认连接恢复并授权继续，网口 enP8p1s0 恢复 UP / carrier=1 / 192.168.1.5。第一次 FAST-LIO 源年龄约 1.3 s，受管重启定位及 Livox 后恢复；运动控制就绪并取得新鲜位姿后，分段执行如下动作，未启动相机、探索或 VLM。

| 指令 | Action 完成时的定位估计进度 | 剩余误差 | Action 用时 | 实际采样速度峰值 |
| --- | --- | --- | --- | --- |
| 前进 0.10 m | 0.08387 m | 少 0.01613 m | 3.734 s | linear.x=0.05 m/s；航向修正 angular.z 峰值约 0.00542 rad/s |
| 左转 15° | 左转 13.792° | 少约 1.208° | 5.113 s | angular.z=0.10 rad/s |
| 右转 15° | 右转 16.208° | 多约 1.208° | 6.190 s | angular.z=-0.10 rad/s |

三个 Action 均返回 code=0 / target reached and settled，误差在现有直行 3 cm、转角约 2° 容差内。这是以 FAST-LIO 位姿反馈判定的控制结果，没有尺测真实位移/角度，不能当作独立精度标定。动作执行窗口位姿源年龄最大分别为 52.4 / 58.1 / 53.9 ms，速度发布者仅 wheeltec_motion_controller。

响应分析采用被动记录的 /cmd_vel 和换算后的底盘位姿：以首次非零指令前 1 s 位姿中位数为基准，前进超过 1 cm / 转角超过 1° 且连续 3 帧保持为“明显运动”判据。首次非零速度到该判据首次满足约 1.925 / 2.021 / 2.371 s；从首次非零输出到达到请求速度只约 0.150 / 0.095 / 0.101 s。该指标包含判定阈值、底盘响应、定位估计及链路影响，并非精确物理死延时，但不能仅用当前加速度斜坡解释。没有同步外部视频/轮速证据，不能直接归因于固件或定位算法，本轮不调参。

状态采样中 SETTLING 持续约 0.751 / 0.452 / 1.356 s；此期间输出零速度。最后显式调用 stop 成功，随后约 5 s 的 101 条速度采样全零、状态 IDLE。软件零输出及控制器停稳判据不等于独立测量物理刹停距离/时间。

本轮完成基础相对动作的初步实车验证；未验证连续导航、11.57 cm 相机外参、运动中的 RGB-D/位姿配对、地图一致性或 Co-NavGPT2 探索闭环。后续优先恢复只读相机探索时间配对，再验证移动时地图和目标方向的一致性，不以此次 Action 成功宣称 baseline 跑通。

证据：runtime_logs/acceptance/20261010-motion-restored/ 中的三段 .json/.log、events.jsonl、summary.json、monitor.json；受管日志 runtime_logs/robot_stack/20261010-135154-8079/。完成后主动停止并关闭本轮受管栈及被动记录器。

## 2026-10-10：低速运动验收启动受阻

用户明确要求修改后开始运动测试。计划分段前进 0.10 m（0.05 m/s）、左右各 15°（0.10 rad/s）。启动时底盘通信 READY，但 Livox 报 bind failed / Failed to init livox lidar sdk；主机 enP8p1s0 为 DOWN、carrier=0，未配置雷达使用的主机地址 192.168.1.5。被动记录无 /Odometry、无 /cmd_vel 发布者，未启动到运动控制阶段，没有发送运动目标。已请求用户恢复雷达供电/网线连接，本轮栈与记录器随后主动关闭；恢复后重新执行验收，不把启动尝试记为运动通过。证据：runtime_logs/robot_stack/20261010-135030-7485/ 和 runtime_logs/acceptance/20261010-motion/。

## 2026-10-10：地面参考相机高度配置更新

按用户决定，奥比预览使用地面有效旋转中心，Z 从上一轮诊断的 0.115 m 更新为 0.1157 m；来源是假定厂家参考点关系后的 0.0897 + 0.026。保留 confirmed=false，本轮未核实厂家变换链，也未开展新的实物测量。

- 配置加载回归确认读取 Z=0.1157 m、底盘到 IMU 的变换未变，D455 空安装外参仍拒绝。
- `bash scripts/test_exploration.sh src/wheeltec_exploration/test/test_core.py` 通过：包括探索核心和隔离 ROS 合成输入回归；脚本同时收集测试目录和指定文件，核心用例重复收集，不以运行计数代表额外覆盖。
- `bash scripts/start_exploration_preview.sh --check` 通过；安装配置符号链接最终指向修改的源码 YAML，无须额外复制。`git diff --check` 通过。
- 本轮没有启动硬件或发送实车运动请求。上述检查证明软件采用新数值及相关行为未回归，不证明外参物理准确或完整导航已验收。

## 2026-10-09：固定板复查与实测高度修正

用户补充：上一轮板子为手持，本轮已稳定靠在架子上；尺测镜头离地约 11.5 cm、板距约 57 cm。手持运动和倾斜会影响非同一曝光时刻的边缘/测点比较，因此不能把上一轮单帧疑点直接归因为固定外参错误。

- 相机单独运行约 45 s：997 组 RGB-D，22.47 Hz，有效深度中位约 53.84%，最大接收间隔 0.222 s，源年龄最大 0.0986 s，未出现 STALE。该负载没有定位/探索和速度发布者，不能替代完整链路连续性复验。采样报告已完整写出，采集命令包装进程返回 143；本次只据落盘帧/时间记录作数据结论，不将其算作正常退出 smoke 通过。
- 中央 60×60 像素区域逐帧深度中位数为 0.610–0.611 m；同一板面上部约 0.67 m、下沿约 0.565 m。用户的 0.57 m 参照应对应具体测点；板面倾斜、边框/板面位置不同，不能直接以中央差 4 cm 认定传感器测距误差。
- 5 个间隔约 10 s 的固定板快照：中央带板体→阴影的彩色边缘约第 395 行，深度边缘约第 393 行，差约 2 px。第 402–403 行的更强亮度梯度对应阴影→地面，不应误当板体边缘。此前约 20 px 的大偏移未复现；这只验证该局部边缘，不是整幅图或多距离配准验收。
- 预览配置 Z 从厂家 0.0897 m 改为用户尺测约 0.115 m，source 记录测量来源/日期，confirmed 仍为 false；保留原 XY、光学轴、第一台车运动控制配置。按相同水平安装假设，下沿相对参考地面的高度从约 -0.0508 m 改为 -0.0255 m，剩余差异需核验倾角、参考点、内参及板子实际接地点。
- 配置与核心回归通过（25 项）及启动入口 `--check` 通过；此处仅验证配置可用和现有逻辑未回归，不代表新的安装矩阵已物理标定完成。

证据目录：`runtime_logs/acceptance/20261009-fixed-board/`，包括 report/samples、5 组 RGB-D 快照、physical_edge_comparison.json、edge_height_comparison.json、core-tests.xml。完整负载复验记录在该目录的 `full-stack/`，本节后续结论按实际记录补充。

## 2026-10-09：真实静止验收未通过，移动验收未执行

用户在实验室明确授权代理开始验证。计划为静止合格后依次前进 0.2 m（最高 0.05 m/s）、左右各转 15°（最高 0.1 rad/s）。由于静止阶段出现下列问题，没有下发任何 Action 运动目标，也没有启用导航。此轮没有运行 D455 或 VLM，未改外参、速度、定位门槛或超时。

| 实际操作与输入 | 观测结果 | 结论与局限 |
| --- | --- | --- |
| 用户给雷达通电后启动已有机器人栈 | 网口恢复 192.168.1.5；首次 FAST-LIO 位姿源年龄约 1.396 s，受管超时重启后就绪 | 已知启动延迟现象仍存在，恢复成功不表示根因已修复 |
| 恢复后被动记录约 579 s | 5792 条位姿，源年龄中位 0.03446 s、95 分位 0.04982 s、最大 0.09056 s；坐标 X/Y/Z 跨度约 2.39/1.44/3.96 cm，XY 首尾相差约 1.56 cm | 时间新鲜；坐标变化需要结合实物是否完全静止判断，不能直接当成量化定位精度 |
| 同期订阅真实 `/cmd_vel` 和控制状态 | 11565 条速度均为零；唯一发布者为 wheeltec_motion_controller，控制器空闲且 pose_ready | 软件输出为零，不是物理制动试验；本轮运动目标数为 0 |
| 原实验室场景：低位视角、地面反光，10 s RGB-D 采样 | 130 组、约 13.24 Hz，源年龄最大 0.0869 s，有效深度平均 2.27%；地图仅形成少量细线 | 真实取流接通，但当前场景不足以验收地图覆盖或探索能力 |
| 用户在前方放入纸箱，6 s 再采样 | 77 组、约 12.80 Hz，有效深度平均 54.39%，新帧结束 READY；箱面深度主要约 0.6 m | 相机对该目标可产生连续深度；实测箱面距离尚未获得，不能据此宣称测距精度通过 |
| 比较同一 RGB-D 快照的箱体边缘及暂定外参投影 | 中央带彩色下边缘候选约第 391–396 行，0.5–0.8 m 深度目标末行约 416；按暂定底盘矩阵投影，选定目标点约 17.6% 位于参考地面以下，最低约 -8.6 cm | 单帧诊断疑点，需实测相机高度、核验标定/配准与参考原点，并排除目标放置时运动/边缘对应差异；未擅自修改外参 |
| 只读探索真实运行，并在输入恢复后显式 reset 一次 | epoch 0 和 epoch 1 均出现 STALE，清地图并持续零建议；累计处理数分别停在 318、416。记录到的规划处理最大约 0.101 s | 当前端到端连续性未通过；保护确实触发，不能靠反复 reset 或放宽超时宣称通过 |
| 额外 30 s 同时被动订阅原始图像和原子 RGB-D | 原始彩色最大接收间隔 0.511 s、深度 0.371 s、RGB-D 0.346 s；该轮 RGB-D 源年龄最大 0.0703 s | 观测到间歇，尚未分清传感器/驱动、BEST_EFFORT 传输缺帧或消费者调度；单个订阅者间隙不等于硬件一定停流 |

停止接口返回 `Stop requested; this response is not a physical stop measurement.`。随后关闭本轮启动的预览、奥比驱动与受管机器人栈。原始数据位于 `runtime_logs/acceptance/20261009-live/`：`acceptance_summary.json`、`events.jsonl`、`monitor.json`、`static-camera/`、`target-camera/`、`raw_depth_report.json`、`frame_timing_report.json` 和各启动/停止日志。受管日志位于 `runtime_logs/robot_stack/20261009-220645-25638/`。原始数据不入 Git。

关闭检查：受管 motion → FAST-LIO → Livox → base 均退出 0，相机与预览入口退出 0。预览后台 executor 在 Ctrl+C 时打印了 ExternalShutdownException，进程仍正常退出；该线程关闭日志需要后续整理，不将其当成运动或相机通过的证据。上表监测数字来自停止前的约 579 s 汇总截点，原始事件文件还包含后续停止阶段。

下一步：测量镜头中心离地高度和箱面到镜头距离，核验 RGB/深度边缘配准；记录更长的驱动/适配器/消费者时间链，定位间歇来源。完成这些检查并通过静止复验后，再执行已规划的小范围移动一致性检查。当前不把外形合理、数据 READY 或 FMM 输出 PLANNED 当作几何验收通过。

## 2026-10-02：只读随机探索软件链

本轮边界：用户不在车边，禁止车辆运动；D455 随意放置，不用于调试。只使用合成图像/深度/位姿及隔离 ROS 域，没有启动相机、雷达、厂家串口，没有向实车 `/cmd_vel` 或导航输入发布运动请求，也没有 VLM 调用。保留实车 motion YAML、厂家和第三方定位源码。

### 实际验证结果与含义

| 输入与操作 | 实际检查结果 | 证明什么 / 不证明什么 |
| --- | --- | --- |
| 已知深度、内参、光学轴和刚体变换 | 反投影方向、米制尺度、世界位置和 XY 栅格一致 | 数学契约；不能证明厂家外参或现有内参文件物理正确 |
| 人造遮挡墙、天花板、未知区、断开的空闲区 | 射线保留顺序，墙后不清为空闲；天花板不证明地面可通行；未知和断开区域不供规划通行 | 这些地图规则；不能证明动态避障、坑洞检测或真实深度质量 |
| 合成房间 300 帧，64×48 RGB-D，数值更新虚拟位姿 | 全部产生 PLANNED；132 次前进建议、168 次左转建议；6565 空闲格、120 障碍格；132/168 是虚拟决策，不是车体动作。处理均值 0.02368 s、最大 0.05806 s | 建图→随机前沿→官方 FMM→方向链可以运行并观察模拟墙；不证明高分辨率/模型实时性、探索效率或安全运动 |
| 相同种子重复运行；固定目标配相反车头方向；长期无进展 | 相同输入复现，左右转方向符合约定，目标保持及暂时排除按规则执行 | 有限场景下的决策行为；不证明实际转角或覆盖率 |
| 旧观测、地图边界、位姿跳变/断流；显式 reset | 旧帧不重复建图；严重故障清地图并锁存；新 epoch 需新数据 | 不自动延续旧地图/任务；不证明所有现场定位异常都已覆盖 |
| 在隔离 ROS 输入处理中故意增加 0.32 s 延迟 | `/decision` 实际收到 PROCESSING_EXPIRED，linear_x/angular_z 均为零 | 过期结果被丢弃；这里是预览字段，不是底盘制动 |
| 真实 MotionNode + NavigationClient + DecisionBridge，合成 Odometry，独立测试话题；另用模拟客户端注入桥接故障 | `/exploration_test/synthetic_output` 实际收到正向速度及 stop 后零速度，输出在本车上限内；桥接单元测试验证旧帧/epoch/客户端故障会解除 arm，速度与预览不一致会拒绝 | 控制接口与软件输出，以及所注入的桥接异常；没有串口/轮子，不证明物理响应或停车时间 |
| 无输入安装入口启动 1 s 并退出 | 默认 Astra；WAITING/零建议；退出记录 EXITED、processed=0，没有启动硬件 | 安装/入口/生命周期；不是相机取流验收 |

早期 80 帧 demo 未观察到障碍，因此当时只能证明基本数据链接通；扩展到 300 帧后观察到模拟墙。虚拟位移约 0.906 m 仅由数值运动学计算，不能写为实车移动。当前随机策略的覆盖率、目标变动频率及绕障效果没有量化验收。

可选本地模型分支的掩码→世界点、语义接近目标/无进展/重置逻辑已用合成几何验证；本轮没有运行新分支的完整 YOLO/MobileSAM 推理或真实识别，不将此前模型部署检查替代本分支验收。

### 命令、覆盖记录与证据位置

- `bash scripts/build_and_test.sh`：5 个自研包构建安装通过；控制 61、相机 49、探索 35，汇总 145，0 errors/failures/skipped；启动管理 15 及安装入口 smoke 通过。数字用于确认未漏跑，判断依据为上表行为和测试源码中的实际断言。
- 原控制回归涵盖同速持续更新、运动中变更、最新请求优先、源/接收超时、旧会话过滤、Action 互斥、主动/失效停止、加速度与速度上限，以及原相对 Action 到达/停稳/结果语义。真实控制节点和消息均仅使用合成位姿与测试输出。
- `bash scripts/start_exploration_preview.sh --check`：配置和锁定官方 FMM 检查通过；其 5 cm 栅格和 25 cm 到达单位在首版固定，配置改动会明确拒绝。
- `bash scripts/start_exploration_preview.sh --demo --steps 300 --output runtime_logs/exploration/demo300`：上述模拟报告、地图与轨迹已生成。
- `ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 bash scripts/start_exploration_preview.sh --start --duration 1 --output runtime_logs/exploration/no-input-smoke`：无输入入口正常退出。沙箱限制网络接口/UDP，ROS 提示 Operation not permitted；同进程隔离 ROS 测试通过，不能据此推断实车跨进程/网络 DDS 性能。
- 完整回归：`runtime_logs/validation/20261002-174042-7/`；总输出 `runtime_logs/exploration-full-validation.log`。新测试 XML：`build/wheeltec_exploration/pytest.xml`。模拟与无输入记录：`runtime_logs/exploration/demo300/`、`runtime_logs/exploration/no-input-smoke/`。这些原始输出忽略 Git。

下一步现场证据应来自 Astra 标定/配准检查、静止世界地图和用户运动验收；操作顺序、通过/失败现象及剩余限制见 [EXPLORATION_PREVIEW](EXPLORATION_PREVIEW.md)。本轮结论为只读探索软件链通过已列验证，完整 Co-NavGPT2 实机自主探索未验收。

## 2026-10-01：自研 Astra RGB-D 接口

- `bash scripts/build_and_test.sh`：4 个自研包编译安装通过；原控制 61 项、启动管理 15 项、安装后 Action/导航禁用入口与 CLI/关闭 smoke 通过。相机后续扩展用独立域 177/178 复验，最终 `python3 -m pytest src/wheeltec_rgbd/test -q --junitxml=build/wheeltec_rgbd/pytest.xml` 为 26 passed；`colcon test-result --verbose` 为 87 tests，0 errors/failures/skipped。
- 覆盖 RGB/BGR、16UC1/32FC1、毫米/米、大小端与 padding、范围/NaN、编码/尺寸/内参错误、精确图像内参匹配、时间过期/超前、重复/乱序、有限缓存、去畸变与投影、显式 YAML、断流恢复、ROS 时钟冻结后的单调超时、处理停顿不能发布过期帧、完整 VGA bundle 可在有效期内消费。合成 ROS 测试不启动驱动或底盘。
- `bash scripts/start_astra_rgbd.sh --check` 参数解析通过，默认加载已有本机 RGB/IR 文件；Bash 语法和 Git diff 空白检查通过。第一台运动配置及厂家/第三方源码未修改。
- 独立 ROS 域 176、localhost，真实相机最终 8 秒取流：消费 128 组、约 20.98 Hz，RGB/深度 640×480；配对时间差最大 0.02969 s，源时间年龄最大 0.17064 s，结束 READY。深度米制/NaN，当前场景范围内有效覆盖平均约 8.44%；仅为短时场景结果，不是测距精度或长期 30 Hz 验收。域内 /cmd_vel 发布者为 0，结束后关闭本次驱动与适配器。
- 设备 SDK 两种参数查询格式、取流前后均返回 NaN；原驱动深度 CameraInfo 的 K 含 NaN、彩色回退为 FOV 估算。原驱动还会在无效 SDK 参数时覆盖文件深度 K。自研层独立加载指定 YAML，保留真实 frame/stamp 与尺寸约束；未伪造设备内参或改厂家。现有两文件 K/D/R/P 相同、名称不同且未绑定序列号，输出明确标记 local_yaml_device_binding_unverified，物理归属/配准精度待验收。
- 发现图像 bytes 赋值触发逐字节校验：本机 1.23 MB 赋值约 0.214 s，而 array('B') 约 0.000163 s；改为字节缓冲后通过真实取流复验，未延长超时。Cyclone 对比未解决旧实现瓶颈，最终使用原环境默认中间件，没有改全局 ROS 设置。
- 最终快照离线调用 Co-NavGPT2 原版 build_full_scene_pcd：28626 个有效像素，经上游 0.05 m voxel/DBSCAN 后 339 点，点与颜色有限。仅相机光学坐标系，不包含世界变换或运动。首次检查曾错误要求滤波后点数等于输入像素数，按源码的降采样/聚类契约修正后通过。
- 构建记录：runtime_logs/validation/20261001-193953-9/；最终相机记录/NPZ/点云：runtime_logs/camera/20261001-rgbd-interface-verified/。标定原文件 SHA256：rgb=f5f6f46da364fce1902a393d4e73659fa20b5798ef85276012cc3cef1dddcede，ir=31e84754340d4843be61a6f4bd60affe45f58898db9fa2501f20b71a20b052cb；未复制这些文件到 Git。

## 2026-10-01：厂家功能包只读追踪

- 从桌面常用指令核对厂家 src/install 包索引、launch/run 入口、相机与车型 Nav2 配置；除 qt_ros_test 外，指令中的去重入口均找到安装入口，包含嵌套 launch。
- 核对 Astra 源码、整车/EKF/TF、RTAB 同步与深度转扫描、Nav2 输出、RRT 目标转发、HSV/PID 跟随；结论见 FACTORY_CAPABILITIES。
- 仅为静态源码/配置审计，不是 ROS 运行、算法性能或实车验收；未启动节点、打开相机、发送速度或修改厂家/第三方工程。文档经 Git diff 空白检查。

## 2026-10-01：当前相机识别与短时读流

- USB/sysfs 确认 ORBBEC / ASTRA S、2bc5:0402、480 Mb/s；RealSense 枚举无设备。使用现有 OpenNI SDK 2.3.0.85 查询固件 5.8.22。
- 每模式约 5 秒：VGA 彩色/深度约 29.6 / 29.1 fps；HD 彩色 1280×720 与 VGA 深度双流可读；配准开启成功但未测对齐误差。高分辨率深度 1280×1024@7 在单流可读约 7.3 fps，与 30 fps 彩色同步的双流组合未收到深度。
- 短时采样发现远超 ASTRA S 标称范围的非零值、深度有效覆盖有限及个别非递增 SDK 时间戳，保留为后续范围掩码、时钟与安装场景检查项，不声称已验收测距精度或长期稳定性。
- 未初始化 ROS 运动节点、发布底盘速度、修改厂家源码/固件或持久相机设置。完整比较见 CAMERA_ASSESSMENT；原始程序/输出位于 runtime_logs/camera/20261001-astra-s/，忽略 Git。

## 2026-10-01：持续速度导航接口

环境为小车 Ubuntu 22.04/aarch64、ROS 2 Humble、Python 3.10.12。在实际 motion 仓库执行 `bash scripts/build_and_test.sh`，只构建本仓库两个包，未修改/重建厂家底盘、STM32 或第三方定位代码。第一台车与 example 的所有既有 YAML 值逐项与 HEAD 比对无变化，只新增六项导航设置；构建后 install 配置与 src 相同，本车速度 cap=0.15 m/s、0.35 rad/s，保留已用外参与加速度 0.30 / 1.0。

默认 ROS 域只读执行 `ros2 node list --no-daemon` 返回空列表，本轮没有启动实车链路或读回运行中的实车参数。限速核对依据实际管理脚本读取的源码 YAML 和构建后的安装 YAML；随后正式启动仍须查询运行值。

- 最终构建通过；**61 passed，0 errors / failures / skipped**：16 项原核心控制、7 项标定、16 项导航核心、16 项导航 ROS/适配器、6 项原 ROS Action 集成。原相对运动的正反向、左右转、停稳成功、忙碌拒绝、取消、停止、位姿失效、超时/无进展和参数门槛均回归通过。
- 导航核心覆盖：重复同速新请求持续运动、运动中改变/反向、最新请求优先、旧/重复及旧会话包不覆盖且不续期、接收与源年龄独立超时、源时钟跳变、待机/零请求、立即停机、无自动重放、每个不支持分量及 NaN/±Inf、错误 frame、正负越界拒绝、限速与逐轴加速度限制、定位故障跨恢复样本保留。
- **独立域 173、ROS_LOCALHOST_ONLY=1**：合成位姿仅 `/motion_test/odom`，速度只 `/motion_test/velocity`。通过真实 DDS 消息、Action、SetBool/Trigger 与 MotionStatus 检查同速更新之间无零输出，稀疏新请求之间仍由 20 Hz timer 输出、请求变化、唯一速度发布者、时间戳乱序/源过期、非法/越界请求、定位超时与跳变、stop/退出、控制定时器缺口、参数锁定、Action 互斥和结果语义。
- 节点完整退出/重建后，旧包与未启用时的新包均不能产生运动；重新启用仍须新的有效请求。Python 适配器检查新决策及状态/会话、拒绝重复/过期决策，服务停车和显式恢复通过；没有循环重发功能。补充启用被 Action 占用拒绝时不会顺带停止现有 Action 的回归检查。
- 启动管理 **15 passed**；独立域 **174** 使用禁用输出 example 检查安装 launch、原 motion_client、navigation_client 的 status/enable/disable/stop、禁用输出时动作/导航拒绝，以及 Ctrl+C 正常退出，全部通过。
- 初轮沙箱内 ROS 通过共享内存完成测试，但 DDS 网络接口报 Operation not permitted；入口退出另发现重复 SIGINT 中断资源清理。已修复清理阶段信号处理，最终在沙箱外复验完整流程通过，不把沙箱网络错误当作实车 DDS 性能问题。

最终原始输出：`runtime_logs/validation/20261001-173744-18172/`（build.log、test.log、test-result.log、stack-tests.log、launch/client/navigation-client smoke）及 `runtime_logs/validation/navigation-delivery-run.log`，均忽略 Git。此前通过的 60 项轮次位于 20261001-173326-16376，本轮补充适配器拒绝用例至 61 项。

本次没有启动厂家串口/雷达/相机进行验收，没有发送实车运动目标或底盘速度。验证结论为**软件接口与输出契约通过**，不证明物理停车时间、实车惯性/换向响应、真实定位连续性或 VLM/感知实时性；完整 Co-NavGPT2 自主探索仍待 D455、坐标变换、上游出口接入及用户试车。

## 2026-09-29：Co-NavGPT2 无相机软件部署

- 官方代码位于 `~/workspace/conavgpt2_ws/upstream/Co-NavGPT2`，提交 `891ba09959278b666ef01a7f8b6c64f8fe8436cc`；CLIP 来源、依赖版本和权重 SHA256 分别保存在 conavgpt2_ws 的 deployment.lock.json / models.lock.json。上游算法文件未修改。
- 独立 .venv 继承本机 Jetson torch/torchvision、Open3D 和 ROS 库，安装实车 import 链所需依赖；未改 motion 参数、底盘、Livox/FAST-LIO、系统 CUDA 或仿真环境。
- 在 conavgpt2_ws 加载 `source scripts/env.sh` 后执行 `python scripts/check_deployment.py`：CPU 离线检查通过。随后在可访问本机 NVIDIA 设备的环境执行 `python scripts/check_deployment.py --models --device cuda`：全部通过。
- 范围：23 个上游 Python 文件语法、实车模块导入、合成点云/前沿/FMM、rgb8/16UC1 图像桥接、Orin CUDA 张量及 NMS、YOLO-World/CLIP 类别编码与图像推理、MobileSAM 框提示分割。原始报告在 conavgpt2_ws/runtime_logs/deployment-cpu.json 和 deployment-cuda-models.json；不代表实车识别精度或实时性能验收。
- 现有 RealSense 驱动 rs_launch.py 参数解析通过；未找到 realsense2_description 包，相机单独启动准备不依赖它。本车未连接 D455，未验证真实图像流、相机外参/TF、WHEELTEC 控制适配、完整导航或云 API。
- 检查未初始化 ROS 节点、发布速度或发送实车动作，未运行上游会发送 Go2 ZeroMQ 速度的导航入口。torchvision 可选 nvjpeg 扩展缺失提示仍存在，但 OpenCV/CvBridge 与本轮 GPU 推理通过。

## 2026-09-29：项目复查与响应问题说明

- 加载系统 Humble 后执行 `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PWD/src/wheeltec_motion_control:$PYTHONPATH" python3 -m pytest -p no:cacheprovider src/wheeltec_motion_control/test/test_core.py src/wheeltec_motion_control/test/test_calibration.py -q`：23 passed（16 核心控制＋7 标定）。初次未加载 Humble 时 rosbag 用例无法导入 rosbag2_py；加载环境后全部通过，无需安装依赖。
- 执行 `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s scripts/robot_stack -p 'test_*.py' -v`：15 passed。
- 两个启动入口 `--check` 均通过，仅检查环境与配置。本轮未重跑 6 项 ROS 集成测试、未启动硬件或发送实车动作。
- 文档补充用户反馈的手柄迟滞观察、停稳判据噪声敏感性与同条件验证顺序；0.25 s 迟滞和 0.4 s 切换为尚未同步测量的体感描述，3 mm 抖动为计算示例，均不作为量化实车验收结果。

## 2026-09-28：第一档响应参数试调

- 仅 motion_control.yaml 五项变化：linear_accel=0.30、angular_accel=1.0、distance_gain=1.2、rotation_gain=1.6、settle_time=0.25；实车旧配置备份在 runtime_logs/tuning/20260928-responsive-1/motion_control.before.yaml，Git 保守基线 ff29eb7。
- 参数合法性与差异集合检查通过；复用现有纯软件模拟器对当前 YAML 做前后 0.2 m / 1 m、左右 90°六组检查，均完成且输出不超过原限速。该理想模型不包含实际底盘延迟、摩擦和惯性，不能替代实车验收。
- 小车无既有运行节点，应用配置后启动四标签链路；启动检查通过，12 项实际运行参数读回与 YAML 一致。
- 10 秒只读检查收到 103 条 Odometry，时间年龄均值 0.05096 s、最大 0.35795 s，均在当前 0.5 s 门槛内；201 条速度消息全零且只有控制器一个发布者。未发送实车运动目标，运动响应改善待用户测试。
- 本轮输出在 runtime_logs/tuning/20260928-responsive-1/；启动日志在 runtime_logs/robot_stack/20260928-224940-tabs-16698/。无需重新编译，控制算法、频率、限速、到达容差、静止速度阈值及底盘固件均未改。


## 2026-09-28：FAST-LIO 迁入 vendor_ws

- 新位置仅编译 fast_lio，Release 构建通过（约 2 分 40 秒）；未重建 Livox/SDK。编译有上游 GCC 参数传递 ABI 提示，无构建错误。
- FAST-LIO 完整复制后核对 130 个文件内容一致；新安装 mid360.yaml 与旧安装参数一致。Livox 安装 JSON 逐字节未变，仍使用 .145。
- 开发机和小车启动管理测试各 15 项通过，包含共享 vendor 去重加载以及旧分离工作空间兼容；两个启动入口 --check 均通过。
- 在新的 base/vendor/motion 环境中运行安装入口 smoke：禁用输出下拒绝动作、stop CLI、正常关闭均通过。控制算法未修改，29 项核心/标定/ROS 测试的基线结果见下节，本次未重复。
- 新交互终端 ros2 pkg prefix fast_lio 解析到 ~/workspace/vendor_ws/install/fast_lio；lclocal 保留原命令名，.bashrc 不再加载 conavGPT_ws。实际运行进程也来自新路径。
- 本轮开始时雷达网口 NO-CARRIER，无 192.168.1.5 地址，Livox bind failed；用户接线通电后恢复。总控按现有策略重试并达到四组件就绪。
- 15 秒只读静止验收：150 条 Odometry（约 10 Hz），时间年龄均值 0.03254 s、最小 0.01903 s、最大 0.04594 s；camera_init/body 正确且时间戳递增。283 条 cmd_vel 全为零，唯一发布者为 wheeltec_motion_controller；Odometry 唯一发布者为 laser_mapping，Action 和 stop 服务均可用。
- 旧 conavGPT_ws 已整体归档到 ~/workspace/archive/fastlio-migration-20260928/conavGPT_ws，归档后路径检查通过。四标签链路保持运行供用户测试；没有发送实车运动目标，运动验收待用户执行。

构建与迁移快照：~/workspace/archive/fastlio-migration-20260928/。入口 smoke 和只读结果：runtime_logs/validation/fastlio-migration-20260928/。本轮四标签日志：runtime_logs/robot_stack/20260928-222437-tabs-9260/。以下 v0.3.0 数据是控制基线验收，不代表本次重复执行。

## v0.3.0 基线验证（2026-09-26）

日期：2026-09-26；环境：小车原生 Ubuntu 22.04/aarch64、ROS 2 Humble、Python 3.10.12。验证对象为本发布提交的源码；通过 v0.3.0 标签定位版本。

## 本次发布复验

在小车 `~/workspace/wheeltec_motion_ws` 执行：

```bash
bash scripts/build_and_test.sh
bash scripts/start_robot.sh --check
bash scripts/start_robot_tabs.sh --check
```

| 检查 | 结果与范围 |
| --- | --- |
| colcon build | 2 packages finished，约 9.3 s；control/interfaces 均 0.3.0 |
| colcon test / test-result | 29 passed，0 errors / failures / skipped；16 核心控制、7 离线标定、6 合成 ROS 集成测试 |
| 启动管理 unittest | 14 passed；依赖顺序、延迟触发恢复、同页重启、Ctrl+C/管理进程退出与子进程清理 |
| 安装后入口 smoke | 通过；使用禁用输出 example，Action 拒绝请求、stop CLI 成功、Ctrl+C 正常退出 |
| 原前台入口 --check | CHECK PASSED；四空间包路径和实车配置解析通过，未启动硬件 |
| 四标签入口 --check | CHECK PASSED；配置解析通过，未创建新标签/启动硬件 |
| 开发机静态检查 | Bash 语法、Python 编译、Git diff 空白检查通过；14 启动管理测试也通过 |
| 配置与代码保留 | 实车 YAML 的参数值与发布前完全一致；控制算法、ROS 节点、CLI、Action 和标定算法未改 |
| 厂家/第三方保留 | base 源码与厂家来源 45 文件一致；base、FAST-LIO、vendor 仓库状态与审计基线一致；驱动 src=.45 / install=.145 的差异已记录，没有更改 |

测试采用 localhost 上的独立 ROS 域：colcon 合成测试 173，安装入口检查 174；既有集成测试也在隔离域使用假里程计。没有打开实车串口、启动雷达或发送实车运动目标。安装入口检查明确指定 example，避免第一台的启用配置参与 smoke。

本次原始输出：`~/workspace/wheeltec_motion_ws/runtime_logs/validation/20260926-230447-35519/`，包含 build.log、test.log、test-result.log、stack-tests.log、smoke.log、launch/client-smoke.log 和 ROS 日志。新一轮验证会生成独立目录，不覆盖它；原始日志不进入 Git。

## 已有实车事实与验收边界

用户此前完成低速 0.2 m 直行和 90°旋转，返回成功；直行残余约 0.02097 m 满足当前 0.03 m 容差。此前也验证过上游故障恢复、标签内重启及主动中断关闭全链路。本次整理保持这些核心实现，但没有重新进行实车运动或故障注入。

以上不能证明毫米级精度、各种地面/负载下的重复性、长期稳定定位、实际停车时间或完整自主导航。偶发 1.3–1.4 s 定位年龄仍是已知未定位根因问题；自动恢复是当前处理策略。下一步验收与边界见 STAGE_SUMMARY。

## 整理与追溯

清理前已备份小车的 README、CHANGELOG、docs、logs、analysis、实车配置和 Git 历史，以及第三方配置差异。位置为 `~/workspace/archive/motion-before-v0.3.0/`；开发机另有 `archive/motion-before-v0.3.0/` 副本。随后移除 motion 仓库里的旧逐次开发日志、原始日志、分析输出与重复脚本说明，保留 Git 历史；旧 runtime_logs 整目录移动到同一归档的 runtime_logs/，保留全部文件；本次 validation/ 移回当前工作空间。系统 ~/.ros/log 未处理。

本次开发事实集中于 STAGE_SUMMARY，操作集中于 OPERATIONS，部署集中于 WORKSPACE_SETUP。原始记录仅在用户明确要求历史排查时查阅，不再作为后续任务默认输入。

## 2026-10-01 RGB-D / FAST-LIO 只读配对

- `colcon build --packages-select wheeltec_rgbd --symlink-install` 成功，1 包安装完成。
- `ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=$PWD/runtime_logs/posed-validation python3 -m pytest src/wheeltec_rgbd/test -q --junitxml=build/wheeltec_rgbd/pytest.xml`：43 passed / 3.49 s，包含此前 26 项回归和新增 17 项。
- 覆盖光学轴/平移/旋转组合、非刚体拒绝、位姿有界缓存与匹配窗口、重复/倒序/跳变/断流/未来/过期锁存、单调超时、未确认外参拒绝、真实 ROS Odometry 订阅/帧名/重置，以及合成图像→适配 bundle→位姿配对→世界点的完整只读数据链。
- 本轮没有重跑 61 项运动或 15 项管理测试，运动代码及配置未改。未启动相机硬件、底盘或 FAST-LIO；只使用本机隔离 ROS 域的合成消息，没有 /cmd_vel 发布者。
- 首次测试因默认 ROS 日志目录在沙箱只读而失败，随后将 ROS_LOG_DIR 指向仓库忽略的 runtime_logs 后通过；未修改系统权限。构建输出在 runtime_logs/posed-validation-build.log。
- 真实相机外参、内参设备归属、物理配准、曝光时间同步和世界地图质量均未验收；接口成功不等于自主导航跑通。

## 2026-10-02 D455 工具准备与独立相机接口

- 只读 USB/SDK 检查：D455 8086:0b5c，5 Gbps；固件 5.15.1.55。现有 ros-humble-librealsense2 2.58.4 与 realsense2-camera 4.58.4 已包含 SDK 头文件/库、Viewer 和诊断工具，无需重复安装或替换内核驱动。`d455_tools.sh --check` 通过；Viewer 无缺失动态库，真实 X11 窗口 RealSense Viewer v2.58.4.0 创建成功，测试后关闭。桌面快捷方式已安装并标记可信。
- `colcon build --packages-select wheeltec_rgbd --symlink-install`：1 包成功。Bash 语法、启动 --show-args 与 git diff --check 通过。
- 隔离 ROS 域 181、ROS_LOCALHOST_ONLY=1，pytest 相机包：49 passed / 3.56 s（原 43 + 新 6）。包含 D455 深度范围/设备投影、未对齐深度拒绝、默认奥比配置、不同话题/光学帧，以及两种相机的选择端相互隔离。新增边界测试发现 float32 乘 0.001 将 6000 mm 算成略大于 6.0，修正为 /1000 后 inclusive 上界通过。
- 最终分离版静止验证：独立 ROS 域 180，仅 D455 驱动+自研适配器，10 秒含启动共消费 109 组，约 13.82 Hz；适配器统计 accepted=113、rejected=0，结束 READY；RGB/对齐深度消息同戳，最大源年龄 0.235 s，当前场景应用掩码有效深度平均约 71.1%。输出 frame=wheeltec_d455_color_optical_frame，设备 CameraInfo 来源，无 Astra YAML 覆盖。该域 /cmd_vel 发布者为 0；相机与适配进程均干净退出。原始数据在 runtime_logs/camera/20261002-d455-separated/。
- 初次共用输出原型曾消费 183 组 / 14.54 Hz，1 次拒绝原因未单独留存；随后增加拒绝原因统计，最终独立输出版本 0 拒绝。最终版本以分离话题和帧记录为准，不把原型验证代替最终结果。
- 最初真实快照在 Co-NavGPT2 官方 build_full_scene_pcd 函数中生成 15214 个降采样/过滤点，输入 232524 个有效深度像素，点均有限；仅光学坐标系，未附世界位姿。结果在 runtime_logs/camera/20261002-d455-interface/conav_geometry.json。
- 实际驱动请求并读取 640×480 RGB8/Z16 / 15 Hz，开启 enable_sync/align_depth，禁用 IMU/点云。保留启动时 power_line_frequency 默认值越界告警的事实，取流通过，未改设备固件或厂家/定位代码；本轮也未测曝光同步、深度物理精度或长时稳定性。
- 未重跑运动/启动管理测试（未改对应源码/配置）。主线默认奥比，D455 为后续切换测试；安装外参、世界建图和完整 baseline 仍待验收。
