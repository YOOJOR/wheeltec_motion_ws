# 当前验证摘要

## 2026-10-12：扩大范围验收未通过，图像迟到与接口缺陷定位

用户重新确认在车边，授权本轮有限低速测试。速度保持0.05m/s、0.25rad/s，范围为最长60s、距起点0.5m或累计转角10rad；检测/VLM关闭、D455未使用。没有放宽导航0.3s、定位0.5s或配对50ms门槛。

| 输入与检验 | 实际结果 | 结论 |
| --- | --- | --- |
| 原版Agent数值房间2000步、每步模拟0.1s | INITIAL_SCAN 252、PLANNED 1748，forward 607、left 952、right 441，模拟碰撞拦截0；前后位移1.625m；平均计算0.080s、最大0.153s | 当前目标更新节奏可持续推进模拟探索；不是实车避障/覆盖率证明，模拟使用离线默认12m/320×240输入 |
| 四向6m地图边界、跨π及反向旋转、重复帧、静止扫描超时 | 6项通过；边界触发零输出并清图锁存，恢复位姿不重放，显式reset建立新原点；反转扣减扫描角，静止新帧不算扫描完成 | 检查边界和恢复的实际输入→状态行为，无硬件 |
| 实车静止25s与45s（原消费者DDS配置） | 363/606组有效观测，无过期；最大图像年龄0.161/0.154s，计算0.065/0.061s | 静止输入有余量，不能推断运动期间无迟到 |
| 实车首次尝试 | 16条新决定后第17帧PROCESSING_EXPIRED，约转0.149rad；停止成功、最终IDLE/零，用户确认停稳无异常 | 未完成扫描；旧报告遗漏触发帧耗时，不能据此前16帧推定失败耗时；控制记录另有一次receive timeout |
| 第二次尝试 | 导航启用响应超时，最终stop成功，85/90s记录窗口内该轮非零输出0 | 未进入运动；服务可能已启用但响应未被取到，按不确定状态停止处理，不能宣称服务器明确拒绝 |
| 内置DDS消费者静止40s，发布端仍32MiB | 570组有效观测，无过期，最大图像年龄0.152s、计算0.063s | 消费者不必强制相机发布端配置；入口现尊重调用方DDS或默认配置 |
| 修复服务等待后的最后运动 | 95条新请求，只有左转；第96帧输入年龄0.263024s、Agent调用0.053741s、总处理0.058871s，处理后图像年龄0.316852s，触发保护；stop成功、IDLE/零 | 明确延迟主要在Agent之前；未完成一圈、更未进入持续前进，长程验收未通过 |

最后一轮记录中请求年龄最大0.141s、定位年龄最大0.081s，/cmd_vel唯一发布者为wheeltec_motion_controller，前进峰值0、角速度峰值0.25rad/s。定位前后角度变化约1.479rad，不能用含噪累计路径当物理里程。用户确认最终实际停稳、无其他异常；没有物理停车时间测量。随后主动关闭本轮基础栈和相机。

故障诊断修复：运动过期时先保存触发帧的input_age_sec、agent_sec、source_age_sec与processing_sec到最终报告，再由finally停止，停止先于文件归档。独立域192用0.35s旧合成图像验证保留原因且不输出非零，1项通过；不把此改动称为图像延迟已修复。

服务发现对照：默认DDS的3次停止探针成功；相机自定义配置3次未在2s内发现服务。内置DDS另有一次发现耗时2.004s，旧客户端仍发出请求但响应等待为0，复现确定的等待预算缺陷。现服务发现与响应各自最多timeout（默认各2s），请求只发一次，失败不自动重试；速度请求TTL未变。域193回归18项通过（2项RPC边界+16项已有导航ROS契约）。自定义配置的发现差异只是现场观测，尚不能据此认定所有DDS延迟根因。

还发现教程中的 `python -m wheeltec_motion_control.navigation_client stop` 缺少main执行入口，可能以0退出但没有调用服务。已补入口；新测试真正启动子进程，在合成控制器上验证退出导航及零输出，并验证不存在的服务返回非零。该项与2项RPC边界复验共3项通过。停车状态下真实命令返回 `Stop requested; this response is not a physical stop measurement.`；早期fixed-stop-probes.log为空，是发现此问题的证据，不算成功调用。

证据：本仓库 `runtime_logs/conav/acceptance/20261012-extended/` 的static、static-diagnostics、static-builtin-dds、motion、retry、final及对应recording/samples，stop-probes.jsonl、stop-builtin-probes.jsonl、navigation-client-tests.log、stop-cli-tests.log、manual-stop-check.log；源码部署仓库 `runtime_logs/source-agent/extended-20261012/` 的room-2000、boundaries.log、expiry-diagnostics.log。失败与成功记录均保留。

下一步应同步采集运动时原始彩色/深度、原子RGB-D的到达与源时间、位姿及配对队列年龄，定位图像迟到发生在哪一段。保持保护策略，在找到原因前不继续扩大实车范围；前一轮短程成功仍有效，但不足以宣称持续探索稳定。

## 2026-10-12：原版Agent适配与有限实车探索

本轮主开发改为 `conavgpt2_ws/development/Co-NavGPT2` 的源码分支，旧自研预览是测试对照。执行原版 ROS_Agent.mapping、前沿、FMM和动作方法，检测/VLM关闭；坐标、观测配对、测量一圈扫描及本车速度接口单独适配。用户在车边明确授权本轮低速、有限预算试车，并确认结束实际停稳。

| 验证输入 / 操作 | 实际结果 | 所证明的行为与局限 |
| --- | --- | --- |
| 原版源码AST对照、单位/投影/左右方向、不同裁边采样、圆盘膨胀逐格对照、短FMM路径展开 | 10项算法测试通过；原版地图/规划方法的边界改动有对照证据 | 原版方法实际复用、优化及坐标边界符合约定；采样密度降低不保证地图逐格等同全分辨率 |
| 独立ROS域190，合成RGB-D/位姿，实际控制节点仅输出/source_test/synthetic_output | 最终8项通过：扫描→PLANNED→非零→停止，断流→零、恢复不重放，0.12s图像延迟仍处理新决定，0.35s旧图像只停止，预算正常退出/地图归档，启用拒绝或服务缺失不stop别人的Action | 验证真实ROS输入/模式/输出与时序契约；未创建实车/cmd_vel发布者、不接厂家串口。不是实测物理停车 |
| 相机可选限频，合成原子帧与跨进程ROS传输，独立域191 | 59项相机测试通过，含限频丢弃多余帧、不改源时间、断流STALE、不重发旧帧；wheeltec_rgbd构建成功 | 限频发生在相机适配出口，不改厂家驱动/内参；默认0不限频，不降低失效门槛 |
| 真实传感器、只读、无RViz的25s预检 | 316组有效决定，无过期；最大图像源年龄约0.169s、计算约0.063s | 当前现场/负载下输入和处理有实时余量；不保证任意负载或长期稳定 |
| 最终真实运动：奥比、FAST-LIO、6m地图/步长4、0.05m/s前进与0.25rad/s转向，45s/0.2m/8rad预算 | 完成6.296rad启动扫描，386个INITIAL_SCAN后170个PLANNED，556条新鲜导航请求，约39.3s有效决定；预算结束stop成功 | 短程原版建图→选目标/FMM→运动输出链路通过；预算由定位估计，不能当作精确物理边界 |
| 被动订阅真实输出、定位和状态，操作者现场确认 | /cmd_vel唯一发布者wheeltec_motion_controller；输出峰值0.05/0.25；最终IDLE且输出零。定位前后位移约0.215m，用户确认一圈后转向/前进、最终停稳无异常 | 软件停止已验证，物理停稳有定性人工确认；未尺测位移或停车时间，累计路径受定位噪声影响不作物理里程 |

最终实车请求最大年龄0.0823s、图像年龄0.2119s、计算0.0679s、位姿年龄0.1215s；控制状态未出现导航超时。被动订阅收到548个决定，源程序逐帧记录556个有效决定，以各自记录范围为准，不以订阅漏帧推算源码调用数。

最初两次实车尝试均因导航请求源时间过期而退出并停车，没有通过。原因是将图像采集时间直接作为运动请求时间，传输/配对/计算消耗了请求期限。当前在**开始计算一组新观测**时取decision_stamp_ns，原图像时间保留source_stamp_ns；图像年龄、计算耗时各0.3s、控制器请求0.3s、定位0.5s、配对50ms及原限速没有放宽。不在计算完成时盖新戳、不重复发旧决定。先前大分辨率超时记录同样保留。

最终物理运行按预算停止，但当时入口把正常预算结束记为RuntimeError；收尾代码已改为正常结束，并补充最终状态/地图归档与控制权清理，最后8项ROS测试在这些改动后全通过。没有为日志整理再重复实车运动。本轮算法10项来自delivery-tests.log，其中5项ROS曾失败；最终ROS结果以final-ros-tests.log的8项通过为准，不将中间整体失败日志称作全套通过。数量仅帮助核对覆盖内容。当前目标更新节奏调整后没有重跑数值房间，不沿用早期模拟报告作当前版本结论。

复验命令（合成输入；源码目录内，先加载Humble、motion install与conavgpt2 scripts/env.sh；线程数均1，相机FastDDS配置同运行教程）：

```bash
ROS_DOMAIN_ID=190 ROS_LOCALHOST_ONLY=1 python -m pytest wheeltec/test_agent.py wheeltec/test_runtime.py -q
ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1 python -m pytest /home/wheeltec/workspace/wheeltec_motion_ws/src/wheeltec_rgbd/test -q
bash /home/wheeltec/workspace/wheeltec_motion_ws/scripts/start_debug_rviz.sh --check --minimal
```

算法/ROS证据在 `~/workspace/conavgpt2_ws/runtime_logs/source-agent/{delivery-tests.log,final-ros-tests.log}`；相机/静止/三次运动证据在本仓库 `runtime_logs/conav/acceptance/20261012/`，最终为 `source-motion-final.log`、`motion-final/report.json`、`motion-final/decisions.jsonl`、`motion-final-recording.json`及samples文件。原始日志不入Git。现场教程见 SOURCE_AGENT，源码差异见开发分支 WHEELTEC_ADAPTATION。

收尾已停止并关闭本轮基础栈、相机、探索、记录及RViz。奥比地面高度0.1157m、完整外参confirmed=false；D455未参与。下一步是现场更长探索、地图/低障碍投影、边界与避障、真实物理停车测试，随后才接检测/VLM；不宣称完整Co-NavGPT2任务或多车系统完成。以下条目为旧测试预览及历史阶段结果。

## 2026-10-12：论文式前沿策略与启动扫描验证完成

本轮按用户确认的论文流程补齐点投影已观测层、最大轮廓减膨胀障碍的前沿、随机前沿选择、无前沿全图随机候选和按位姿累计启动旋转。2026-10-10 开始时先调用 stop 并关闭本轮所有实车程序；2026-10-12 收尾期间保持关闭，没有读真实相机或发实车运动。

| 验证输入 / 操作 | 实际结果 | 证明范围 |
| --- | --- | --- |
| 三组已观测/障碍栅格：矩形、障碍分隔、大小区域并存 | 本车前沿点结果与从锁定原源码抽取的 Global_Map_Proc.Frontier_Det 相同 | 当前前沿几何对齐；不证明真实点云质量 |
| 全未知地图、已知墙体和断开区域、固定不可用随机候选 | 可在未知区选目标但地图保持未知；不强行清除障碍，有限尝试耗尽后停止建议 | 原版式未知规划/兜底及本车有效性约束 |
| 前沿进入0.25 m到达容差、出现新的合格前沿 | 已到达前沿不再反复重选；新前沿优先于旧全图随机目标 | 消除 FRONTIER_REACHED 循环，不以删除到达容差推进 |
| 新配对位姿跨±π、重复图像、静止/反向旋转、超时及近障碍 | 只有有符号实际转角推进扫描；超时/阻挡给零建议并清图锁存，reset后重新等待输入 | 扫描状态与异常行为；不是实测旋转精度或碰撞验收 |
| 独立ROS域188，合成RGB-D/Odometry | 实际收到 INITIAL_SCAN→EXPLORING/PLANNED、分层OccupancyGrid、带清除/寿命的MarkerArray；断流/位姿跳变仍清图，显式reset恢复 | ROS数据契约与旧保护未回归；入口没有 /cmd_vel 发布者 |
| 300步数值房间闭环 | INITIAL_SCAN 252次、PLANNED 48次，累计转角6.30 rad、3个目标；平均处理18.9 ms、最大42.8 ms，motion_output=false | 虚拟扫描完成后能持续规划；不是实车导航或环境覆盖证明 |

最终命令及结果：`WHEELTEC_EXPLORATION_TEST_DOMAIN_ID=188 bash scripts/test_exploration.sh`，58项通过（23.22 s，数量仅用于漏跑检查）。其中包含旧known_free对照、官方FMM、相机配对输入、故障/reset和实际控制节点的隔离测试出口。只有 synthetic_output 允许测试非零速度，未连接底盘串口或实车 /cmd_vel。

300步demo：`bash scripts/start_exploration_preview.sh --demo --steps 300 --output runtime_logs/exploration/paper-policy-demo`。构建wheeltec_exploration成功，脚本语法、`--check`及diff检查通过。中间测试曾把首个空地图消息误当作有效地图，已改为等待有尺寸的地图；最终近处到达处理及退出状态也已复验。

证据：`runtime_logs/exploration-test/paper-policy-final.log`、`paper-demo.log`、`paper-build.log`，以及 `runtime_logs/exploration/paper-policy-demo/` 的报告、分层map.npz和模拟轨迹。旧中间日志保留；本轮不重复实车动作或相机验收。

当前默认前沿策略允许未知参与FMM，与此前known_free限制不同；未知颜色仍保留，不能据此宣布路径实际无障碍。启动旋转按一圈反馈替代原版处理步数，本车半径/限速/时间门槛保留；完整对应差异见 EXPLORATION_POLICY。现场入口仍只读，实车扫描及运动桥联合验收待后续执行。

## 2026-10-10：RViz 地图探索调试入口

新增 `scripts/start_debug_rviz.sh` 和 `config/rviz/exploration_debug.rviz`，只打开可视化，不启动硬件、规划或运动。验证内容：

- `bash -n scripts/start_debug_rviz.sh`、默认/Astra 与 D455 的 `--check`、`--help` 均通过；检查模式不启动图形或 ROS 节点。
- 在独立 ROS 域 188、ROS_LOCALHOST_ONLY=1、Xvfb 虚拟显示下实际运行入口约 22 s，发布合成 120×120 地图、目标/短期目标、Odometry、Path 和 VGA RGB/16UC1 深度。七路启用显示均有一个订阅者，实际截图可见地图、彩色目标与轨迹及两个图像窗口。
- 图像发布使用 Best Effort，地图为 Reliable + Volatile，验证布局能够接收对应 QoS；未启动串口、相机、雷达、FAST-LIO 或运动控制。`/cmd_vel` 和导航速度输入的发布者均为 0，结束关闭验证 RViz、合成源和 Xvfb。
- 虚拟显示/Mesa 首次创建地图时有一次 GLSL sampler 日志，截图中地图仍正常呈现；合成源未发布 TF，Global Status 有固定帧警告。此验证证明实际加载、订阅和渲染，不代表实车地图正确、Orin 桌面 GPU 无警告或相机外参验收。实际桌面表现由用户运行入口检查。

证据：`runtime_logs/visualization/validation/` 中的 `report.json`、`layout.png`、`rviz.log` 和合成验证脚本。D455 仅验证参数/布局生成逻辑，未打开 D455 设备；本次没有实车运动。配置/脚本直接从仓库读取，无需重建 ROS 包；其余算法/控制未改，不重复整仓回归。

## 2026-10-10：相机更新与配对等待修复完成，静止整链连续性复验通过

问题分为两段：厂家驱动的小 CameraInfo 稳定约 29 Hz，而大图像与原子 RGB-D 明显缺失；默认 Fast DDS 共享内存段约 512 KiB，当前一组 RGB-D 像素约 2.15 MB。仅将相机发布进程段容量改为 32 MiB 后大图像接收恢复。随后仍出现新图像不断替换配对等待对象、以及等待图像落在不可配对位姿间隙内的停滞，已分别修正。没有提高图像超时、同步容差或定位门槛。

| 真实输入验证 | 结果 | 所证明的行为 |
| --- | --- | --- |
| 原配置，约 90 s，定位栈与同步诊断负载 | 彩色 18.31 Hz、深度 17.69 Hz、CameraInfo 约 29 Hz、RGB-D 15.09 Hz；RGB-D 最大接收间隔 0.380 s；消费读取有 106 次 NO_NEW_OBSERVATION | 原始大图像与小元数据接收不一致，支持检查传输容量，不能仅认为相机整体停流 |
| 仅扩容发布进程共享内存，90 s，并加入只读预览 | RGB-D 29.07 Hz、最大接收间隔 0.110 s，消费读取 899 次均为新帧，诊断 READY；旧配对逻辑仍在 73 帧后停住 | 传输配置改善已独立证实；相机持续更新不等于探索配对持续推进 |
| 正式入口、初版单等待帧，180 s | RGB-D 28.48 Hz、最大接收间隔 0.099 s，无 STALE；探索处理到 1397 后末段约 25 s 锁存 NO_TIME_MATCHED_POSE | 该中间实现未通过整链验收；进一步回归证明不可配对的等待帧不能一直保留至过期 |
| 最终配对逻辑，300 s，同时有真实定位/诊断/预览负载 | RGB-D 28.81 Hz、最大接收间隔 0.123 s，无 STALE；探索持续处理 2480 组、最大推进间隔约 0.331 s，结束保留地图且正常 EXITED | 本轮静止输入下传输和配对连续性通过；偶发等待会恢复，没有永久冻结或自动 reset 掩盖问题 |

最终代码真实启动补充复验 25 s：RGB-D 约 29.15 Hz、最大接收间隔 78.3 ms，诊断持续 READY、适配器 rejected=0，处理最大约 51.1 ms；验证预加载后首帧与后续更新正常。结束时主动 stop 并关闭本轮相机、受管栈和记录器。

最终五分钟配对差值均 ≤0.05 s（观测最大约 49.994 ms）；位姿源年龄最大约 94.1 ms，探索单次处理最大约 110.5 ms。448 次 NO_TIME_MATCHED_POSE 为短暂等待，后续继续处理；2 次 PROCESSING_EXPIRED 输出零建议，不以计算成功掩盖源年龄到期。5874 条实际 /cmd_vel 采样均为零，控制器唯一发布、未启用导航或发送 Action；本轮仅做静止观测，不是物理制动试验。

最终实现：

- Astra launch 仅为 Fast DDS 驱动/适配器子进程加载随包安装的 fastdds_camera.xml（32 MiB + UDPv4）；尊重调用方已有显式 DDS 配置和其他 RMW，不改厂家源码或全局环境。
- PosedRGBDReader / 预览优先最新可配对图像，最多保留一组尚新鲜的等待图像；位姿时间已经越过且仍不能配对的等待图像及时替换。原始源时间、原始单调接收时间、缓存失效 epoch 同时约束，不重新盖时间戳；过期、失效、定位异常/重置后清除。
- 回归还复现了首帧 OpenCV 懒加载耗时导致 STALE_OR_FUTURE_STAMP；图像后端现在在建立订阅前加载，避免把初始化计入首帧有效期。延迟处理仍按原有效期拒绝，不延长超时。

验证命令及最终结果：两包 symlink build 成功；独立 ROS 域185/186、ROS_LOCALHOST_ONLY=1 下相机完整回归及探索回归通过，包括跨进程 2 MB 级 VGA 连续传输、相位偏移、位姿间隙、时间冻结/失效/重置、处理超时、旧图像不重放。最终相机 58、探索 35 仅作漏跑核对，不作为功能完成的证明。早期回归失败与定位记录保留在原始日志，最终结果见 rgbd-final-tests.xml 和 exploration-final-tests.log。Astra 和探索 --check 均通过，git diff --check 通过。

本轮所有实际处理阶段仍为 NO_REACHABLE_FRONTIER，没有 PLANNED：相机与位姿输入稳定，当前场景未提供可达探索目标。需要后续检查地图自由空间、视野及前沿条件，再接实机速度出口；没有验证运动场景建图、相机外参物理准确、避障或完整 Co-NavGPT2 自主运行。

证据：runtime_logs/acceptance/20261010-camera-fix/ 下 baseline、large-shm、final（保留中间未通过结果）、final2（最终五分钟）、startup（最终代码启动复验）、测试 XML/日志；最终配对原始记录 matching.jsonl，统计 exploration-summary.json / summary.json。目录不入 Git。

## 2026-10-10：真实整链只读运行仍未通过连续性验收

基础动作完成后，按用户请求启动奥比 RGB-D、FAST-LIO、随机前沿/FMM 预览；不接 VLM、不实例化运动桥、不发送运动目标。定位首次启动约 1.3 s 延迟，经受管重启恢复后才开始 70 s 预览。运行使用当前 Z=0.1157 m 的相机配置，保持 confirmed=false。

- 共实际处理 48 组观测，处理阶段均为 NO_REACHABLE_FRONTIER；没有产生 PLANNED，故当前场景尚不能验证 FMM 有效运动目标。
- 随后 RGB-D 消费端保持同一源时间戳，源年龄从约 0.102 / 0.201 / 0.301 / 0.401 s 增长至触发 STALE。下一次读取已恢复新帧并成功配对（年龄约 0.083 s，配对差约 15 ms），但探索按故障策略保持 STALE 并清地图，processed 停在 48，直至正常退出。没有通过自动 reset 掩盖故障。
- 配对诊断记录有 62 次 NO_TIME_MATCHED_POSE，但本轮首次终止连续处理的原因是观测 STALE；不能把全部问题归因为配对。故障后重复读取成功的记录不代表处理了新的地图帧。仍需区分相机驱动、传输、适配器和消费者造成的观测间歇。
- 预览期间定位源年龄最大约 80.1 ms，处理耗时最大约 51.8 ms；14 条 PROCESSING_EXPIRED 来自运行时的源年龄/耗时联合门槛，不能据名称认定计算超过 0.3 s。1387 条采样 /cmd_vel 全为零。
- 预览正常退出，随后主动 stop 并关闭本轮相机、受管栈和记录器。结果证明链路能启动、取流、配对并尝试建图/选目标，未证明连续探索、外参物理准确或完整自主运动闭环。

证据：runtime_logs/acceptance/20261010-exploration/ 下 matching.jsonl（仅诊断包装，不改生产逻辑）、events.jsonl、summary.json、preview/report.json、启动日志；受管目录 runtime_logs/robot_stack/20261010-135611-10193/。

接下来先定位观测间歇并验证恢复流程，再检查当前场景的可达自由空间/前沿，随后接通已准备的 DecisionBridge 实机入口。当前 readonly runtime 没有运动出口；不能把基础 Action 完成当作完整 Co-NavGPT2 已接通。

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
