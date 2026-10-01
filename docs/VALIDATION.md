# 当前验证摘要

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
