# CoNavBot 实机部署与 WHEELTEC 运动工作区

命名迁移：导航使用 `conavbot` 模块与 `/conavbot/*` 话题；D455 使用 `/camera/d455/raw/d455/` 话题及 `d455_color_optical_frame`。通用启动环境变量使用 `CONAVBOT_*`；WHEELTEC 运动接口名称保留。部署时须同步更新两个仓库并重启相机、导航及相关监控/显示进程，不混用旧命名。

本仓库提供实机运动控制、D455 独立启动、基础栈管理和可视化。探索主实现位于相邻的 [conavgpt2_ws](../conavgpt2_ws/README.md)，直接使用原版 Co-NavGPT2 Agent；早期 `wheeltec_exploration` 测试包已移除。

截至 2026-10-21：有限低速链路曾接通，持续探索尚未验收。AI 开发约束见 [AGENTS.md](AGENTS.md)，历史见 [CHANGELOG.md](CHANGELOG.md)。

## 当前状态与下一步

TODO：D455 部署与安装几何（优先完成）

- [ ] 车载部署新启动配置，核对已安装 RealSense 驱动的参数支持及实际读回值：640×480、15 Hz、RGB8/Z16、同步、对齐、话题名称和光学坐标系。清理部署中已删除包的旧构建/安装残留，避免旧入口继续生效。
- [ ] 测量并填写导航配置中的 D455 `base_from_camera`（底盘到相机光学坐标的安装变换），核对轴向和单位；固定目标投影、地面高度与转动一致性确认后再设置 `confirmed`。
- [ ] 外参确定后补齐 RViz 中底盘到 D455 的安装 TF；驱动继续提供相机内部 TF，避免重复发布同一条关系。
- [ ] 现场核对总启动/停止、D455 独立运行和断流恢复；相机恢复不重启 FAST-LIO，全零深度不触发驱动重启。

现场核验须单独授权；不新增或运行测试代码、模拟或合成验证。导航端待办见 [conavgpt2_ws](../conavgpt2_ws/README.md#已知问题与后续工作)。

TODO：根据目标与地图独立执行运动（方向待细化）

- 接收 Agent 输出的目标位置和持续更新的地图。
- 根据实时定位与障碍信息跟踪目标、计算速度，运动执行独立于 Agent 循环。
- 结合最新障碍和数据时效处理减速、停车及目标更新。
- 地图/目标接口、跟踪方式和具体执行规则留待后续确定。

| 已有证据                              | 证明范围与限制                                                      |
| --------------------------------- | ------------------------------------------------------------ |
| 合成 RGB-D/位姿及隔离 ROS 测试             | 坐标、配对、导航/Action互斥、超时归零及恢复不重放；不能证明真实投影或物理停车                   |
| 2026-10-12 有限实车：完成启动扫描后转向、前进、预算停车 | 定位估计前后位移约 0.215 m，用户确认实际停稳；不是尺测精度、避障距离或房间覆盖验收                |
| 2026-10-12 扩大范围失败                 | 一帧进入 Agent 时已旧 0.263 s，计算约 0.054 s 后超过 0.3 s 门槛并停止；延迟根因未定位   |
| 2026-10-13 原始图像更新、RGB-D停更         | NO_USABLE_DEPTH；停车后三帧约75%为零，其余正值全大于3 m，当前0.4–2 m过滤后无点；用户确认停稳 |
| 2026-10-15 base_link迁移            | 仅合成几何/隔离ROS验证，尚无迁移后实机投影验收                                    |
| 2026-10-16 普通电脑审阅                 | 控制核心/导航合成测试与源码检查通过；没有重跑完整ROS或Agent运行测试，没有硬件操作                |

下一步核验D455数据时序、深度有效性和安装外参，再核对固定目标投影、地面高度和转动一致性。完整覆盖、低障碍/盲区、物理停车时间、长期稳定性、语义任务和多车协作未完成。D455 接口可用但安装外参未配置。

结构化验收资料保留在车载电脑：本仓库 `runtime_logs/conav/acceptance/20261012/`、`20261012-extended/`、`20261013-timing/`，以及 `runtime_logs/validation/base-link-migration-20261015/`。部分旧文本日志已清理，不保证旧文件仍存在；本轮未重新读取这些车载资料。旧文档可从整理前提交 `b4df124` 查阅，无需默认加载。

## 目录与数据流

```text
D455 → RealSense驱动 → RGB/深度/内参 ─┐
MID360S → Livox → FAST-LIO → 位姿 ├→ conavgpt2_ws 原版Agent → 新鲜导航请求
                               └→ wheeltec_motion_control → 厂家底盘 /cmd_vel
```

| 位置 | 用途 |
| --- | --- |
| src/wheeltec_motion_control | node.py 集中完成运动控制；导航客户端、动作命令行和离线标定各自单文件 |
| src/wheeltec_motion_interfaces | ExecuteMotion Action、MotionStatus |
| scripts/d455.launch.py、config/d455.yaml | 独立D455驱动；导航端直接接收和配对 |
| scripts/robot_stack | 基础栈依赖检查、启动与恢复 |
| scripts/start_conav_exploration.sh | 相邻导航仓库的主Agent入口 |
| config/rviz | 地图及传感器显示布局 |

## 坐标、参数与接口

当前参数源为 [motion_control.yaml](src/wheeltec_motion_control/config/motion_control.yaml)；新车使用禁用输出的 [example](src/wheeltec_motion_control/config/motion_control.example.yaml)。配置中的启用/确认开关不等于全部机械测量已经完成。

- 底盘参考点是厂家四轮几何中心 `base_link`，`base_footprint` 在其下方 0.026 m。
- MID360S 后装：`T_body_base` 平移 `[-0.10718,-0.02237,-0.17788] m`、零RPY；XY暂保留旧旋转拟合作安装估计，未独立量到几何中心。不要使用厂家 laser_link/radar 代替。
- 历史奥比相机 `base_link → camera_link` 平移 `[0.16083451891873,0.000745202407130424,0.089704] m`、零RPY，组合离地0.115704 m；光学轴x右/y下/z前显式转为底盘x前/y左/z上。这是历史奥比安装信息；当前D455无安装外参。
- 完整三维计算 `T_world_base=T_world_body*T_body_base`，相机再乘 `T_base_optical`；地面点为 `T_world_base*[0,0,-0.026,1]`，倾斜时也如此。
- 最高速度0.15 m/s、0.35 rad/s；请求源年龄和接收年龄各0.3 s，位姿0.5 s，默认动作超时45 s。速度按上限截断，正常运动使用加速度限制；stop、取消和超时直接归零。

| 接口 | 契约 |
| --- | --- |
| /Odometry | camera_init/body，FAST-LIO IMU位姿；不是底盘点 |
| /wheeltec_motion/navigation/cmd_vel | TwistStamped，仅base_link的linear.x/angular.z，其他分量必须为零 |
| /wheeltec_motion/navigation/set_enabled | SetBool；必须检查success，启用不直接产生运动 |
| /wheeltec_motion/execute | 相对直行/转角Action，与导航互斥；先检查accepted，再等成功Result |
| /wheeltec_motion/stop | Trigger；退出导航/停止Action并请求零输出 |
| /wheeltec_motion/status | 软件状态；须检查时间新鲜度，不代表实测速度或物理停稳 |
| /cmd_vel | 由运动控制器唯一发布；上层不得直发或与手柄并行发布 |

每条导航请求必须源时间递增且晚于启用，旧/重复包不续期；新非法请求退出导航。显式零请求立即归零并保持导航；stop、失效、退出清缓存转IDLE。定位恢复后须重新启用导航，节点重启不恢复旧任务。进入VLM等长等待前明确stop，禁止刷新缓存决定的时间戳。

相对运动的 move 单位 m、rotate 单位 rad，正号前进/左转。直行使用起始航向投影计算距离，并进行航向纠偏；距离与航向都进入容差即归零并返回成功。横向偏差只作反馈。旋转按累计转角控制，支持跨越 ±π；进入角度容差即完成。动作请求可指定速度和超时，速度仍受车辆上限约束。直接结束客户端不能替代 stop。

运动控制代码集中在 `src/wheeltec_motion_control/wheeltec_motion_control/node.py`，没有独立 core/navigation 中间层。YAML 只保留四组：启用与安装、速度与加速度、增益与容差、超时。参数只读，修改后重启；控制频率固定20 Hz、反馈5 Hz，源时间超前容许量0.1 s。接口固定为上表名称，里程计坐标固定 camera_init/body。

横移纠偏、静止确认、无进展检测、定位跳变检测、连续位姿计数和调度缺口检测已删除；动作不设额外距离、转角和超时上限。到达成功表示当前定位进入容差。

## 相机与时间契约

D455 与 FAST-LIO 独立发布感知和定位。相机由 [start_d455.sh](scripts/start_d455.sh) 启动，参数集中在 [d455.yaml](config/d455.yaml)：640×480、15 Hz、RGB8/Z16、同步和深度对齐；关闭点云、IMU及红外输出。单独执行 `bash scripts/start_d455.sh --start`，可追加 `serial_number:=设备序列号`；已有统一栈时不要重复启动。

直接发布 `/camera/d455/raw/d455/` 下 `color/image_raw`、`aligned_depth_to_color/image_raw` 及各自 `camera_info`。旧 RGBDFrame、适配器/status 和奥比接入已移除。

导航端后台接收四路消息，待配对缓存每路两条，完整观测两组。图像与内参同戳，RGB/深度差≤30 ms；按深度时间取最近位姿，差≤50 ms，不外推。仅解码最新可配对的新观测，毫米转米，0.4–6 m外及无效深度置NaN；原版Agent仍有4 m截断和边缘处理。标定变化时重新检查及生成去畸变映射。定位失效仍锁存，显式重置清除旧地图与缓存。

基础栈监控四路数据更新，不判断像素有效率；全零深度由导航端报告，不重启相机。D455安装外参为空，采集可独立运行，世界建图入口拒绝缺失外参。

## 获取与部署

两个仓库放在同一父目录，已有目录先检查Git状态和远端，不覆盖修改：

```bash
git clone https://github.com/YOOJOR/wheeltec_motion_ws.git
git clone https://github.com/YOOJOR/conavgpt2_ws.git
```

普通电脑阅读无需ROS、CUDA、模型。车载环境为Ubuntu22.04/ROS2 Humble；Jetson依赖见导航仓库README。启动配置仍含车载路径，检查 `scripts/robot_stack/config.yaml`、导航 `conavbot/config.yaml` 及 `CONAVBOT_CONAV_ROOT`，不能直接把电脑clone当作已部署。

外部工作区：`wheeltec_base_ws` 为厂家通信/消息/serial三包；`vendor_ws` 为Livox/FAST-LIO。以下是已有部署来源，不授权修改厂家或第三方算法：

| 组件 | 锁定提交/注意事项 |
| --- | --- |
| 厂家三包 | 本地基线fe99b01；不上传厂家源码，按车型和设备配置 |
| Livox-SDK2 | 08f523c930b2f0ba1e98a6afaa8d7476bf479908 |
| livox_ros_driver2 | 4a1def929e5b59c7a8122d19fce6efba581ce9f7 |
| Ericsii/FAST_LIO_ROS2 | 2fffc570a25d0df172720bac034fbdb6a13d2162；ikd-Tree e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4 |

本车Livox主机IP为192.168.1.5；历史src雷达IP为.45、实际install为.145，重建前必须核对，不能假定源码等于运行配置。FAST-LIO已有部署关闭内部外参在线估计，内部LiDAR/IMU变换与底盘偏移分开。SDK旧构建缓存无法证明安装二进制来源；新车重新构建，不复制build/install/.venv。vendor增量构建显式选包，不能使用会清整个install的厂家build脚本。新车重新测量外参并从禁用模板开始。

已准备Humble及依赖后，在本仓库执行 `bash scripts/build.sh` 构建正式包；普通电脑不要为阅读运行它。启动后读回实际参数，YAML编辑不会自动重载旧进程。

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
- 请求超时：核对新观测/新请求是否持续，不放宽0.3 s保护或重发旧决定。
- 定位异常：核对camera_init/body、时钟、源年龄和重启；`topic delay`包含扫描/处理/调度，不等于网络耗时。
- 启动失败：看 `runtime_logs/robot_stack/` 下supervisor及组件日志。通信故障保留定位；Livox/FAST-LIO故障按依赖重启，旧任务必须结束。

只读记录器（需要已启动的输入）不保存像素、不启用运动；额外订阅可能增加负载：

```bash
python3 scripts/record_conav_acceptance.py --duration 60 --sensor-timing --output runtime_logs/conav/acceptance/new-run.json
```

离线旋转拟合：`ros2 run wheeltec_motion_control calibrate_rotation --bag /绝对录包目录 --output /新的输出目录`；也支持 `--csv`。工具不发速度、不改配置；平面旋转不能独立确定高度或厂家几何中心，`--body-z`须来自测量及坐标换算，建议不得直接覆盖车辆YAML。

2026-10-16审阅发现的导航侧异常回调外逃、地图正向边界索引问题见导航README，尚未修复。原始日志/录包/地图不进Git。

2026-10-20按用户要求移除测试用例、合成检查、离线模拟及旧探索测试包，历史验收结论保留。设备就绪检查、运行时数据校验和停车保护保留；不将静态检查写成实机复验。

软件测试代码已移除，禁止新增、恢复或运行测试套件、模拟验证和临时测试脚本，也不使用测试数量或通过率作为质量说明。代码改动仅做源码审阅、语法、依赖引用及配置核对；现场运动由用户操作。
