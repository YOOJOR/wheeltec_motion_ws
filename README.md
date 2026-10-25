# CoNavBot 实机部署与 WHEELTEC 运动工作区

本仓库负责路径跟踪、相对运动、D455 启动、基础栈管理和可视化。感知建图与 FMM 规划见相邻 [conavgpt2_ws](../conavgpt2_ws/README.md)。开发约束见 [AGENTS.md](AGENTS.md)，历史见 [CHANGELOG.md](CHANGELOG.md)。

## 当前状态

- Agent 输出前方约1 m路径；Navigation 以20 Hz根据实时定位跟踪。motion 不处理地图、障碍判断或局部绕行。
- motion 接口包和控制包已于2026-10-23在 x86_64 ROS Humble 容器构建成功，关闭 BUILD_TESTING；此次未启动节点，车载构建及路径跟踪效果待确认。
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
- 最高速度0.15 m/s、0.35 rad/s；请求源年龄和接收年龄各0.3 s，位姿0.5 s，默认动作超时45 s。速度按上限截断，正常运动使用加速度限制；stop、取消和超时直接归零。

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

路径以本次决策开始时间标记，路径和各点 frame_id 均为 camera_init。每条新路径替换旧路径，不重置输出速度；相邻重复点合并，单点作为位置目标，姿态字段不参与控制。Navigation 先投影到最近路径段，再向前取 `path_lookahead=0.25 m`；航向比例控制使用 heading_gain，偏角≥60°时立即停止平移、原地转向，偏角较小时按 cos(偏角) 降低前进速度。路径末端按 distance_gain 减速，进入 distance_tolerance 后归零。状态为 WAITING_PATH / TRACKING / REACHED；到达后仍受0.3 s路径超时约束。状态 requested_linear_x/angular_z 表示跟踪器加速度限制前的目标速度。

启动扫描异步执行旋转2π的 Action（45 s超时、使用 --turn-speed），期间持续建图。成功后才启用 Navigation，等待启用后的新观测生成路径。只读运行不提交扫描，--skip-initial-scan 跳过扫描。数据故障、预算耗尽或退出均 stop；新鲜路径显示在 RViz 的 Agent navigation path（/conavbot/path）。

相对运动的 move 单位 m、rotate 单位 rad，正号前进/左转。直行使用起始航向投影计算距离，并进行航向纠偏；距离与航向都进入容差即归零并返回成功。横向偏差只作反馈。旋转按累计转角控制，支持跨越 ±π；进入角度容差即完成。动作请求可指定速度和超时，速度仍受车辆上限约束。直接结束客户端不能替代 stop。

运动控制集中在 `node.py`。YAML 分为启用与安装、速度与加速度、增益与容差、超时；参数修改后重启生效。控制20 Hz、反馈5 Hz，源时间超前容许量0.1 s。

## 相机与时间契约

D455 与 FAST-LIO 独立发布感知和定位。相机由 [start_d455.sh](scripts/start_d455.sh) 启动，参数集中在 [d455.yaml](config/d455.yaml)：640×480、15 Hz、RGB8/Z16、同步和深度对齐；关闭点云、IMU及红外输出。单独执行 `bash scripts/start_d455.sh --start`，可追加 `serial_number:=设备序列号`；已有统一栈时不要重复启动。

直接发布 `/camera/d455/raw/d455/` 下 `color/image_raw`、`aligned_depth_to_color/image_raw` 及各自 `camera_info`。

导航端后台接收四路消息，待配对缓存每路两条，完整观测两组。图像与内参同戳，RGB/深度差≤30 ms；按深度时间取最近位姿，差≤50 ms，不外推。仅解码最新可配对的新观测，毫米转米，0.4–6 m外及无效深度置NaN；原版Agent仍有4 m截断和边缘处理。标定变化时重新检查及生成去畸变映射。定位失效仍锁存，显式重置清除旧地图与缓存。

基础栈监控四路数据更新，不判断像素有效率；全零深度由导航端报告，不重启相机。D455安装外参为空，采集可独立运行，世界建图入口拒绝缺失外参。

## 获取与部署

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
- 请求超时：核对新观测/新请求是否持续，不放宽0.3 s保护或重发旧决定。
- 定位异常：核对camera_init/body、时钟、源年龄和重启；`topic delay`包含扫描/处理/调度，不等于网络耗时。
- 启动失败：看 `runtime_logs/robot_stack/` 下supervisor及组件日志。通信故障保留定位；Livox/FAST-LIO故障按依赖重启，旧任务必须结束。

只读记录器（需要已启动的输入）不保存像素、不启用运动；额外订阅可能增加负载：

```bash
python3 scripts/record_conav_acceptance.py --duration 60 --sensor-timing --output runtime_logs/conav/acceptance/new-run.json
```

离线旋转拟合：`ros2 run wheeltec_motion_control calibrate_rotation --bag /绝对录包目录 --output /新的输出目录`；也支持 `--csv`。工具不发速度、不改配置；平面旋转不能独立确定高度或厂家几何中心，`--body-z`须来自测量及坐标换算，建议不得直接覆盖车辆YAML。

软件测试及临时模拟代码禁止新增或运行，具体约束见 AGENTS.md。日志、录包和地图不提交 Git。
