# CoNavBot 启动与调试

本工作区管理 WHEELTEC 底盘、Livox、FAST-LIO、D455、路径跟踪和桌面仿真。相邻 Agent 使用 RGB-D 与定位建图、选前沿并规划 FMM 路径。

[实机启动](#实机启动) · [实机调试](#实机调试) · [实机验证状态](#实机验证状态) · [桌面仿真](#桌面仿真) · [仿真调试](#仿真调试)

## 实机启动

车载环境为 Ubuntu 22.04、ROS 2 Humble 和 GNOME Terminal。底盘与雷达工作区、Jetson 虚拟环境按以下文档部署。 [工作区复刻](../README.md#复刻第二台车) · [Jetson 环境准备](../conavgpt2_ws/README.md#jetson-环境准备)

### 构建与启动基础栈

首次部署或更新 Motion 源码与消息接口后，使用现有构建入口。两份项目源码配套更新，`NavigationPath` 等接口由构建生成。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
bash scripts/build.sh --packages-select wheeltec_motion_interfaces wheeltec_motion_control
```

在小车图形桌面的终端运行以下命令。`--check` 检查安装前缀、配置及启动参数。启动命令由 tabs 管理底盘、Livox、FAST-LIO、Motion 和 D455。默认关闭 RViz，需要时给检查和启动命令都加 `--rviz`。若已有手动启动的组件，先结束对应进程，避免重复占用设备和话题。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
bash scripts/start_robot.sh --check && bash scripts/start_robot.sh
bash scripts/start_robot.sh --status
```

`--status` 给出受管组件状态和当前日志目录。等待各组件就绪并确认总控记录 `STACK READY`。此状态表示基础栈健康检查通过，RGB-D 与定位配对由下一步的 Agent 检查。单组件启动命令用于隔离诊断，整车开发使用上述入口。

### 静止建图

在另一个终端运行 Agent 20 秒。默认只读，使用实机 RGB-D 和 FAST-LIO 位姿建图并生成规划结果。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
bash scripts/start_conav_exploration.sh --start --duration 20
```

检查本轮 `report.json` 的 `processed`、`exit_reason` 和阶段地图。`processed` 应大于零，地图和坐标方向应与现场一致。只读运行可能在始终等不到配对时耗尽时长，因此要结合 `last_decision` 或 `decisions.jsonl` 判断，不能只看 `duration completed`。日志位置见[实机调试](#实机调试)。

### 有限运动与停车

确认只读结果和外参后，在本次试车已获授权、现场有人观察且能随时停车的条件下运行。下方为保守的几何导航示例，默认先原地扫描一圈，再选前沿并跟踪 FMM 路径。实车有限任务最长 120 秒。试车前按场地明确速度、时间、距起点位移及累计转角预算。时间和转角预算包含启动扫描，位移预算按定位估计的距起点距离计算。示例参数与历史试跑参数各自独立。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
bash scripts/start_conav_exploration.sh --start --motion --duration 45 \
  --forward-speed 0.05 --turn-speed 0.25 --max-distance 0.2 --max-rotation 8.0
```

在 Agent 终端按 Ctrl+C 可结束探索并请求停车。另一个终端可通过现有客户端直接请求停车，随后读回控制状态。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
python3 -m wheeltec_motion_control.navigation_client stop
python3 -m wheeltec_motion_control.navigation_client status
```

核对 `IDLE` 和零速度输出，并观察车体实际停稳。任务结束后用以下命令关闭全部受管组件。在受管标签页按 Ctrl+C 或关闭标签页也会结束基础栈。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
bash scripts/start_robot.sh --stop
bash scripts/start_robot.sh --status
```

## 实机调试

基础栈先用 `bash scripts/start_robot.sh --status` 找到当前会话目录。Agent 默认输出到 `runtime_logs/conav/latest/runs/<run_id>/`，指定绝对路径 `--output` 后查看对应目录的 `runs/<run_id>/`。先看 `report.json` 的 `processed`、`exit_reason` 和 `motion_status_before_stop`，再按需要读同轮 `decisions.jsonl`、`map.npz` 与 `stages/`。决策中的 `request_published` 表示 Agent 已发出请求，Motion 的接收与输出要结合自身状态核对。

| 现象 | 先查哪里 |
| --- | --- |
| 基础栈未就绪或反复重启 | 查当前会话的 `supervisor.log` 和对应组件标签页日志。 |
| D455 已启动但缺彩色或对齐深度 | 查相机日志、四路话题和内参。2026-11-12 曾报告 `Right MIPI error`，同配置重启相机后图像恢复。 |
| `NO_TIME_MATCHED_POSE` | 对照图像、雷达、IMU 和 `/Odometry` 的时戳。Agent 要求深度与定位相差不超过 50 ms，基础栈检查使用 Motion 的定位时效配置，两者用途不同。 |
| `NO_USABLE_DEPTH` 或 `NO_NEW_OBSERVATION` | 查看 RGB、对齐深度、内参及配对状态。 |
| `PROCESSING_EXPIRED` 或路径超时 | 分开检查 `input_age_sec`、`agent_sec`、`processing_sec` 以及 Motion 的路径状态。 |
| 地图方向或位置不对 | 核对 D455 安装外参、底盘参考点和定位会话。FAST-LIO 重启后重新启动 Agent 建图。 |
| RViz 显示占用较高 | 从本工作区运行 `bash scripts/start_debug_rviz.sh --minimal`，使用关闭图像与点云显示的布局。 |

定位时延排查使用现有只读脚本。等基础栈运行后，在独立 Bash 终端采样。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
source /opt/ros/humble/setup.bash
source ../vendor_ws/install/local_setup.bash
python3 scripts/diagnose_pose_delay.py --seconds 15
```

2026-11-12 和 11-13 均出现过 `STACK READY` 后仍无法将 RGB-D 与定位配对。11-13 的雷达帧末尾年龄约 10–19 ms，`/Odometry` 消息平均年龄 1.386 s，静止建图处理 0 帧。重启受管 FAST-LIO 后，`/Odometry` 平均年龄降至 52 ms，恢复约 10 Hz 输出，重启 Agent 后静止建图恢复。延迟根因待查。遇到同类问题，先结束 Agent 并确认停车，保存时延记录，再通过前文的整栈启停命令重新检查。定位恢复后先重做静止建图。若仍复现，根据当轮日志隔离排查定位。

需要记录轨迹、速度和路径时，使用项目已有的只读记录器。加 `--sensor-timing` 可同时记录图像时戳。记录器放在独立终端，先开始记录再启动任务，`--duration` 按启动和任务时长设置，须大于 0 秒且不超过 120 秒。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
../conavgpt2_ws/.venv/bin/python scripts/record_conav_acceptance.py \
  --duration 90 \
  --output "runtime_logs/conav/acceptance/run-$(date +%Y%m%d-%H%M%S).json"
```

任务结束时先停车，保持基础栈运行到记录器正常退出，再确认汇总 `.json` 和 `.samples.json` 已落盘。当前记录器在被 SIGINT 提前中断时可能因 ROS 上下文先关闭而归档失败。缺采样时，可从有效规划路径首点还原对应的 FAST-LIO 定位轨迹，精度结论按其数据来源标明。记录器当前不采集整机 CPU、GPU 或内存峰值。

| 配置 | 用途 |
| --- | --- |
| [robot_stack/config.yaml](scripts/robot_stack/config.yaml) | 工作区路径、组件命令及日志目录。 |
| [motion_control.yaml](src/wheeltec_motion_control/config/motion_control.yaml) | 首台车底盘外参、速度与时效。当前速度上限 0.15 m/s、0.35 rad/s。改动后重启控制节点。 |
| [motion_control.example.yaml](src/wheeltec_motion_control/config/motion_control.example.yaml) | 新车从禁用输出的模板开始，使用本车标定。 |
| [d455.yaml](config/d455.yaml) | 相机分辨率、频率与深度对齐。 |
| [Agent config.yaml](../conavgpt2_ws/development/Co-NavGPT2/conavbot/config.yaml) | 相机安装外参与 Motion 配置路径。 |

## 实机验证状态

2026-11-13 在首台 WHEELTEC 上按上文的 tabs 和 Agent 入口验证。基线为 Motion `f3948f41` 与 Agent `0fdd39c`，Agent 有限运动时长上限在本地由 60 秒提高到 120 秒。实车使用 D455 RGB-D、FAST-LIO、几何前沿和 FMM，VLM、物体检测与分割均关闭。

| 运行 | 观测结果 |
| --- | --- |
| 定位恢复后的静止建图 | 20 秒内处理 141 帧，生成几何地图、彩色地图和关键帧，共 2 次阶段导出。 |
| 首轮几何探索 | 累计转角达到 18 rad，约 92 秒时停车。Agent 判定到达 5 个前沿，路径跟踪阶段累计约 4.2 m，共 5 次阶段导出。 |
| 第二轮几何探索 | 完成 120 秒任务，含启动扫描。处理 864 帧，Motion 收到 684 个路径请求，Agent 判定到达 7 个前沿。约 96.5 秒的路径请求区间内累计约 7.1 m，起终点直线距离约 4.0 m。停车后读回 `IDLE` 和零速度，再关闭基础栈。共 7 次阶段导出。 |

第二轮使用 `--motion --duration 120 --forward-speed 0.10 --turn-speed 0.30 --max-distance 4.0 --max-rotation 38.0 --map-size-cm 1200 --export-interval 15`。里程按约 1 秒间隔的 FAST-LIO 平面轨迹估算，路径图同时保留采样的 FMM 短路径。前沿到达由 Agent 按 0.10 m 容差判定，任务中车到当前目标的最大瞬时距离约 2.58 m。发布路径的决策处理平均约 80 ms、最高 147 ms，记录到的最大路径请求年龄约 125 ms、最大定位消息年龄约 170 ms。运行期间未连续采集整机资源峰值。

两分钟记录支持首台车基础几何闭环初步可用。更远的固定目标寻路、长时间稳定性、全程跟踪误差、障碍规避和实机语义搜索待验证。Motion 当前未接独立雷达避障输入。

2026-11-12 的短测约 45 秒，距起点位移约 0.97 m。首个前沿进入 0.10 m 容差，第二个前沿途中触发转角预算。首个前沿的定位采样点相对首条规划路径的平均平面距离约 3.3 cm、最大约 8.6 cm，使用决策日志还原的定位点。该轮几何处理平均约 46 ms，95% 在约 67 ms 内。

### 本机证据

两个原始运行目录已备份到工作区根目录 `archive/2026-11-13-conavbot-phase/`。下表路径以其中的 `20261113-8ZUgXj/` 为起点。本机仍可从 `runtime_logs/geometry_extended/20261113-8ZUgXj/` 查看原件。

| 证据 | 归档路径 |
| --- | --- |
| FAST-LIO 重启前后的时戳 | `timing-before.log`、`timing-after.log` |
| 初次静止建图失败 | `stationary/runs/c650cf4070af47bd8629558d2f661348/report.json` |
| 静止建图复测 | `stationary-recheck/runs/8c7eb4f2fea2447aadb265bcc4172c03/` |
| 首轮探索及转角预算 | `exploration/runs/12893743e4a84854b4a8125597ed50c8/`、`motion-record.json`、`motion-record.samples.json` |
| 120 秒探索及地图、轨迹图 | `exploration-full/runs/659cc2d002e040e8a62c7f98d9cfa255/`。先看 `stages/stage_0007/overview.png`、`path_tracking_review.png` 和 `report.json`，原始决策与地图为 `decisions.jsonl`、`map.npz`。 |
| 120 秒探索的独立记录和停车读回 | `full-motion-record.json`、`full-motion-record.samples.json`、`full-final-motion-status.txt` |
| 离线检测、掩码和 Orin 对照 | `offline-segmentation/`。[条件与结果](../conavgpt2_ws/README.md#离线识别与-orin-推理) |

11-12 的短测在同一归档下的 `20261112-E0BLhW/`，原件在 `runtime_logs/basic_motion/20261112-E0BLhW/`。最终任务位于 `navigation-final-agent/runs/9fab1da4e4d5442a8170db9cd9ea4d85/`，停车与定位诊断分别记录在 `final-stop.json`、`pose-timing.log` 和 `pose-timing-after-restart.log`。

## 桌面仿真

在装有 Podman 的桌面终端执行。两个仓库应并列放置，示例使用 `~/workspace`，其他目录请替换工作路径。首次运行先 `build`，之后用 `up` 打开 Gazebo 和 RViz。仿真车默认静止，后文读取报告的示例需要 `jq`。

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_sim.sh build
bash scripts/start_sim.sh up
```

场景是 12×10 m 的简化住宅。客厅通过两个 1.6 m 门洞分别通往厨房和卧室，冰箱位于厨房北侧，正面朝南。等 RViz 中的图像显示稳定，再选择操作方式。

| 要做什么 | 操作 |
| --- | --- |
| 指定一个导航目标 | 在 RViz 选 **2D Goal Pose**，在空地点击并拖动后松开 |
| 静止观察并保存地图 | `bash scripts/start_sim.sh preview`，采集约 20 秒 |
| 静止识别冰箱 | `bash scripts/start_sim.sh preview-sam`，默认采集三帧。首次使用先准备模型 |
| 开始自主探索 | `bash scripts/start_sim.sh explore`，仿真车会先旋转扫描再开始探索 |
| 搜索冰箱 | `bash scripts/start_sim.sh search-object`，接入 VLM 前沿探索、三维目标导航和到达停车，闭环效果待人工验证 |
| 回到 RViz 点目标模式 | `bash scripts/start_sim.sh goal` |
| 停止当前任务 | `bash scripts/start_sim.sh stop` |
| 仿真急停 | `bash scripts/start_sim.sh estop`，清空指令队列，车体仍需物理制动 |
| 恢复初始场景 | `bash scripts/start_sim.sh reset`，回到 RViz 点目标模式 |
| 关闭仿真 | `bash scripts/start_sim.sh down` |

切换模式时会停止旧任务并保留当前位置。仿真持续探索需要手动停止，或在故障时退出。

### 静止冰箱分割

首次使用时构建镜像、显式准备模型，再启动仿真。已有容器如果仍引用旧镜像，先在本目录执行 `bash scripts/start_sim.sh down`。这会关闭现有仿真会话。

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_sim.sh build
bash scripts/start_sim.sh prepare-sam
bash scripts/start_sim.sh up
```

`prepare-sam` 下载 YOLO-World、MobileSAM 和 CLIP ViT-B/32 权重，并把来源和 SHA-256 写入 `runtime_logs/sim/models/manifest.json`。识别参数集中在 `simulation.yaml`。默认 CPU、单类别 `refrigerator`、置信度门槛 0.25、每帧最多三个框。缺少权重时识别会报错。

在 RViz 用 **2D Goal Pose** 将车开到已实测的厨房斜侧观察点 `(3.15, 1.35)`，拖动箭头朝向冰箱 `(4.8, 4.2)`，约 `1.05 rad`。等车停稳，RGB 中能同时看到冰箱正面和左侧后运行下面两条命令。

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_sim.sh preview-sam
bash scripts/start_sim.sh status
```

识别会退出当前导航任务，在原地逐帧处理完整的 640×480 图像。RViz 的 **Refrigerator segmentation** 显示类别、YOLO 检测分数、框和半透明掩码，可拖动面板下方的分隔线放大图像。进程结束后以本轮文件为准。`status` 显示 run ID、阶段和结果，宿主结果在 `runtime_logs/sim/agent/runs/<run_id>/segmentation/`，同级 `report.json` 记录模型加载和推理耗时。每帧以同一时间戳保存 RGB、叠加图、JSON 和命中时的二值掩码。空检测保存原图、标有 `no refrigerator detected` 的叠加图和空实例列表。

隔离仿真在斜侧观察点的三帧均检测到冰箱，YOLO 分数约 0.804，掩码覆盖可见柜体。CPU 冷加载约 5.94 秒，含 MobileSAM 的单帧推理约 1.42–2.08 秒。正面观察点 `(4.5, 1.05)` 的三帧均为空检测。两个门洞在静态地图中各留出 32 格，跨门洞行驶待人工观察。若 RGB 一直没有更新，先查看适配器是否已建立 `SIM_EPOCH_OFFSET_NS`。时钟未稳定时，适配器会等待，识别会报告采集超时。

用下面的命令中止识别并回到 RViz 点目标模式。

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_sim.sh stop
bash scripts/start_sim.sh goal
```

### VLM 探索

默认的 `explore` 使用几何前沿。`explore-vlm` 有多个合格前沿时会发送同一份 RGB-D 彩色俯视地图的多张 JPEG，每张标出 R0 和一个完整前沿。模型在 `robot_0` 字段返回编号，并提供 `reason`。只有一个候选时直接规划，无候选时沿用随机回退。选点前停车，回复后用新观测重规划。探索的物体检测关闭，静止分割仍用 `preview-sam`。[Agent 协议与上游差异](../conavgpt2_ws/README.md#vlm-前沿选择)

首次配置时复制模板，然后编辑 `runtime_logs/sim/vlm.yaml` 的 `base_url`、`model` 和 `target`。已有配置时直接编辑。住宅冰箱任务使用 `target: refrigerator`。百炼的 `base_url` 使用业务空间的 OpenAI 兼容地址，末尾为 `/compatible-mode/v1`。密钥环境变量使用模板中的 `CONAVBOT_VLM_API_KEY`。

```bash
cd ~/workspace/wheeltec_motion_ws
mkdir -p runtime_logs/sim
cp -n ../conavgpt2_ws/development/Co-NavGPT2/conavbot/vlm.example.yaml runtime_logs/sim/vlm.yaml
```

用 Bash 执行下方命令。使用 fish 时先输入 `bash`。密钥在终端中输入，`up` 会将其传给新会话。已有会话需要先关闭再启动。

```bash
cd ~/workspace/wheeltec_motion_ws
read -r -s -p 'VLM API key: ' CONAVBOT_VLM_API_KEY
printf '\n'
export CONAVBOT_VLM_API_KEY
bash scripts/start_sim.sh down
bash scripts/start_sim.sh up
```

窗口就绪后，在同一工作区启动 VLM 探索。这会让仿真车运动。

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_sim.sh explore-vlm
```

### 冰箱目标搜索

沿用上面的 VLM 配置和密钥加载方式。检查 `runtime_logs/sim/vlm.yaml` 的 `target: refrigerator`。同名目标和单类别表在 `simulation.yaml`，对应 `class_id: 0`。这里还配置了本地权重路径、YOLO 0.25、Agent 语义分数大于 0.75、目标掩码内至少 10 个有效三维点。语义阈值为仿真初值，闭环效果待人工验证。[Agent 的官方逻辑与轮式适配](../conavgpt2_ws/README.md#单机器人目标搜索)

已有仿真镜像时用 `compile` 更新。已有会话时，在允许重启该会话后用 `reset` 重新生成配置和加载模块。若需要把新密钥传入会话，按上节使用 `down` 和 `up`。以下命令在未启动会话时运行。权重已准备好时可省略 `prepare-sam`。

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_sim.sh compile
bash scripts/start_sim.sh prepare-sam
bash scripts/start_sim.sh up
```

等 Gazebo 和 RViz 就绪后，手动启动任务。车辆会执行旋转扫描，后台识别按角度采样的观测。扫描 Action 完成后开始发布导航路径。探索时低频后台识别，导航继续用新观测更新。接受三维目标后切换至物体导航，满足 FMM 到达条件并收到停车应答时结束本轮。

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_sim.sh search-object
```

在另一个终端查看状态。用 Ctrl+C 退出查看器。

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_sim.sh status
bash scripts/start_sim.sh watch
```

本轮的原图、框、掩码、三维接受记录和变换在 `segmentation/`。用时间戳与 `decisions.jsonl` 对照。VLM 候选图和回复在 `vlm/`。最终 `report.json` 用 `task_success` 和 `stop_result` 记录任务结果。

```bash
cd ~/workspace/wheeltec_motion_ws
run_id="$(bash scripts/start_sim.sh status | jq -r .run_id)"
run_dir="runtime_logs/sim/agent/runs/$run_id"
jq '{exit_reason, task_outcome, target_reached, task_success, stop_result, last_detection}' "$run_dir/report.json"
jq -c 'select(.target_acquired or (.request_published and .pending_detection_stamp_ns != null)
  or .goal_kind == "object" or .reason == "EXITED") |
  {reason, source_stamp_ns, pending_detection_stamp_ns, goal_world, request_published, target_plan, stop_result}' "$run_dir/decisions.jsonl"
ls "$run_dir/segmentation"
```

人工观察 `watch` 显示后台识别时前沿路径是否继续更新，再对照冰箱框、掩码、有效深度和点云、`TARGET_ACQUIRED`、物体导航及 `TARGET_REACHED` 后的停车。冰箱框内深度无效且场景其他区域仍有有效深度时，检查 `NO_VALID_TARGET_DEPTH` 和无目标切换。整帧无有效深度报告 `NO_USABLE_DEPTH`。在另一终端可随时结束本轮。

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_sim.sh stop
```

## 仿真调试

先打开一个单独的终端看实时状态。RViz 的 **Exploration status and goals** 同步显示前沿和当前目标。`Ctrl+C` 只退出状态查看器，停车用 `stop`。

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_sim.sh watch
```

其他命令也在这个工作区执行。

| 要查什么 | 命令或位置 |
| --- | --- |
| 会话模式、run ID、导出结果 | `bash scripts/start_sim.sh status` |
| 启动和进程报错 | `bash scripts/start_sim.sh logs` |
| 图像卡顿、观测和定位时间 | `bash scripts/start_sim.sh diagnose --duration 120 --label rviz-on` |
| 轨迹、控制输出和规划路径 | `bash scripts/start_sim.sh record`，用 Ctrl+C 停止录包 |
| 从容器检查 ROS 话题 | `bash scripts/start_sim.sh shell` |
| API 域名和连接 | `bash scripts/start_sim.sh api-status`。此命令无鉴权，返回 401 表示服务有响应 |
| VLM 对话与选点 | 查看本轮 `vlm/<观测时间戳>.json` 和同时间戳的候选 JPEG。用 `vlm_selection.request_stamp_ns` 关联 `decisions.jsonl` 中的规划、发布目标及路径 |
| API 推理报错 | 查看本轮 `vlm/error.json`。实际推理返回 401 时核对密钥、地域和业务空间。请求头协议错误时检查密钥是否夹带空格或换行 |
| VLM 回复被截断 | `VLM_RESPONSE_TRUNCATED` 对应回复的 `finish_reason=length`。查看同轮 JSON 的回复原文与 `request.parameters.max_tokens`。百炼模板使用 256 token |

任务异常退出时，先用 `status` 找到 run ID，再查宿主机 `runtime_logs/sim/agent/runs/<run_id>/report.json` 的 `exit_reason`。探索报告另含 `motion_status_before_stop`，逐条决策在同目录的 `decisions.jsonl`。界面状态事件在 `runtime_logs/sim/status/<run_id>/events.jsonl`。`task.log` 会追加历史输出，结合当前 run ID 和时间判断。

### 导出地图

Agent 运行期间可手动请求导出。`status` 可查看后台导出结果。默认约每 15 秒保存一次，正常结束时还会保存最终阶段。

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_sim.sh export
bash scripts/start_sim.sh status
find runtime_logs/sim/agent/runs -name overview.png -print
```

每轮在 `runtime_logs/sim/agent/runs/<run_id>/stages/` 下单独保存。先打开 `overview.png` 查看几何图、彩色图和关键帧。需要原图或坐标信息时，再看对应的 `Kxxxx.png` 和 `metadata.json`。

### 仿真验证状态

用户确认住宅仿真探索初步可用。VLM 复跑 `d4ad9143` 持续约 5 分 22 秒，9 次回复均完整，模型目标均有路径发布和 Agent 到达记录，最终由用户停止。厨房斜侧观察点的冰箱分割已单独验证，闭环搜索运动效果待人工验证。

仿真图像曾因 Fast DDS 共享内存段过小而间歇丢帧。保留仿真专用配置的 **64 MiB** 共享内存段。修改该文件后，有会话时用 `reset`，否则用 `up` 加载。[fastdds.xml](scripts/sim/fastdds.xml)

场景、仿真速度和导出间隔都在仿真配置中。修改后执行 `bash scripts/start_sim.sh compile`，再用 `reset` 重启已有会话，或用 `up` 启动新会话。桌面重新登录后如果打不开窗口，用当前桌面终端执行 `down` 再 `up`，重新挂载图形会话。[simulation.yaml](src/conavbot_sim/config/simulation.yaml)

仿真 RGB 渲染到 30 m，覆盖住宅内可见表面。配对深度保留 6 m 上限，超范围像素以无效深度发布。静止分割使用完整 RGB，VLM 彩色地图使用 Agent 累积的有效 RGB-D 点。[相机配置](src/conavbot_sim/config/simulation.yaml)

仿真使用场景提供的定位。实车使用 D455 与 FAST-LIO，验证范围见[实机验证状态](#实机验证状态)。

开发约定和变更历史分别见 [AGENTS.md](AGENTS.md) 与 [CHANGELOG.md](CHANGELOG.md)。
