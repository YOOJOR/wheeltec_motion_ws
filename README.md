# CoNavBot 启动与调试

CoNavBot 是基于 Co-NavGPT2 的轮式机器人探索项目。这个工作区负责传感器和底盘启动、路径跟踪以及 Gazebo 仿真。相邻工作区的 Agent 根据 RGB-D 和定位建图、选择探索目标，并把路径交给运动控制器。[Agent 与 Jetson 环境](../conavgpt2_ws/README.md)

[桌面仿真](#桌面仿真) · [仿真调试](#仿真调试) · [实机启动](#实机启动) · [实机调试](#实机调试)

## 桌面仿真

将 `wheeltec_motion_ws` 和 `conavgpt2_ws` 放在同一个父目录。在装有 Podman 的桌面终端执行。首次运行先 `build`，之后可以直接 `up`。脚本会打开 Gazebo 和 RViz，仿真车默认静止。

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
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
| 搜索冰箱 | `bash scripts/start_sim.sh search-object`，使用 VLM 前沿探索，三维定位后导航，到达后停车 |
| 回到 RViz 点目标模式 | `bash scripts/start_sim.sh goal` |
| 停止当前任务 | `bash scripts/start_sim.sh stop` |
| 仿真急停 | `bash scripts/start_sim.sh estop`，清空指令队列，车体仍需物理制动 |
| 恢复初始场景 | `bash scripts/start_sim.sh reset`，回到 RViz 点目标模式 |
| 关闭仿真 | `bash scripts/start_sim.sh down` |

切换模式时会停止旧任务并保留当前位置。仿真持续探索需要手动停止，或在故障时退出。

### 静止冰箱分割

首次使用时构建镜像、显式准备模型，再启动仿真。已有容器如果仍引用旧镜像，先在本目录执行 `bash scripts/start_sim.sh down`。这会关闭现有仿真会话。

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
bash scripts/start_sim.sh build
bash scripts/start_sim.sh prepare-sam
bash scripts/start_sim.sh up
```

`prepare-sam` 下载 YOLO-World、MobileSAM 和 CLIP ViT-B/32 权重，并把来源和 SHA-256 写入 `runtime_logs/sim/models/manifest.json`。识别参数集中在 `simulation.yaml`。默认 CPU、单类别 `refrigerator`、置信度门槛 0.25、每帧最多三个框。缺少权重时识别会报错。

在 RViz 用 **2D Goal Pose** 将车开到已实测的厨房斜侧观察点 `(3.15, 1.35)`，拖动箭头朝向冰箱 `(4.8, 4.2)`，约 `1.05 rad`。等车停稳，RGB 中能同时看到冰箱正面和左侧后运行下面两条命令。

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
bash scripts/start_sim.sh preview-sam
bash scripts/start_sim.sh status
```

识别会退出当前导航任务，在原地逐帧处理完整的 640×480 图像。RViz 的 **Refrigerator segmentation** 显示类别、YOLO 检测分数、框和半透明掩码，可拖动面板下方的分隔线放大图像。进程结束后以本轮文件为准。`status` 显示 run ID、阶段和结果，宿主结果在 `runtime_logs/sim/agent/runs/<run_id>/segmentation/`，同级 `report.json` 记录模型加载和推理耗时。每帧以同一时间戳保存 RGB、叠加图、JSON 和命中时的二值掩码。空检测保存原图、标有 `no refrigerator detected` 的叠加图和空实例列表。

隔离仿真在斜侧观察点的三帧均检测到冰箱，YOLO 分数约 0.804，掩码覆盖可见柜体。CPU 冷加载约 5.94 秒，含 MobileSAM 的单帧推理约 1.42–2.08 秒。正面观察点 `(4.5, 1.05)` 的三帧均为空检测。两个门洞在静态地图中各留出 32 格，跨门洞行驶待人工观察。若 RGB 一直没有更新，先查看适配器是否已建立 `SIM_EPOCH_OFFSET_NS`。时钟未稳定时，适配器会等待，识别会报告采集超时。

用下面的命令中止识别并回到 RViz 点目标模式。

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
bash scripts/start_sim.sh stop
bash scripts/start_sim.sh goal
```

### VLM 探索

默认的 `explore` 使用几何前沿。`explore-vlm` 有多个合格前沿时会发送同一份 RGB-D 彩色俯视地图的多张 JPEG，每张标出 R0 和一个完整前沿。模型在 `robot_0` 字段返回编号，并提供 `reason`。只有一个候选时直接规划，无候选时沿用随机回退。选点前停车，回复后用新观测重规划。探索的物体检测关闭，静止分割仍用 `preview-sam`。[Agent 协议与上游差异](../conavgpt2_ws/README.md#vlm-前沿选择)

首次配置时复制模板，然后编辑 `runtime_logs/sim/vlm.yaml` 的 `base_url`、`model` 和 `target`。已有配置时直接编辑。住宅冰箱任务使用 `target: refrigerator`。百炼的 `base_url` 使用业务空间的 OpenAI 兼容地址，末尾为 `/compatible-mode/v1`。密钥环境变量使用模板中的 `CONAVBOT_VLM_API_KEY`。

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
mkdir -p runtime_logs/sim
cp -n ../conavgpt2_ws/development/Co-NavGPT2/conavbot/vlm.example.yaml runtime_logs/sim/vlm.yaml
```

用 Bash 执行下方命令。使用 fish 时先输入 `bash`。密钥在终端中输入，`up` 会将其传给新会话。已有会话需要先关闭再启动。

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
read -r -s -p 'VLM API key: ' CONAVBOT_VLM_API_KEY
printf '\n'
export CONAVBOT_VLM_API_KEY
bash scripts/start_sim.sh down
bash scripts/start_sim.sh up
```

窗口就绪后，在同一工作区启动 VLM 探索。这会让仿真车运动。

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
bash scripts/start_sim.sh explore-vlm
```

### 冰箱目标搜索

沿用上面的 VLM 配置和密钥加载方式。检查 `runtime_logs/sim/vlm.yaml` 的 `target: refrigerator`。同名目标和单类别表在 `simulation.yaml`，对应 `class_id: 0`。这里还配置了本地权重路径、YOLO 0.25、Agent 语义分数大于 0.75、目标掩码内至少 10 个有效三维点。语义阈值为仿真初值，闭环效果待人工验证。[Agent 的官方逻辑与轮式适配](../conavgpt2_ws/README.md#单机器人目标搜索)

已有仿真镜像时用 `compile` 更新。已有会话时，在允许重启该会话后用 `reset` 重新生成配置和加载模块。若需要把新密钥传入会话，按上节使用 `down` 和 `up`。以下命令在未启动会话时运行。权重已准备好时可省略 `prepare-sam`。

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
bash scripts/start_sim.sh compile
bash scripts/start_sim.sh prepare-sam
bash scripts/start_sim.sh up
```

等 Gazebo 和 RViz 就绪后，手动启动任务。车辆会执行旋转扫描，后台识别按角度采样的观测。扫描 Action 完成后开始发布导航路径。探索时低频后台识别，导航继续用新观测更新。接受三维目标后切换至物体导航，满足 FMM 到达条件并收到停车应答时结束本轮。

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
bash scripts/start_sim.sh search-object
```

在另一个终端查看状态。用 Ctrl+C 退出查看器。

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
bash scripts/start_sim.sh status
bash scripts/start_sim.sh watch
```

本轮的原图、框、掩码、三维接受记录和变换在 `segmentation/`。用时间戳与 `decisions.jsonl` 对照。VLM 候选图和回复在 `vlm/`。最终 `report.json` 用 `task_success` 和 `stop_result` 记录任务结果。

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
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
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
bash scripts/start_sim.sh stop
```

## 仿真调试

先打开一个单独的终端看实时状态。RViz 的 **Exploration status and goals** 同步显示前沿和当前目标。`Ctrl+C` 只退出状态查看器，停车用 `stop`。

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
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
| API 域名和连接 | `bash scripts/start_sim.sh api-status`。这是无鉴权探测，返回 401 表示服务有响应 |
| VLM 对话与选点 | 查看本轮 `vlm/<观测时间戳>.json` 和同时间戳的候选 JPEG。用 `vlm_selection.request_stamp_ns` 关联 `decisions.jsonl` 中的规划、发布目标及路径 |
| API 推理报错 | 查看本轮 `vlm/error.json`。实际推理返回 401 时核对密钥、地域和业务空间。请求头协议错误时检查密钥是否夹带空格或换行 |
| VLM 回复被截断 | `VLM_RESPONSE_TRUNCATED` 对应回复的 `finish_reason=length`。查看同轮 JSON 的回复原文与 `request.parameters.max_tokens`。百炼模板使用 256 token |

任务异常退出时，先用 `status` 找到 run ID，再查宿主机 `runtime_logs/sim/agent/runs/<run_id>/report.json` 的 `exit_reason`。探索报告另含 `motion_status_before_stop`，逐条决策在同目录的 `decisions.jsonl`。界面状态事件在 `runtime_logs/sim/status/<run_id>/events.jsonl`。`task.log` 会追加历史输出，结合当前 run ID 和时间判断。

### 导出地图

Agent 运行期间可手动请求导出。`status` 可查看后台导出结果。默认约每 15 秒保存一次，正常结束时还会保存最终阶段。

```bash
cd /home/ujiu/workspace/wheeltec-project/ws/wheeltec_motion_ws
bash scripts/start_sim.sh export
bash scripts/start_sim.sh status
find runtime_logs/sim/agent/runs -name overview.png -print
```

每轮在 `runtime_logs/sim/agent/runs/<run_id>/stages/` 下单独保存。先打开 `overview.png` 查看几何图、彩色图和关键帧。需要原图或坐标信息时，再看对应的 `Kxxxx.png` 和 `metadata.json`。

### 仿真调试结论与实机迁移参考

用户确认住宅仿真探索初步可用。VLM 复跑 `d4ad9143` 持续约 5 分 22 秒，9 次回复均完整，模型目标均有路径发布和 Agent 到达记录，最终由用户停止。厨房斜侧观察点的冰箱分割已单独验证，闭环搜索运动效果待人工验证。

仿真图像曾因 Fast DDS 共享内存段过小而间歇丢帧。保留仿真专用配置的 **64 MiB** 共享内存段。修改该文件后，有会话时用 `reset`，否则用 `up` 加载。[fastdds.xml](scripts/sim/fastdds.xml)

场景、仿真速度和导出间隔都在仿真配置中。修改后执行 `bash scripts/start_sim.sh compile`，再用 `reset` 重启已有会话，或用 `up` 启动新会话。桌面重新登录后如果打不开窗口，用当前桌面终端执行 `down` 再 `up`，重新挂载图形会话。[simulation.yaml](src/conavbot_sim/config/simulation.yaml)

仿真 RGB 渲染到 30 m，覆盖住宅内可见表面。配对深度保留 6 m 上限，超范围像素以无效深度发布。静止分割使用完整 RGB，VLM 彩色地图使用 Agent 累积的有效 RGB-D 点。[相机配置](src/conavbot_sim/config/simulation.yaml)

仿真使用场景提供的定位。迁移到实机时，先核对相机、FAST-LIO 和安装外参，再用只读建图检查观测是否配对、地图是否对齐。整车探索尚未完成实机验收。

## 实机启动

以下命令在小车的 Ubuntu 22.04 / ROS 2 Humble 桌面执行。准备 GNOME Terminal、厂家底盘通信工作区 `wheeltec_base_ws`、Livox 与 FAST-LIO 所在的 `vendor_ws`，以及相邻的 Agent 工作区。[基础栈部署配置](scripts/robot_stack/config.yaml) · [Jetson 环境准备](../conavgpt2_ws/README.md#jetson-环境准备)

首次部署或更新运动接口后构建实机包。重新部署雷达驱动时，核对实际加载的雷达地址与外参。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
bash scripts/build.sh --packages-select wheeltec_motion_interfaces wheeltec_motion_control
```

检查通过后，启动底盘、Livox、FAST-LIO、运动控制和 D455。下方命令同时打开 RViz。日常只启动基础栈时可省略 `--rviz`。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
bash scripts/start_robot.sh --check && bash scripts/start_robot.sh --rviz
```

等各组件就绪后，在另一个终端先启动 20 秒只读建图，观察 RGB、深度、定位和地图是否正常更新。Agent 由单独的命令启动。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
bash scripts/start_conav_exploration.sh --start --duration 20
```

### 有限运动与停车

确认外参和只读地图正常后，才在获得现场授权、有人观察且能随时停车的条件下运行以下示例。它会让实车运动，最多运行 45 秒，限制线速度、角速度、距起点位移和累计转角。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
bash scripts/start_conav_exploration.sh --start --motion --duration 45 \
  --forward-speed 0.05 --turn-speed 0.25 --max-distance 0.2 --max-rotation 8.0
```

在 Agent 终端按 Ctrl+C 可结束探索并请求停车。下方命令可在另一个终端单独请求停车。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
bash -c 'source /opt/ros/humble/setup.bash && source install/local_setup.bash && python3 -m wheeltec_motion_control.navigation_client stop'
```

确认车体实际停稳后，用 `bash scripts/start_robot.sh --stop` 关闭基础栈。在受管标签页按 Ctrl+C 或关闭标签页，也会结束整套基础栈。

## 实机调试

以下命令在 `/home/wheeltec/workspace/wheeltec_motion_ws` 执行。

| 现象 | 先查哪里 |
| --- | --- |
| 基础栈未就绪或反复重启 | `bash scripts/start_robot.sh --status`，然后看 `runtime_logs/robot_stack/` 中相应组件的日志 |
| RViz 显示太重 | `bash scripts/start_debug_rviz.sh --minimal`，只开地图调试布局，关闭图像和点云显示 |
| `NO_USABLE_DEPTH` 或 `NO_NEW_OBSERVATION` | 看 D455 的 RGB、对齐深度和内参，再检查图像与 `/Odometry` 的时间是否能配对 |
| `PROCESSING_EXPIRED` 或路径超时 | 看本轮报告中的观测年龄、Agent 计算耗时和运动状态，确认是否持续收到新路径 |
| 地图方向或位置不对 | 检查 D455 安装外参和底盘参考点配置。定位重启后重新启动 Agent 建图 |

实机 Agent 结果保存在 `runtime_logs/conav/latest/runs/<run_id>/`。诊断时先看 `report.json`，需要时再看 `decisions.jsonl`。基础栈启动后，可在另一个 Bash 终端用以下命令只读记录一分钟的传感器时间和运动状态。

```bash
cd /home/wheeltec/workspace/wheeltec_motion_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
../conavgpt2_ws/.venv/bin/python scripts/record_conav_acceptance.py \
  --duration 60 --sensor-timing \
  --output "runtime_logs/conav/acceptance/run-$(date +%Y%m%d-%H%M%S).json"
```

| 配置 | 用途 |
| --- | --- |
| [robot_stack/config.yaml](scripts/robot_stack/config.yaml) | 依赖工作区路径、启动命令和日志目录 |
| [motion_control.yaml](src/wheeltec_motion_control/config/motion_control.yaml) | 本车底盘外参、速度上限和时效参数。修改后重启控制节点 |
| [motion_control.example.yaml](src/wheeltec_motion_control/config/motion_control.example.yaml) | 新车从禁用运动输出的模板开始，重新核对外参 |
| [d455.yaml](config/d455.yaml) | 相机分辨率、频率和深度对齐 |
| [Agent config.yaml](../conavgpt2_ws/development/Co-NavGPT2/conavbot/config.yaml) | 相机安装外参与运动控制配置路径 |

开发约定和变更历史见 [AGENTS.md](AGENTS.md) 与 [CHANGELOG.md](CHANGELOG.md)。
