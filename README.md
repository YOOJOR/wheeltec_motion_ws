> 当前主探索入口：`bash scripts/start_conav_exploration.sh`，直接运行原版 Co-NavGPT2 Agent，目标识别/VLM暂时关闭。默认只读；使用、可视化和有限移动验收见 [SOURCE_AGENT](docs/SOURCE_AGENT.md)。此前 `wheeltec_exploration` 是测试对照。

最新进一步测试发现运动时图像迟到会触发保护停车，持续探索尚未通过；服务等待与手动停止模块入口已修复。下一步先排查图像/配对时序，见阶段总结最新结论。

2026-10-13进一步定位：本轮原始图像持续，转向后所有原始正深度读数落在0.4–2m接受范围之外，RGB-D因NO_USABLE_DEPTH停止输出并触发停车。下一步验证深度范围及无近处点的处理；详细边界见阶段总结，尚未擅自放宽范围。

2026-10-12 已完成奥比相机 + FAST-LIO + 原版Agent + 持续速度接口的有限实车运动验收：一圈扫描后进入FMM转向/前进，达到预算后停车，用户确认停稳无异常。完整房间探索、避障及语义任务仍待验收；最新结论见阶段总结。

# WHEELTEC 闭环运动控制 · v0.3.0

ROS 2 Humble 下的相对直行和原地转向接口：FAST-LIO 位姿 → 底盘参考点换算 → `/cmd_vel` → 原厂串口通信。适用于当前 `senior_mec_bs` 麦克纳姆底盘，不修改 STM32 固件。2026-09-28 已将 FAST-LIO 迁入 vendor_ws，当前 main 使用新布局，v0.3.0 标签保留迁移前布局。

当前 main 的第一台车配置已应用响应参数试调，实车效果待用户确认；对比与回退见 [操作手册](docs/OPERATIONS.md)。

2026-10-01 当前 main 新增持续速度导航模式，用于 Co-NavGPT2 周期性更新底盘速度；Action 与导航互斥，停止/失效后不重放旧请求。接口、时间戳与最小接入示例见 [导航说明](docs/NAVIGATION.md)。本次仅完成速度接口和隔离验证，完整自主探索待后续接入。

2026-10-01 新增独立 RGB-D 接口：相机专用启动、已有 YAML 标定加载、米制深度与最新观测读取；真实静止取流已通过，文件的设备归属与车体外参待核验。见 [相机接口](docs/CAMERA_INTERFACE.md)。

**后续开发先读 [阶段总结](docs/STAGE_SUMMARY.md)**。它记录当前状态、接口、实车参数、已验证范围、已知问题和下一步；无需从历史对话或原始日志恢复上下文。

2026-10-02 新增 [单车探索预览文档](docs/EXPLORATION_PREVIEW.md)：RGB-D / FAST-LIO 配对、世界 XY 地图、随机可达前沿、官方 FMM 和方向建议已完成合成及隔离 ROS 验证。入口只发布地图和诊断，没有运动输出、不调用 VLM。默认奥比，D455 保留独立接口；厂家暂定相机外参用于只读预览，尚未物理确认。

| 文件 | 用途 |
| --- | --- |
| [docs/STAGE_SUMMARY.md](docs/STAGE_SUMMARY.md) | 后续工作的统一上下文入口 |
| [docs/OPERATIONS.md](docs/OPERATIONS.md) | 日常启动、动作调用、参数调整与故障排查 |
| [docs/VISUALIZATION.md](docs/VISUALIZATION.md) | RViz 预设启动、探索地图/图像/定位显示与调试 |
| [docs/EXPLORATION_POLICY.md](docs/EXPLORATION_POLICY.md) | 论文地图/前沿、启动扫描、随机目标兜底及与原版的对应 |
| [docs/CAMERA_INTERFACE.md](docs/CAMERA_INTERFACE.md) | 相机单独启动、RGB-D 契约和新鲜观测读取 |
| [docs/NAVIGATION.md](docs/NAVIGATION.md) | 持续速度导航契约、Co-NavGPT2 适配与失效停止 |
| [docs/SOURCE_AGENT.md](docs/SOURCE_AGENT.md) | 当前原版Agent的运行、可视化、停止与有限实车验收教程 |
| [docs/EXPLORATION_PREVIEW.md](docs/EXPLORATION_PREVIEW.md) | 旧自研测试预览的使用与验证边界 |
| [docs/WORKSPACE_SETUP.md](docs/WORKSPACE_SETUP.md) | 第二台车从源码到编译的复刻步骤；部署为 `~/workspace/README.md` |
| [docs/CALIBRATION.md](docs/CALIBRATION.md) | 离线旋转录包标定工具 |
| [docs/VALIDATION.md](docs/VALIDATION.md) | 当前版本验证摘要 |
| [CHANGELOG.md](CHANGELOG.md) | 简洁版本变更 |

## 在第一台小车运行

三个工作空间已编译时，在小车图形桌面的终端执行：

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_robot_tabs.sh
```

四个标签分别运行通信、Livox、FAST-LIO、控制接口。看到控制接口页“就绪，可以接收新的控制指令”后，另开普通终端：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/wheeltec_motion_ws/install/local_setup.bash
ros2 run wheeltec_motion_control motion_client move 0.2 --max-speed 0.05
ros2 run wheeltec_motion_control motion_client rotate 1.570796 --max-speed 0.1
ros2 run wheeltec_motion_control motion_client stop
```

上面是三个独立操作示例，不是自动任务序列。距离单位 m，转角单位 rad；正数前进/左转，负数后退/右转。

```bash
bash scripts/start_robot_tabs.sh --status
bash scripts/start_robot_tabs.sh --stop
```

任一受管标签 Ctrl+C / 关闭该页会停止整套服务；异常进程退出会在原页内自动恢复。原前台版本为 `bash scripts/start_robot.sh`。两版不能同时运行，重启不续跑旧动作。

## 编译与检查

Ubuntu 22.04 / ROS 2 Humble：

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/build_and_test.sh
```

脚本仅构建本仓库自研包，使用独立 ROS 域和合成话题进行测试，不启动厂家串口或传感器。输出位于忽略 Git 的 `runtime_logs/validation/`。外部依赖安装见复刻说明。

**`config/motion_control.yaml` 是第一台车的实车配置，输出已启用。** 第二台车先用 `motion_control.example.yaml`（禁用输出、外参占位），完成本车测量后再启用；不要把第一台的偏移当成通用值。

## 仓库范围

- `src/wheeltec_motion_interfaces`：ExecuteMotion Action、MotionStatus 定义。
- `src/wheeltec_motion_control`：控制器、CLI、离线标定和测试。
- `src/wheeltec_rgbd*`：两种相机的独立观测、状态和位姿配对接口。
- `src/wheeltec_exploration`：只读地图/随机前沿/FMM 预览；依赖已部署 conavgpt2_ws 环境。
- `scripts`：构建检查、两种启动管理方式、被动延迟诊断。
- `docs`：当前维护说明；旧逐次开发日志和实验数据已归档到仓库外。

仅本仓库上传 GitHub。厂家底盘和第三方传感器/定位源码不纳入；日常读阶段总结与当前手册，旧记录只在用户明确要求排查历史问题时查阅。

D455 现已接入统一 RGB-D 接口，启动、取流验证和 Python 使用见 [D455_INTERFACE](docs/D455_INTERFACE.md)。
