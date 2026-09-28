# WHEELTEC 闭环运动控制 · v0.3.0

ROS 2 Humble 下的相对直行和原地转向接口：FAST-LIO 位姿 → 底盘参考点换算 → `/cmd_vel` → 原厂串口通信。适用于当前 `senior_mec_bs` 麦克纳姆底盘，不修改 STM32 固件。2026-09-28 已将 FAST-LIO 迁入 vendor_ws，当前 main 使用新布局，v0.3.0 标签保留迁移前布局。

当前 main 的第一台车配置已应用响应参数试调，实车效果待用户确认；对比与回退见 [操作手册](docs/OPERATIONS.md)。

**后续开发先读 [阶段总结](docs/STAGE_SUMMARY.md)**。它记录当前状态、接口、实车参数、已验证范围、已知问题和下一步；无需从历史对话或原始日志恢复上下文。

| 文件 | 用途 |
| --- | --- |
| [docs/STAGE_SUMMARY.md](docs/STAGE_SUMMARY.md) | 后续工作的统一上下文入口 |
| [docs/OPERATIONS.md](docs/OPERATIONS.md) | 日常启动、动作调用、参数调整与故障排查 |
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

脚本仅构建本仓库两个包，使用独立 ROS 域和合成话题进行测试，不启动厂家串口或传感器。输出位于忽略 Git 的 `runtime_logs/validation/`。外部依赖安装见复刻说明。

**`config/motion_control.yaml` 是第一台车的实车配置，输出已启用。** 第二台车先用 `motion_control.example.yaml`（禁用输出、外参占位），完成本车测量后再启用；不要把第一台的偏移当成通用值。

## 仓库范围

- `src/wheeltec_motion_interfaces`：ExecuteMotion Action 定义。
- `src/wheeltec_motion_control`：控制器、CLI、离线标定和测试。
- `scripts`：构建检查、两种启动管理方式、被动延迟诊断。
- `docs`：当前维护说明；旧逐次开发日志和实验数据已归档到仓库外。

仅本仓库上传 GitHub。厂家底盘和第三方传感器/定位源码不纳入；日常读阶段总结与当前手册，旧记录只在用户明确要求排查历史问题时查阅。
