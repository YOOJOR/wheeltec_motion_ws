# 当前验证摘要

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
