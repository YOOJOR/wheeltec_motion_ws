# 四标签页版本（新增入口）

在小车图形桌面的终端运行，不需要先 source：

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_robot_tabs.sh
```

打开一个新的 GNOME Terminal 窗口，四个标签分别为 **底盘通信、Livox 雷达、FAST-LIO 定位、自动控制接口**。每页实际运行对应的 ros2 launch，实时显示原始输出并保存日志。进程退出或持续数据异常时，在原标签页中退出旧进程并重新启动，不另开新页。启动命令结束并返回提示符后，四个标签页仍继续运行。

后台总控沿用原 supervisor 的启动顺序、就绪检查及故障恢复规则；通信 → 雷达 → 定位 → 控制接口，依赖的数据就绪后才启动下一步。控制接口页显示 **就绪，可以接收新的控制指令。** 后，再在另外一个终端发送 motion_client 指令。脚本不会自动发送移动目标，也不续跑中断的动作。详细门槛与恢复规则见 [原版说明](README.md)。

## 查看与停止

```bash
bash scripts/start_robot_tabs.sh --status
bash scripts/start_robot_tabs.sh --stop
```

- `--status` 显示各组件运行状态、就绪提示及日志目录。
- **在任一受管标签页按 Ctrl+C 或关闭该标签页，停止整套服务，不再自动重启。** 这是主动停止，区别于 ROS 节点意外退出后自动恢复。输出日志可在停止后查看；GNOME Terminal 通常在运行命令结束时关闭标签。
- 关闭启动入口所在的原终端不停止这四个标签页；要停止应在受管标签页按 Ctrl+C，或运行 `--stop`。
- 所有受管标签页只显示输出，不提供额外的 shell 命令提示符；调试与发控制指令请另开普通标签页。
- 总控进程丢失/心跳超过 15 s 时，标签页退出自己的进程，不独立继续重启。这是管理进程失联处理时间，不是物理停车时间。

仅检查配置，不启动：

```bash
bash scripts/start_robot_tabs.sh --check
```

## 与旧版本对比

原来的 `start_robot.sh` 保持原样。两版共享同一个单实例锁，不能同时运行。

如果旧版本在前台运行，先在它的终端按 Ctrl+C，等四个节点退出，再运行新版。如果旧版本在后台，先核对 PID 的命令：

```bash
stack_pid=$(cat ~/workspace/wheeltec_motion_ws/runtime_logs/robot_stack/supervisor.lock)
ps -p "$stack_pid" -o pid,args
# 确认是 robot_stack/supervisor.py 后：
kill -INT "$stack_pid"
# 等原节点退出后：
bash ~/workspace/wheeltec_motion_ws/scripts/start_robot_tabs.sh
```

切回原版：

```bash
bash ~/workspace/wheeltec_motion_ws/scripts/start_robot_tabs.sh --stop
bash ~/workspace/wheeltec_motion_ws/scripts/start_robot.sh
```

新版只管理自身创建的标签页和进程，不接管已有手动节点，也不杀死已有的另一份总控。重复运行新版会提示已运行，使用 `--status` 查看即可。普通 SSH 没有图形会话时不能直接打开标签；应在小车桌面执行入口。SSH 中仍可使用 `--status` / `--stop`。

## 配置与日志

共用 [config.yaml](config.yaml) 的工作空间、四条启动命令、启动/健康门槛、重试间隔。默认不启动 RViz。修改后停止并重新启动，新脚本无需 colcon build。

每次运行生成 `~/workspace/wheeltec_motion_ws/runtime_logs/robot_stack/日期时间-tabs-PID/`：

- `base.log` / `livox.log` / `fastlio.log` / `motion.log`：各页 ROS 原始输出，重启后追加。
- `*-tab.log`：该页的启动次数、停止、就绪提示。
- `supervisor.log`：整体启动顺序与 `UNHEALTHY` / `RECOVER` 的具体原因。
- `bootstrap.log`：打开终端失败或总控异常。
- `ros/`：ROS 日志；两个配置 YAML 是本次运行快照。
- JSON 文件用于标签页与总控通信，请勿在运行中修改/删除。

控制接口一直没启动时，先看 FAST-LIO 页，再看 supervisor.log 的源时间戳/就绪原因。定位一直过旧仍会拒绝开放接口，不能只用“进程还活着”判断定位可用。
