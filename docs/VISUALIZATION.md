# RViz 地图与探索调试入口

在 Orin 图形桌面的普通终端运行：

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_debug_rviz.sh
```

默认奥比，脚本自动加载 Humble 和项目布局，无须手工添加显示项，无须重新编译。它只打开 RViz，不启动底盘、雷达、相机、探索、导航或运动；关闭窗口或 Ctrl+C 仅退出可视化。启动前也可执行 `bash scripts/start_debug_rviz.sh --check`，只检查安装和布局，不打开图形窗口或设备。SSH 无图形会话时，请改在 Orin 桌面运行。

## 真实数据的启动顺序

四个终端分别运行，已有正常节点时复用，不重复启动：

```bash
# 终端一：通信 / 雷达 / FAST-LIO / 控制，等待 STACK READY
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_robot.sh
```

```bash
# 终端二：奥比相机及适配器
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_astra_rgbd.sh --start
```

```bash
# 终端三：只读地图 / 随机前沿 / FMM
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_exploration_preview.sh --start --skip-initial-scan
```

```bash
# 终端四：可视化
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_debug_rviz.sh
```

上述启动不提交 Action、不启用导航。控制节点空闲时发送零速度，探索和 RViz 没有速度出口。ROS_DOMAIN_ID 与其他网络/RMW 设置沿用当前终端，四个终端须一致；不要把隔离测试域与实车运行域混用。

## 画面怎么看

布局文件为 `config/rviz/exploration_debug.rviz`。Fixed Frame 固定为 `camera_init`，默认俯视、10 FPS；鼠标可平移/缩放。显示项可在左侧勾选，File → Save Config As 可将个人布局另存到 runtime_logs，不覆盖受版本管理的默认布局。

| 显示项 | 话题及意义 |
| --- | --- |
| Exploration map | `/wheeltec_exploration/map`：白色已观测非障碍、黑色障碍、灰色未知；5 cm 栅格，地图约 1 Hz |
| Frontiers goals and scan status | `/wheeltec_exploration/debug_markers`：青色边缘、黄色候选、绿色目标、橙色短期目标和扫描/状态文字 |
| Explored points layer / Obstacle mask | 默认关闭，分别查看点投影层和二值障碍层；障碍掩码的白色只表示未标记障碍 |
| FAST-LIO IMU pose | `/Odometry`：蓝色箭头为 IMU/body 位姿，不是地面 wheeltec_base，也不是相机原点 |
| FAST-LIO IMU path | `/path`：定位轨迹，缺少该话题时不影响探索地图显示 |
| Lidar registered cloud | `/cloud_registered`：默认关闭，可手动勾选对比雷达几何；不是 RGB-D 探索地图 |
| Camera RGB / depth | 两个原始图像窗口；奥比深度显示范围固定为 400–2000 mm，超出显示范围不代表有效测量 |

默认目标改为带 0.5 s 寿命的球形标记，每轮清除旧标记，故障/reset/退出时清除。旧 Pose 话题仍发布，但布局默认关闭其箭头显示；手动开启时，箭头只表示位置且可能遗留。是否有效以最新决策、epoch 和源时间为准。

上述静止教程显式跳过启动扫描。去掉 `--skip-initial-scan` 时会显示 INITIAL_SCAN 和转角进度，但只读入口不会转动车体，静止保持至 45 s 会触发扫描超时。跳过扫描会在诊断中明确标记，不代表扫描验收通过。

RGB/深度显示为驱动原始图像；算法使用自研 `/wheeltec_camera/rgbd` 的去畸变、米制、有效范围过滤观测。原始图像窗口不需要车体相机 TF，因此这里只用 Image 显示，不用要求世界相机变换的 Camera/DepthCloud。入口不生成任何假定 TF；缺少 wheeltec_base 或世界相机 TF 不能通过改帧名解决。

地图订阅为 Reliable + Volatile，与探索发布者匹配；图像、定位和点云使用 Best Effort，兼容传感器 QoS。只保留较小订阅缓存；点云默认关闭，以减少可视化负载。

## 决策和故障诊断

RViz 原生布局不显示 JSON 决策文本。在另一个终端查看：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/wheeltec_motion_ws/install/local_setup.bash
ros2 topic echo /wheeltec_exploration/decision
```

`processed` 持续增加表示新观测被处理；`INITIAL_SCAN` 表示旋转观察建议，`PLANNED` 表示目标规划建议，均不驱车。`goal_source` 区分 random_frontier 和 random_map_fallback；默认策略允许灰色未知参与规划，颜色不代表已证实可通行。`NO_VALID_RANDOM_GOAL` 表示有限采样未找到可用目标。策略细节见 [EXPLORATION_POLICY](EXPLORATION_POLICY.md)。

画面没有数据，先展开对应显示的 Status：

- 图像空白：确认相机终端运行、topic 名称正确，查看 `/wheeltec_camera/status`。原始深度缺失时先检查有效视野，不调整规划速度。
- 没有地图：确认探索运行、`processed` 增加，检查相机和 `/Odometry` 是否新鲜、能否时间配对。
- 目标不更新：检查最新 reason；扫描阶段没有目标是正常行为；探索阶段无前沿会进入全图随机兜底。
- 定位轨迹变化但地图偏移/重影：检查时间配对、外参、FAST-LIO 是否重启，不能把 IMU 点误作底盘点。
- 整体无话题：核对 ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY / RMW 设置是否与生产节点一致。
- RViz 没有窗口：在图形桌面运行；虚拟显示启动验证不能替代实际桌面检查。

输入异常原因查清并恢复后，可显式清图重新开始：

```bash
ros2 service call /wheeltec_exploration/reset std_srvs/srv/Trigger '{}'
```

FAST-LIO 重启改变原点后也需要 reset。服务回复只是排队，等待新 epoch 和新输入。结束测试时先调用运动 stop，再退出探索、相机和基础栈；关闭 RViz 本身不会关闭这些服务。停止命令和基础栈操作见 OPERATIONS。

## D455 独立图像选项

```bash
bash scripts/start_debug_rviz.sh --camera d455
```

仅将两个图像窗口切换到独立 D455 raw 话题，深度显示范围改为 400–6000 mm；生成配置在 `runtime_logs/visualization/d455_debug.rviz`。它不启动 D455、不替换外参，也不让缺少安装外参的 D455 获得世界建图能力。当前 baseline 仍使用奥比。

本入口可辅助核验真实几何和观察规划，不能证明自主探索、外参精度、避障或物理停车已通过。当前软件/实车验证结果见 VALIDATION。
