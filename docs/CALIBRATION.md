# 用原地旋转录包辅助估计安装平移

本工具完全离线，不需要启动控制节点，不发布速度、不修改实车配置。它估计 IMU 到实际有效旋转中心的偏移，并非自动确认底盘模型原点。只能在平整地面、近似固定旋转中心、定位稳定的条件下使用；低残差不等同于精确标定真值。

## 1. 采集

保持原有 Livox 和 FAST-LIO 正常工作，不启动新增闭环控制器。另开终端：

```bash
ros2 bag record -o imu_rotation_calibration /Odometry
```

使用现有键盘低速原地左转一整圈，停止，再右转一整圈。尽量连续匀速，前后/横移指令为零，保证雷达观测环境稳定；结束录包按 Ctrl-C。不用先知道外参。

## 2. 编译并安装依赖

在小车的 Ubuntu/Humble 环境：

```bash
sudo apt install ros-humble-rosbag2-py python3-numpy python3-matplotlib
cd ~/workspace/wheeltec_motion_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
```

## 3. 分析

先不填写未知高度：

```bash
ros2 run wheeltec_motion_control calibrate_rotation \
  --bag /录包的绝对路径/imu_rotation_calibration \
  --output ~/rotation_result_01
```

输出目录必须不存在，避免覆盖旧实验。`--bag` 指整个录包目录，包含 metadata.yaml 和所有分片。默认 sqlite3；其他存储插件通过 `--storage-id` 指定（需要相应 ROS 插件，当前只对 sqlite3 做端到端验证）。不用 ros2 bag play，也不需要与在线 ROS 图通信。

已测得“底盘参考点在 IMU 坐标中的 Z 分量”后可以提供它，例如 **-0.42 仅为示例**：

```bash
ros2 run wheeltec_motion_control calibrate_rotation \
  --bag /录包的绝对路径/imu_rotation_calibration \
  --body-z -0.42 --output ~/rotation_result_02
```

body_z 不是直接测得的竖直高度的通用替代：IMU 与底盘轴一致时，IMU 在底盘点上方 0.42 m 对应 -0.42；倾斜安装时需要换算到 IMU 轴下的 Z 分量。

## 4. 输出如何看

- `report.json`：完整机器可读报告，含使用参数、数据摘要、每段转角/时段/估计、条件数、残差及质量拒绝原因。
- `report.md`：文字摘要。
- `trajectory.png`：IMU 与换算后的旋转中心轨迹，以及逐段中心残差曲线。
- `poses.csv`：完整导出的选定时间范围位姿，可重复分析；列为 stamp,x,y,z,qx,qy,qz,qw。
- `translation_suggestion.yaml`：仅在检查通过且提供 body_z 时生成。是待审阅的合并片段，不能覆盖完整控制配置；保持 `extrinsics_calibrated: false` 和 `control_enabled: false`。

`translation_conditioned_on_body_z` 的单位是米，表达在 IMU/body 坐标下。未给 body_z 时第三项为求解约束零，**不是测出来的高度**；X/Y 也以该约束为条件。IMU Z 轴明显倾斜且未提供高度时会拒绝质量检查。默认容许 2° 小倾斜并不消除高度耦合：高度误差乘以倾角的量级会影响水平估计。

质量通过只表示录包与固定旋转中心模型一致，仍需核对机械测量、底盘参考点和轴向。工具不求 `body_from_base_rpy`，也不会自动确认外参。即使左右结果一致，也不能排除同一种系统性打滑偏差。

每段单独拟合中心，因此中间暂停或移动到另一位置不会被强迫共享一个中心，但旋转段内部的移动会表现为残差，可能导致拒绝。单方向数据可以计算，但缺少反向交叉检查，质量结果不通过。

## 5. 可调参数

执行 `ros2 run wheeltec_motion_control calibrate_rotation --help` 查看所有参数。常用项：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| --min-samples | 30 | 每段最少样本数 |
| --min-angle-deg | 180 | 每段最小连续转角，建议实际采一整圈 |
| --min-rate | 0.03 rad/s | 区分转动与停顿的航向速度阈值 |
| --max-gap | 0.5 s | 采样间隙超过该值则分段 |
| --max-jump | 0.3 m | 相邻位置突变的拒绝阈值 |
| --max-rms | 0.03 m | 中心残差 RMS 上限 |
| --max-p95 | 0.06 m | 中心残差 P95 上限 |
| --max-direction-difference | 0.03 m | 左右转/重复段估计一致性阈值 |
| --start-sec / --end-sec | 全部 | 相对录包首条目标话题源时间戳的裁剪范围 |
| --no-plot | 关闭 | 不绘图，可不安装 matplotlib |

阈值是工程初值，不是已验证的实车精度。噪声可能使连续转动被切碎，优先检查录包；不要只通过放宽阈值强行获得“通过”。原地转动主要提供两个可观测平移分量；若 IMU 朝向令固定 body Z 后的水平解仍不适定，工具会拒绝求解。

退出码：0=质量检查通过（仍不代表 Z 已知或可以直接试车）；2=得到报告但质量检查未通过；1=输入或依赖错误，无法完成分析。

## 6. 复用 CSV

```bash
ros2 run wheeltec_motion_control calibrate_rotation \
  --csv ~/rotation_result_01/poses.csv --output ~/rotation_result_03
```

CSV 默认假设包含与指定流程一致的世界/IMU 位姿，无法检查 frame 名；录包输入会严格检查 frame。文件夹与录包属于实验数据，不要上传大型 rosbag；保留报告、所用代码提交号和实测高度以便追溯。
