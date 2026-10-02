# D455 调试工具与独立相机接口

2026-10-02 已在 Orin 检测到 D455（8086:0b5c，USB 5 Gbps），设备固件 5.15.1.55。已有 librealsense SDK 2.58.4、ROS 驱动 4.58.4、C/C++ 头文件与动态库、RealSense Viewer、深度质量/录制/转换工具；检查依赖完整，Viewer 已成功打开图形窗口。无需重复安装 SDK 或替换 Jetson 内核驱动。

**当前 baseline 默认继续使用奥比 ASTRA S，跑通后再切换 D455。** 两种相机的配置、原始输入、观测输出、状态话题和光学坐标系分开，上层通过相机选项选择；没有自动选设备或混合数据。

## 给人类使用的调试工具

桌面已添加 **RealSense D455 调试** 快捷方式，双击打开 RealSense Viewer。命令行等价入口：

```bash
bash ~/workspace/wheeltec_motion_ws/scripts/d455_tools.sh --viewer
# 列出设备/固件：
bash ~/workspace/wheeltec_motion_ws/scripts/d455_tools.sh --enumerate
# 打开深度质量工具：
bash ~/workspace/wheeltec_motion_ws/scripts/d455_tools.sh --quality
# 只检查工具和 SDK 是否已安装：
bash ~/workspace/wheeltec_motion_ws/scripts/d455_tools.sh --check
```

Viewer 中选择 D455，开启 Stereo Module 深度和 RGB Camera，可查看彩色、深度和点云，以及录制数据。使用 Viewer 或深度质量工具前退出 D455 ROS 入口，避免两个进程争用同一设备。查看和录制阶段不用执行固件升级或设备校准。应用测试日志在 runtime_logs/d455-tools。

CLI 工具实际路径为 /opt/ros/humble/bin/rs-record、rs-convert、rs-enumerate-devices 等；SDK 头文件在 /opt/ros/humble/include/librealsense2，库在 /opt/ros/humble/lib/aarch64-linux-gnu。本代码通过 ROS 驱动读流，不依赖 pyrealsense2 Python 绑定。

## 两种相机的代码选择

| 项目 | 奥比 ASTRA S（默认） | D455（后续替换测试） |
| --- | --- | --- |
| 选择值 | astra_s | d455 |
| 启动脚本 | scripts/start_astra_rgbd.sh | scripts/start_d455_rgbd.sh |
| RGBDFrame 输出 | /wheeltec_camera/rgbd | /wheeltec_camera/d455/rgbd |
| 状态话题 | /wheeltec_camera/status | /wheeltec_camera/d455/status |
| 彩色光学帧 | wheeltec_camera_color_optical_frame | wheeltec_d455_color_optical_frame |
| 应用深度掩码 | 0.4–2.0 m | 0.4–6.0 m |
| 内参来源 | 现有 YAML，设备归属待核验 | RealSense 驱动设备 CameraInfo |

两者消息类型和消费 API 相同，但数据源完全分开；同一上层任务只选择一台相机。切换相机时重建 reader、重新确认该相机外参，并清除旧相机建立的地图/任务。不直接运行上游带 Go2 控制副作用的 ros_single_nav.py。

```python
from wheeltec_rgbd.client import RGBDReader, NoFreshFrame
# 在 rclpy.init() 后创建，将 reader 加入正在 spin 的 executor。
camera = 'astra_s'  # baseline 默认；之后改成 'd455'
reader = RGBDReader(camera=camera)
last_stamp = 0
try:
    obs = reader.read_latest(after_stamp_ns=last_stamp)
    last_stamp = obs.stamp_ns
    rgb, depth_m, cam_K = obs.rgb, obs.depth_m, obs.cam_K
    # build_full_scene_pcd(depth_m, rgb, cam_K) 直接使用米制深度。
    # 原版 Agent 内部 /1000 的预处理才使用 obs.as_legacy_conav_observation()。
except NoFreshFrame:
    pass  # 等下一组；若已在导航中，上层按导航契约调用 stop。
```

PosedRGBDReader 同样接受 `camera='astra_s'` 或 `camera='d455'`，默认奥比；必须分别提供已确认的 body_from_base 与 base_from_camera。选择入口只完成观测源切换，尚未完成上游整个探索 Agent 的控制出口和建图/规划适配。

## D455 ROS 启动和采集

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_d455_rgbd.sh --check
bash scripts/start_d455_rgbd.sh --start
# 多台 RealSense 时显式选设备：
# bash scripts/start_d455_rgbd.sh --start serial_number:=设备序列号
```

默认请求彩色 RGB8 与深度 Z16，均 640×480 / 15 Hz，打开 enable_sync、align_depth.enable；IMU、红外、点云关闭。只启动 D455 驱动和自己的适配器，Ctrl+C 关闭。奥比原启动命令保持可用；D455 内部 TF 使用独立帧名。

另一个终端读取 D455 并保存快照（不打开相机设备）：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/wheeltec_motion_ws/install/local_setup.bash
ros2 run wheeltec_rgbd rgbd_capture --camera d455 --duration 10 \
  --output ~/workspace/wheeltec_motion_ws/runtime_logs/camera/d455-latest
# 奥比读取仍为默认：
# ros2 run wheeltec_rgbd rgbd_capture --camera astra_s --duration 10
```

--camera 自动选择对应话题、frame 和深度范围。修改 adapter_config 时，可用 `--settings /绝对路径.yaml` 同步消费者的图像/深度/时序设置；Settings 默认仍对应 Astra，不要给自定义话题套错默认参数。D455 的 0.4–6.0 m 是首版应用掩码，不代表所有距离/场景同等精度或避障能力。缺失/越界值变 NaN，32FC1 输出单位为米。

D455 原始输入前缀为 `/wheeltec_camera/d455/raw/wheeltec_d455`：color/image_raw、color/camera_info、aligned_depth_to_color/image_raw、aligned_depth_to_color/camera_info。自研适配器只消费已对齐深度，原生未对齐 depth/image_rect_raw 或错误帧/内参仍被拒绝。配置在 src/wheeltec_rgbd/config/d455.yaml，启动在 src/wheeltec_rgbd/launch/d455_rgbd.launch.py；标定 URL 显式为空，不加载 Astra YAML。

## 外参与时间边界

世界变换仍为 `T_world_body × T_body_base × T_base_D455_color_optical`。D455 的安装位置和朝向须单独确认；Astra 厂家安装外参、D455 内部 depth→color 外参都不能代替 D455 相对底盘的安装外参。驱动仅发布设备内部 TF，没有自动建立 body/wheeltec_base→D455 安装链。

保留 RealSense ROS 驱动时间戳，来源标为 realsense_driver_ros_timestamp。RGB/深度同戳和 enable_sync 只验证软件契约，真实曝光与 FAST-LIO 扫描时间误差仍未测量。源/接收超时仍为 0.5 s，允许超前 0.1 s，RGB-D 配对门槛 0.03 s。长耗时感知/VLM 应与 ROS 接收分开；进入阻塞 VLM 前明确 stop，禁止用旧观察/决策换时间戳重发维持运动。

当前已完成 D455 调试工具准备和独立数据接口，主线仍优先用奥比跑 baseline。D455 安装外参、真实世界建图及完整自主探索尚未验收。
