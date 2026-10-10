# 自研 ASTRA S RGB-D 接口

D455 已新增独立启动和配置，继续使用同一 RGBDFrame/Reader 接口，见 [D455_INTERFACE](D455_INTERFACE.md)。奥比仍为 baseline 默认；D455 使用独立话题和帧，上层通过 camera 选项选择。

首版 2026-10-01。复用本机已安装的 Astra/OpenNI 底层驱动及现有标定文件，使用自研 wheeltec_rgbd 做配对、校验、深度单位/范围转换及最新观测读取。只启动相机，不启动厂家整车、EKF、跟随、SLAM、Nav2，不发布 /cmd_vel。厂家的完整功能包只作参考。

## 启动与停止

本仓库新增 wheeltec_rgbd_interfaces、wheeltec_rgbd；编译和隔离测试仍用 `bash scripts/build_and_test.sh`。

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_astra_rgbd.sh --check
bash scripts/start_astra_rgbd.sh --start
```

--check 是默认选项，不打开设备；--start 在前台运行，Ctrl+C 停止本次相机驱动和适配节点。不要同时运行另一个 Astra/OpenNI 进程。脚本仅加载系统 Humble、厂家 astra_camera_msgs/astra_camera 两个设备包以及自研空间；通过 WHEELTEC_ASTRA_INSTALL 可指定另一安装根。它不加载整个厂家 overlay，也不改厂家源码或 YAML。第二台车仍须单独准备兼容的底层驱动、核对设备型号和标定。

本机实际检查发现：SDK 的两种标定查询格式、取流前后均返回 NaN 内参；未加载文件时深度 CameraInfo 含 NaN，彩色则回退到视场角估算。接口拒绝该数据，不使用估算内参冒充设备标定。源码确认 getDepthCameraInfo 会在 SDK 参数无效时覆盖深度 K，因此仅给驱动传标定 URL 仍不能修复；适配层独立加载指定文件并保留真实输入 frame/时间戳与尺寸检查，raw 话题不改写。

启动入口和自研适配层默认显式加载本机已有 ~/.ros/camera_info/rgb_camera.yaml 和 ir_camera.yaml（文件存在时）；其数值有效、尺寸均为 640×480，但两者 K/D/R/P 相同且没有设备序列号，不能证明属于当前 ASTRA S。calibration_source 标为 local_yaml_device_binding_unverified，不把成功读流写成标定精度验收。以 color_info_url、depth_info_url、calibration_source 参数可显式指定其他来源；文件缺失或无效时拒绝观测。厂家 config/camera_info.yaml 名称为 test_camera，未采用。

2026-10-10 大图像传输修复：Astra launch 在使用 Fast DDS、且调用方没有指定 `FASTRTPS_DEFAULT_PROFILES_FILE` / `FASTDDS_DEFAULT_PROFILES_FILE` 时，自动为相机驱动和适配器两个子进程加载 `config/fastdds_camera.xml`。共享内存段为 **32 MiB**，同时保留 UDPv4 供发现与跨机通信；不修改全局 shell、其他节点或厂家源码。640×480 的原子 RGB-D 像素数据约 2.15 MB，默认约 512 KiB 段不足以稳定容纳大消息。[Fast DDS 官方说明](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html)也说明段尺寸接近或小于消息时存在丢失风险。实际对比见 VALIDATION；已有显式 DDS 配置或其他 RMW 保持原设置，需自行保证大图像传输容量。直接单独运行 rgbd_adapter 时，不会自动套用 launch 的环境配置。

图像后端在适配节点建立订阅前完成初始化，避免首次 OpenCV 加载耗时使首组图像超过有效期；实际图像处理超时仍按原门槛拒绝。

请求模式：RGB＋深度 640×480@30，IR/点云/GUI 关闭，深度注册到彩色，设备 RGB/深度同步请求开启，仅发布相机内部 TF。驱动请求成功、帧率和同框内参检查不等于物理对齐误差或硬件同步精度已验收。

另一个终端采集：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/wheeltec_motion_ws/install/local_setup.bash
ros2 run wheeltec_rgbd rgbd_capture --duration 5 \
  --output ~/workspace/wheeltec_motion_ws/runtime_logs/camera/latest
```

输出 report.json 与 rgbd.npz（RGB、米制深度、两套 K 和原始时间戳）。NPZ 不带车体位姿，不能直接视为世界地图。无需调用上游带 Go2 控制副作用的 ros_single_nav.py。

## ROS 契约

| 接口 | 类型/语义 |
| --- | --- |
| /wheeltec_camera/raw/color/image_raw、camera_info | 底层驱动的彩色与标定 |
| /wheeltec_camera/raw/depth/image_raw、camera_info | 请求注册后的驱动深度与标定；原始 16UC1 毫米 |
| /wheeltec_camera/rgbd | wheeltec_rgbd_interfaces/msg/RGBDFrame，原子 RGB-D 观测 |
| /wheeltec_camera/status | diagnostic_msgs/msg/DiagnosticArray，READY/WAITING/STALE 或具体拒绝原因 |

RGBDFrame 包含 color（rgb8）、depth（32FC1、米）、color_info、depth_info、sequence、calibration_source、rectified、valid_depth_fraction、pairing_delta_sec。深度零值、非有限值、超出 0.4–2.0 m 的像素变为 NaN；整帧没有可用深度时不发布观测。范围是 ASTRA S 当前使用配置，不是避障保证。

两张图各自保留驱动原始时间戳；bundle.header 使用两者中较新的时间。sequence 是适配节点本次进程内计数，重启会归零；消费者使用 after_stamp_ns 判断是否新观测。时间来源为驱动主机回调时刻，不是设备采集时刻。

配对必须满足：同一图像与其 CameraInfo 的时间戳完全相同；两张图时间差 ≤0.03 s；消息源年龄 ≤0.5 s，超前 ≤0.1 s；图像/内参 frame、尺寸一致，注册后的两套 K 相符且有效。默认 frame 为 wheeltec_camera_color_optical_frame。检查帧名和 K 不能替代实际标靶配准验收。

输入采用 BEST_EFFORT/KEEP_LAST，最多四条每路缓存；输出 BEST_EFFORT/KEEP_LAST=1。新鲜图像只能消费一次，旧包/重复/乱序不续期；非法数据清除配对缓存并报告错误，新完整有效组可恢复。处理后再次检查源时间，耗时过长的旧观测也不发布。断流时停止发布图像并报告 STALE，不重复发布旧观测来伪装持续取流。

参数在 src/wheeltec_rgbd/config/astra_s.yaml：四个输入 topic、output_topic、status_topic、expected_frame、frame_timeout、future_tolerance、sync_tolerance、depth_min/max；节点还接受 color_calibration_url、depth_calibration_url、calibration_source，通常由 launch 参数传入。以 `adapter_config:=/绝对路径.yaml` 传入自定义配置。仅处理 rgb8/bgr8、16UC1/32FC1；检查 stride、字节序和数据长度，不通过改尺寸/帧标签掩盖错误。输入 CameraInfo 保留在 raw 话题（当前驱动的深度 K 仍可能为 NaN，消费时使用自研 bundle）；适配层按输入 K/D/P 用 OpenCV 去畸变，RGB 用双线性采样、深度用最近邻，缺失边缘为 NaN。输出 CameraInfo 的 K 使用输入 P 的左 3×3，D 清零，R 为单位阵；输出为同一针孔投影。首版要求注册后的两套 K/D/P 一致，R 为单位阵，P 无立体基线；不支持的模型拒绝，不套用 D455 标定。

## Python 与 Co-NavGPT2 接入

RGBDReader 是独立 ROS Node，可加入上层 executor（建议由后台线程 spin），`read_latest(after_stamp_ns=上次戳)` 返回 Observation；没有新观测、过期或异常时抛 NoFreshFrame。读取同时检查源时间和单调接收年龄，即使上层很久没有执行订阅回调，也不会把过期缓存作为新观测使用。

```python
from wheeltec_rgbd.client import RGBDReader, NoFreshFrame
reader = RGBDReader()  # rclpy.init 后创建，并加入正在 spin 的 executor
last_stamp = 0
try:
    obs = reader.read_latest(after_stamp_ns=last_stamp)
    last_stamp = obs.stamp_ns
    rgb, depth_m, cam_K = obs.rgb, obs.depth_m, obs.cam_K
    # geometry/build_full_scene_pcd 使用米制 depth_m；无效值为 NaN。
    legacy_obs = obs.as_legacy_conav_observation()
    # 原版 ROS Agent 的 _preprocess_depth 内部 /1000，legacy_obs 只在这里转回毫米。
except NoFreshFrame:
    pass  # 等下一组新观测；已有导航运动时按上层契约调用 stop。
```

原版上游图像 callback 写死 uint16 和紧凑 stride，不能只改订阅话题就接入本 32FC1 输出。应使用上述适配入口；不要把 depth_m 直接交给会再次除以 1000 的原版预处理。该兼容函数只转换观测，不启动 Agent、不连接 Go2、不发速度，也不提供位置/朝向。

本机现有 YAML 已用于初步取流和去畸变，设备归属与实际精度仍需核对；相机装到车体后的外参也需核对。驱动只发布相机内部 TF，本接口不发布 camera_init/body/wheeltec_base 关系，不从厂家车型模板生成未经实测的车体 TF。光学轴按 ROS 相机约定处理：x 向右、y 向下、z 向前；世界坐标投影需上层结合正确外参和位姿。

本阶段完成相机初步接口，尚未完成现有标定文件的设备归属、物理像素配准误差、长时间稳定性、车体外参、运动场景建图或 Co-NavGPT2 自主探索验收。

最终 8 秒静止验收：128 组、约 20.98 Hz，最大源年龄 0.171 s，结束 READY；26 项相机测试通过。真实快照离线通过上游点云构造（降采样/聚类后 339 点），尚未接世界位姿。原始记录在 runtime_logs/camera/20261001-rgbd-interface-verified/，完整验证见 VALIDATION。

## RGB-D 与 FAST-LIO 位姿衔接（2026-10-01）

新增只读 `wheeltec_rgbd.posed.PosedRGBDReader`，继承 RGBDReader；同时订阅 `/Odometry`，要求 camera_init/body，返回 PosedObservation。调用方提供完整的 4×4 `body_from_base`、`base_from_camera`，并显式设置 `extrinsics_confirmed=True`。后者表示相机**彩色光学坐标系**到 wheeltec_base 的刚体变换，不是相机外壳原点；不得用单位矩阵占位后确认。第一台车 body_from_base 从当前 motion_control.yaml 的平移/RPY 构造，不能用厂家车型模板代替。真实相机外参目前仍未确认，所以本轮只验证合成数据。

```python
from wheeltec_rgbd.posed import PosedRGBDReader
reader = PosedRGBDReader(
    body_from_base=T_body_base,       # 当前控制配置构造的 4×4 矩阵
    base_from_camera=T_base_optical, # 实测并确认的光学坐标系外参
    extrinsics_confirmed=True,
)
# 加入正在运行的 executor；出现 NoFreshFrame 时停止本轮地图/规划更新。
posed = reader.read_latest(after_stamp_ns=last_stamp)
last_stamp = posed.observation.stamp_ns
obs = posed.observation
# 上游 build_full_scene_pcd(obs.depth_m, obs.rgb, obs.cam_K) 返回相机点云。
# world_points = posed.camera_points_to_world(camera_points)
T_world_base = posed.world_from_base
T_world_camera = posed.world_from_camera
```

变换为 `T_world_camera = T_world_body × T_body_base × T_base_optical`，保留三维姿态和高度；不能直接把相机位置用作底盘位置。接口不发布 TF，也不发送速度。地图若采用其他轴向，必须再显式适配，不直接宣称已符合上游二维地图坐标。

位姿缓存最多 16 条，按**深度帧**时间选最近位姿，默认差值 ≤0.05 s，不做外推。优先配对最新图像；最新图像暂时超前于位姿时，最多保留一组尚未消费的图像，等待后续位姿，从而避免每轮用更新图像替换等待对象导致持续配对失败；若位姿时间已越过该图像的配对区间且仍无匹配，就丢弃这一不可配对等待帧并改用后续新图像，不等到它过期。保留帧使用原始源时间与接收单调时间，过期、相机输入失效、定位故障或显式重置后清除；成功后再推进 last_stamp，不构建待执行图像队列。位姿源和单调接收超时均为 0.5 s、允许超前 0.1 s；相邻位移 >0.5 m、三维转角 >0.8 rad、时间倒退/重复或间隔超时均锁存异常并清空位姿。三维转角检查比原控制器仅检查 yaw 更严格，不改变运动节点门槛。源帧校验不代表物理曝光同步已验收。

锁存后上层必须停止导航、清除旧地图/任务、确认新定位原点，再显式 `reset_localization()`；该方法清除位姿和 RGB-D 当前缓存，不自动发布 stop，后续必须收到新数据。ROS 时间整体重置时应重建 reader 与上层任务。纯数值检测不能识别所有静止原点重置，受管 FAST-LIO 重启事件也必须由上层触发地图/任务重置。相机标定来源仍保留在 observation.calibration_source，外参确认不能替代内参设备归属及物理配准验收。
