# 厂家功能包与相机技术链路

2026-10-01，从桌面 `ROS2-V5.0(humble)常用指令.txt` 追踪 `/home/wheeltec/wheeltec_ros2` 的源码、安装索引和配置。没有启动厂家整车、导航、跟随或探索功能，没有发送速度。“已安装”表示入口存在，不表示运行验收通过。

## 相机入口及数据契约

桌面命令为 `ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py`，环境来自厂家 install/setup.bash。本轮未执行。链路为 wrapper → wheeltec_param.yaml 的 camera_mode=astra → astra_camera/astra.launch.xml → astra_camera_node → OpenNI → ASTRA S。

当前默认：senior_mec_bs 底盘，ls_N10Plus_uart 二维雷达，/dev/wheeltec_lidar，stm32 IMU；ranger_avoid_flag 和 ultrasonic_avoid 均 false。厂家二维雷达链与目前 MID360/FAST-LIO 不同。相机 wrapper、Astra XML、车型/相机配置和车型 Nav2 参数的 install 文件均解析到源码链接，已核对读取路径。

| 项目 | 默认源码/配置 |
| --- | --- |
| 彩色 | 640×480、30 fps；/camera/color/image_raw，rgb8；/camera/color/camera_info |
| 深度 | 640×480、30 fps；/camera/depth/image_raw，16UC1、DEPTH_1_MM；/camera/depth/camera_info |
| 普通点云 | enable_point_cloud=true，/camera/depth/points，XYZ 单位 m |
| 彩色点云 | enable_colored_point_cloud=false；开启后驱动会启用配准 |
| 深度到彩色配准 | depth_registration=false |
| 设备 RGB/深度同步 | color_depth_synchronization=false |
| 相机内部 TF | publish_tf=true，默认 10 Hz；传感器 frame 与 optical frame |
| 压缩 | wrapper 添加 JPEG 彩色和 compressedDepth 重发布 |

XML 请求 IR，但 ob_camera_node.cpp::setupVideoMode 对非 UVC 设备会在彩色开启时禁用 IR，不能宣称 ASTRA S 三路同时工作。CameraInfo 使用提供的标定或设备参数，分辨率相同不代表 RGB/深度已经配准。点云只是单帧投影，不是持久地图。

onNewFrame 使用回调时 node_->now() 给 ROS 图像打时间戳，未直接搬运设备采集时间；CameraInfo 使用同一时间戳。因此 ROS 时间新鲜不能单独证明采集延迟或硬件同步。SDK 短时实测和硬件能力另见 CAMERA_ASSESSMENT；本轮没有重复取流或验证实际 ROS 发布。

camera.launch 只启动相机和压缩；sensors.launch 同时启动整车、雷达和相机。整车入口还包含串口、EKF、模型及 TF。相机内部 TF 不等于完整底盘 TF；底盘到 camera_link 由整车 robot_mode_description 发布。senior_mec_bs 模板 base_to_camera=[0.16083,0.00075,0.0897,0,0,0] 是待核对车型外参，不是本轮测量。单独相机入口不会自动连接 FAST-LIO 的 camera_init/body 与自研 wheeltec_base。

## 功能包清单

| 功能 | 主要包 | 源码/配置表达的能力 |
| --- | --- | --- |
| 底盘/遥控 | turn_on_wheeltec_robot、wheeltec_robot_keyboard、wheeltec_joy | Twist 串口发送，里程计/IMU/电池等回包，键盘/手柄输入 |
| 二维建图 | slam_gmapping、wheeltec_slam_toolbox、wheeltec_cartographer | 激光、里程计与 TF 的二维建图 |
| 地图导航 | wheeltec_nav2 | AMCL、代价地图、全局规划、局部控制、行为树、航点 |
| RGB-D 建图 | wheeltec_robot_rtab、orb_slam2_ros | RTAB-Map 与 ORB-SLAM2 RGB-D 入口 |
| 自主探索 | wheeltec_robot_rrt | RRT 前沿生成、筛选、目标分配及 Nav2 调用 |
| 跟随 | simple_follower_ros2、wheeltec_robot_kcf、aruco_ros | HSV/深度/PID、KCF、标记和激光相关示例 |
| 路线/编队/充电 | wheeltec_path_follow、wheeltec_multi、auto_recharge_ros2 | 路线跟随、主从编队、充电对接示例 |
| 交互 | bodyreader、tts、wheeltec_mic_ros2、wheeltec_mic_aiui、ollama_ros_chat、web_video_server | 身体/手势、语音与视频示例 |

常用指令中的去重 launch/run 入口，除 qt_ros_test 外均找到安装索引和入口。部分 launch 在 share 根目录或嵌套目录，不能只查 share/<pkg>/launch。Qt 示例在本厂家工作空间未找到索引/入口。相关设备、动态依赖、服务、模型、标定和运行性能未逐项验收。

## 技术逻辑

```text
STM32 速度/IMU → 串口节点 → /odom + /imu/data_raw
                                  ↓ 2D EKF，10 Hz
                              /odom_combined + TF
二维雷达 → /scan → SLAM 或地图 AMCL → map/代价地图
导航目标 → Navfn → RotationShim + MPPI（20 Hz）→ /cmd_vel → 串口 → STM32
```

EKF 融合 odom 的 vx/vy/wz 和 IMU 的 yaw/wz，不是视觉里程计。当前 Nav2 为 Navfn（use_astar=false），RotationShim 外层与 MPPI 主控制器，motion_model=DiffDrive；麦轮底盘不等于该配置已启用全向运动模型。主 Nav2 代价地图配置输入为 LaserScan 的 /scan、/scan2；主二维导航入口没有启动相机。

厂家 MPPI 配置 vx_max=0.5、vx_min=-0.5 m/s、wz_max=2.0 rad/s，对齐转向速度 1.8 rad/s。这些是规划控制参数，不是本车已验证上限。自研控制继续限定 0.15 m/s、0.35 rad/s。厂家 Nav2 controller_server 与跟随程序直接发布 cmd_vel，没有替自研控制做控制权仲裁，不能叠加启动并假定唯一发布者成立。

RTAB：RGB-D 经 rgbd_sync 近似配对（最大间隔 0.01 s），使用 /odom_combined 和 base_footprint；激光融合建图 subscribe_scan=true，pure3d 建图 false，但仍依赖轮式 EKF，Reg/Force3DoF=true，并非相机独立六自由度定位。

pure3d 导航入口启动整车、相机及两个 depthimage_to_laserscan 节点，将深度生成 /scan、/scan2（scan_height=10、250，range_min=0.45、range_max=10.0）供 Nav2 使用。pure3d 定位 subscribe_scan=true 可以使用上述深度扫描，不能仅因名称就认定矛盾；导航入口没有启用 localization include，桌面另列定位命令。range_max=10 m 是过滤配置，不是 ASTRA S 测距能力。laser1/laser2 的 TF 硬编码 x=0.34、z=0.5/0.1 m，与相机模板不是同一套数值，几何必须实际验收。

RRT：map/TF → 局部与全局随机树发现前沿 → MeanShift 聚类、地图/代价地图筛选 → 信息增益、距离代价和滞回选点 → /assigned_frontier_goal → 动作服务 → Nav2 NavigateToPose。桌面要求 RViz 点选四个边界点和一个内部起点。该几何探索不能替代 Co-NavGPT2 的语义目标、VLM 和 FMM。

visual_follower：HSV 阈值/轮廓 → 目标附近深度估距 → 偏角/距离 PID → Twist。源码有固定 VGA 尺寸和视角假设；不是开放词汇目标识别，更换相机/分辨率不能默认兼容。

## 对 Co-NavGPT2 的复用与下一步

1. 复用相机驱动、标定和消息；单独相机 ROS 静止验收编码、帧率、时间配对、CameraInfo、有效深度和配准，不运行带底盘输出的导航/跟随入口。
2. 核对安装外参与 optical frame 轴向，明确 FAST-LIO body、底盘控制点与相机之间唯一 TF/外参发布者。
3. 自研适配层统一深度毫米到米、内参与配对，先用真实观测验证地图更新，不输出运动。
4. 接已部署模型/FMM 与持续速度接口；阻塞 VLM 前 stop，新决策产生新鲜请求，保留现有限速和超时。
5. 实车探索和物理停车由用户验收。本轮仅完成静态审计，不代表厂家导航或 Co-NavGPT2 探索跑通。

主要厂家源码目录：turn_on_wheeltec_robot/launch、config；ros2_astra_camera-master/astra_camera；wheeltec_robot_nav2/launch、param；wheeltec_robot_rtab/launch；wheeltec_robot_rrt2/src/rrt_exploration；simple_follower_ros2/simple_follower_ros2。
