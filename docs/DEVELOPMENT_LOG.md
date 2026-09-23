# 开发日志

## 2026-09-23 初版开发（持续追加）

### 已确认范围
- ROS 2 Humble；麦克纳姆轮；MID360S 内置 IMU；FAST-LIO /Odometry 为反馈。
- 保留 turn_on_wheeltec_robot 和已烧录 STM32 固件，不实现固定 0.5 秒停车要求。
- 本机编译和模拟测试，实车测量、外参及试车由用户后续完成。
- 所有参数集中配置；版本控制与开发日志为本次明确要求。

### 建立日志前的操作（根据本次执行记录补记）
1. 阅读历史任务、路线图、键盘包、FAST-LIO 输出以及底盘通信源码。
2. 核实 wheeltec_robot_node 订阅 /cmd_vel；base_serial.launch.py 默认串口 /dev/wheeltec_controller、115200，底盘参考系 base_footprint。
3. 只读检查资料中的 WHEELTEC_C50X_2026.09.08.zip：存在有条件启用的命令丢失保护；未修改、未烧录。不能据此确定实车固件版本。
4. 检查本机：Fedora 44、Python 3.14.7，无 /opt/ros、ros2、colcon；存在 Podman。
5. Podman 初次受沙箱只读 /run 限制；随后通过执行权限审查读取镜像列表并成功拉取官方 ros:humble-ros-base-jammy。镜像 ID：ab91f7bbc8af87f468127aaa6c4d7f46fed4fa1f03b3a96b6ca44eed03d0ac0b。
6. ROS 文档网站查询遇到访问限制；后续以容器内 Humble API 和真实编译/集成测试验证实现。
7. 创建 ExecuteMotion.action、ROS 包元数据、独立几何及闭环控制核心，尚未完成 ROS 节点和测试。
8. 项目根目录不是有效 Git 仓库。为避免把厂商资料和 FAST-LIO 副本纳入版本控制，将本次新增的两个包移动到 code/wheeltec_motion_ws/src，建立独立 Git 仓库（main）。未移动用户已有代码。

### 日志规则
后续改动、测试命令、失败与修复均追加至本文件；完整编译/测试输出放 logs/ 并纳入 Git。提交记录标识开发阶段。日志不等同于实车验证。
