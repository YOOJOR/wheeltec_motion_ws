# wheeltec_motion_control

ROS 2 Humble 相对直行/转向、持续速度导航控制器，两个客户端和离线 calibrate_rotation。
只发布 Twist；串口属于原厂底盘节点，本包不发布 TF、不修改 STM32。

第一台车配置 `config/motion_control.yaml` 已启用输出；新车使用 `motion_control.example.yaml`。
Action `/wheeltec_motion/execute`；停止服务 `/wheeltec_motion/stop`。
导航输入 `/wheeltec_motion/navigation/cmd_vel`（TwistStamped），启用/退出
`/wheeltec_motion/navigation/set_enabled`（SetBool），状态 `/wheeltec_motion/status`。
导航限速与失效停止，不做路径跟踪；接入契约与不重发决策的 Python 适配器见
[NAVIGATION](https://github.com/YOOJOR/wheeltec_motion_ws/blob/main/docs/NAVIGATION.md)。

当前文档：[阶段总结](https://github.com/YOOJOR/wheeltec_motion_ws/blob/main/docs/STAGE_SUMMARY.md)、[使用与调试](https://github.com/YOOJOR/wheeltec_motion_ws/blob/main/docs/OPERATIONS.md)。
