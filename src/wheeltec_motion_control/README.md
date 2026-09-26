# wheeltec_motion_control

ROS 2 Humble 相对直行/转向控制器、motion_client 和离线 calibrate_rotation。
只发布 Twist；串口属于原厂底盘节点，本包不发布 TF、不修改 STM32。

第一台车配置 `config/motion_control.yaml` 已启用输出；新车使用 `motion_control.example.yaml`。
Action `/wheeltec_motion/execute`；停止服务 `/wheeltec_motion/stop`。

当前文档：[阶段总结](https://github.com/YOOJOR/wheeltec_motion_ws/blob/main/docs/STAGE_SUMMARY.md)、[使用与调试](https://github.com/YOOJOR/wheeltec_motion_ws/blob/main/docs/OPERATIONS.md)。
