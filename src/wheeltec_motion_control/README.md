# WHEELTEC FAST-LIO 闭环动作控制（初版）

本包只向已有底盘通信节点发布 Twist，不打开串口、不修改 STM32、不发布 TF。
实车入口仍为 `turn_on_wheeltec_robot`，本包只是新增的位置/航向外环。
使用说明及编译、测试记录见工作空间根目录 README.md 和 docs/DEVELOPMENT_LOG.md。

配置文件：`config/motion_control.yaml`。默认禁止速度输出，安装外参为未测量占位值。
接口：`/wheeltec_motion/execute`（ExecuteMotion Action）和 `/wheeltec_motion/stop`（Trigger）。
