# WHEELTEC FAST-LIO 闭环动作控制

本包向已有底盘通信节点发布 Twist，不打开串口、不修改 STM32、不发布 TF。
实车底盘为 `senior_mec_bs`，通信入口为 `turn_on_wheeltec_robot`；本包实现位置/航向外环。

完整使用与独立调试手册见 [工作空间 README](../../README.md)，包括启动环境、配置生效检查、Action 调用、反馈解释、定位延迟排查、底盘不动及失败结果诊断、日志留存。
开发与验证记录见 [DEVELOPMENT_LOG](../../docs/DEVELOPMENT_LOG.md)。

配置文件：`config/motion_control.yaml`。仓库默认禁止速度输出、外参为占位值；实车使用确认后的独立配置。
接口：`/wheeltec_motion/execute`（ExecuteMotion Action）和 `/wheeltec_motion/stop`（Trigger）。
