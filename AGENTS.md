# 后续工作入口

- 开始本项目工作时先读 `docs/STAGE_SUMMARY.md`；按任务需要查阅当前 OPERATIONS / WORKSPACE_SETUP / CALIBRATION。
- 总体规划仍是开发资料中的 `document/Co-NavGPT2-baseline复现路线图.md`，涉及全项目阶段时再参考。该规划早期的 0.5 秒物理停车门槛已被用户暂缓，当前约定以阶段总结为准。
- 用户要求：旧逐次开发日志、原始实验记录、历史聊天不得作为默认上下文；仅在用户明确要求追查具体问题时读取。当前工作不依赖这些记录。
- 保留第一台车已使用的 `motion_control.yaml`；第二台车使用禁用输出的 example 模板并重新标定。
- 只维护自己的运动控制仓库。厂家底盘不改源码、不上传 GitHub；FAST-LIO 与 Livox/SDK 的路径/改动见 WORKSPACE_SETUP。
- 所有开发改动保持 Git 版本控制；当前事实和接口变化更新阶段总结，验证命令/结果更新 VALIDATION。原始运行输出放 runtime_logs，不堆入仓库。
- 不发送实车运动动作进行自动验收；需要试车由用户执行。已授权范围内的编译、隔离测试和只读检查可继续进行。
