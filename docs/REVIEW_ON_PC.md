# 在个人电脑上获取和审阅完整开发代码

当前组织为两个大仓库：wheeltec_motion_ws提供硬件接口和工具，conavgpt2_ws包含完整探索适配源码与部署配置。原先同一仓库三个分支的方案已取消，未曾推送。

## 获取

GitHub目标仓库由用户确认后填写。分别克隆两个仓库到并列目录：

```bash
git clone <motion仓库地址> wheeltec_motion_ws
git clone <nav仓库地址> conavgpt2_ws
cd conavgpt2_ws
python3 scripts/fetch_upstream.py
```

仅阅读开发代码不需要执行最后一步。该脚本按deployment.lock.json从官方远端获取Co-NavGPT2与CLIP，递归初始化子模块；已有目录仅核对版本与已跟踪文件，不覆盖改动。支持--check只读检查。

## 目录与阅读顺序

1. wheeltec_motion_ws/docs/STAGE_SUMMARY.md：最新进度与坐标约定。
2. wheeltec_motion_ws/docs/SOURCE_AGENT.md：主运行链路。
3. conavgpt2_ws/development/Co-NavGPT2/WHEELTEC_ADAPTATION.md：与原版差异。
4. 同目录wheeltec/runtime.py、worker.py、agent.py、pipeline.py和agents/ros2_single_agent.py：观测→原版建图/前沿/FMM→动作。
5. motion仓库的NAVIGATION.md、CAMERA_INTERFACE.md、VISUALIZATION.md及对应src包：执行、相机与调试接口。
6. VALIDATION.md与对应测试：验证意义及实机边界。旧wheeltec_exploration是测试对照，不是主探索实现。

## 版本与运行边界

`development/Co-NavGPT2`现在是nav仓库普通目录，不是子模块，也没有内层.git。后续从conavgpt2_ws根目录统一提交和推送，历史已保留在同一Git提交图中。development.lock.json的source_tree记录该目录内容，可在仓库根目录用`git rev-parse HEAD:development/Co-NavGPT2`比较；后续改动此目录时同步更新锁定。

upstream、虚拟环境、模型权重、构建产物、原始日志、本机标定文件不上传；厂家与第三方依赖来源见WORKSPACE_SETUP.md和部署锁文件。历史补丁只用于追溯合仓前状态，不再需要重放。

当前运行配置包含Orin绝对路径并依赖ROS2 Humble/Jetson；普通电脑clone后可直接阅读，不代表能直接运行实车程序。先按测试说明准备环境并用隔离ROS域与合成输入。
