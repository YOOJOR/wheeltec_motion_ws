# 在个人电脑上获取和审阅完整开发代码

三个本地独立仓库同步到同一个 GitHub 仓库的三个分支；它们是并列组件，不是等待合并到 main 的功能分支。不要将 deployment 或 wheeltec-adaptation 合并进 main。

| GitHub 分支 | 本地目录 | 内容 |
| --- | --- | --- |
| main | wheeltec_motion_ws | 运动接口、RGB-D接口、测试预览、启动脚本、文档 |
| deployment | conavgpt2_ws | 环境与依赖锁定、模型来源、源码版本锁定及适配补丁 |
| wheeltec-adaptation | conavgpt2_ws/development/Co-NavGPT2 | 完整原版Agent源码及本车适配，当前主探索实现 |

## 获取

在你电脑选定的工作目录执行，无须安装ROS即可阅读：

```bash
git clone --branch main https://github.com/YOOJOR/wheeltec_motion_ws.git wheeltec_motion_ws
git clone --branch deployment --single-branch https://github.com/YOOJOR/wheeltec_motion_ws.git conavgpt2_ws
git clone --branch wheeltec-adaptation --single-branch https://github.com/YOOJOR/wheeltec_motion_ws.git conavgpt2_ws/development/Co-NavGPT2
```

私有仓库需使用你的GitHub账号授权；配置SSH后也可以将HTTPS地址替换为git@github.com:YOOJOR/wheeltec_motion_ws.git。

## 推荐阅读顺序

1. wheeltec_motion_ws/docs/STAGE_SUMMARY.md：现状、最新坐标约定和未解决问题；历史段落按日期阅读。
2. wheeltec_motion_ws/docs/SOURCE_AGENT.md：当前主运行链路和现场步骤。
3. conavgpt2_ws/development/Co-NavGPT2/WHEELTEC_ADAPTATION.md：与原版的差异。
4. 同目录wheeltec/runtime.py、worker.py、agent.py、pipeline.py：观测配对、独立计算进程、坐标适配和动作出口；agents/ros2_single_agent.py是主探索算法。
5. wheeltec_motion_ws/docs/NAVIGATION.md及src/wheeltec_motion_control：连续速度接口、模式互斥和停止契约。
6. wheeltec_motion_ws/src/wheeltec_rgbd、docs/CAMERA_INTERFACE.md及docs/VISUALIZATION.md：相机、外参和TF。
7. docs/VALIDATION.md与相应测试：验证输入、结论与实机验证边界。

旧wheeltec_exploration属于测试对照，不能将其误认为当前主探索算法。

## 一致性与复现边界

conavgpt2_ws/development.lock.json记录原版适配提交和源码树；可用以下命令与锁文件比较：

```bash
git -C conavgpt2_ws/development/Co-NavGPT2 rev-parse HEAD
git -C conavgpt2_ws/development/Co-NavGPT2 rev-parse 'HEAD^{tree}'
```

这三个克隆包含自研开发和原版适配代码。厂家底盘、FAST-LIO/Livox依赖、模型权重、虚拟环境、构建产物、原始日志和相机本机标定文件不随代码上传；来源和部署步骤见WORKSPACE_SETUP.md及部署仓库锁文件。需要独立原版参考时，按deployment.lock.json克隆官方仓库。

当前配置含Orin绝对路径，运行依赖ROS2 Humble、Jetson环境和外部依赖，复制到普通电脑不等于可以直接运行实车程序。先用于阅读；运行测试时按各测试说明使用隔离ROS域及合成输入。
