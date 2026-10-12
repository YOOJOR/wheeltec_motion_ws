# 原版 Co-NavGPT2 Agent：探索、可视化与现场验收

当前主代码在 `~/workspace/conavgpt2_ws/development/Co-NavGPT2` 的 wheeltec-adaptation 分支，直接运行源码 ROS_Agent.mapping()。目标识别暂时关闭，VLM不接入；最近前沿/无前沿随机目标、点云建图、FMM和动作选择由原版Agent执行。`src/wheeltec_exploration` 是此前测试实现，作为对照保留，不能当作原版Agent。

最新验收状态（2026-10-12）：前一轮短程链路通过，但扩大到60s/0.5m/10rad后的测试出现运动时图像迟到并保护停止，持续探索尚未通过。当前应先排查输入时序，不直接扩大活动范围。人工停止模块入口和服务等待问题已修复；案例见VALIDATION最新节。

## 1. 三个分别启动的程序

以下入口都不会因执行 --check 而移动。先分别打开终端：

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_robot.sh
```

看到 STACK READY 才表示底盘/雷达/定位/运动接口健康；它仍在空闲输出零速度。首次FAST-LIO时间异常由受管脚本重启，探索不得恢复旧任务。

第二个终端只开奥比驱动和自研RGB-D接口：

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_astra_rgbd.sh --start max_publish_rate_hz:=20.0
```

20是自研原子帧发布上限，实际可能约15Hz；不是修改相机曝光/驱动30Hz模式。默认0保留不限频。多余配对被直接丢弃，不积队列、补发旧帧或改原时间戳。既有内参/配准流程和32MiB FastDDS共享内存保留。

大SHM配置仅由相机发布入口自动设置；探索入口使用调用方的DDS配置或中间件默认值。无需在全局终端导出相机专用配置。

第三个终端先只读原版Agent：

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_conav_exploration.sh --check
bash scripts/start_conav_exploration.sh --start --skip-initial-scan --duration 30 --map-size-cm 600 --input-stride 4
```

--check仅用合成图像检查真实Agent。--start默认也不启用导航；以上命令在车静止时看地图/规划建议。图像源年龄和计算耗时超过0.3s输出stop建议；请求源时间另由控制器按0.3s检查。普通只读默认要求实测一圈初始扫描，静止调图要加 --skip-initial-scan，避免45s后因未完成扫描锁存。

## 2. 可视化看什么

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_debug_rviz.sh
```

默认源码后端 /wheeltec_conav；--backend test_preview才显示旧测试实现。--minimal关闭原始图像/点云显示，保留地图与标记，降低Orin负载；完整图像调试建议停车进行。入口不启动硬件/Agent，不启用导航。RViz的Fixed Frame为camera_init，源码地图帧为conav_map。白/灰黑地图、青色前沿、黄色候选、绿色目标、橙色FMM短期目标，文字显示扫描/规划阶段。启动扫描时隐藏尚未用于控制的目标标记。奥比原始RGB/深度是独立图像窗口；Odometry表示IMU/body，不是地面底盘控制点。

可直接读状态：

```bash
ros2 topic echo /wheeltec_conav/decision
ros2 topic echo /wheeltec_motion/status
```

PLANNED仅说明新观测计算了规划建议。motion_output=false表示没有接控制。PROCESSING_EXPIRED是过期停止，INITIAL_SCAN是位姿累计扫描；HALTED需要新的显式运行/重置，不能把同一旧决定重发来续命。waiting/request_published=false表示这轮没有新请求，控制器按自己的20Hz定时器继续检查上一请求期限，不会逐轮停车重启。decision是规划/软件状态，不是实测速度。

## 3. 本车参考点与图像采样

底盘点是地面的有效旋转中心wheeltec_base；FAST-LIO的body通过当前 motion_control.yaml 转到它。相机矩阵在源码 wheeltec/config.yaml，奥比平移 [0.16083,0.00075,0.1157]m，光学轴x右/y下/z前转底盘x前/y左/z上。用户选择11.57cm高度，但厂家父帧、安装倾角、完整组合仍未核验，confirmed=false，不将其写成标定完成。D455安装矩阵为空，拒绝世界探索；本轮完全不使用D455。

图像步长4表示640×480采样为160×120，K同步除4，原40px裁边改10px，保留投影角度；降低点密度后小物体效果需现场验收。主运行入口默认步长4；地图默认6m，可选12/24m，5cm栅格。选择更大地图/更密采样前须重新静止检查计算期限。原版固定栅格膨胀、强制目标可通行及已走区清障策略保留，没有另加雷达避障。相机点用于建图，底盘点用于规划与速度参考；相机离底盘16cm不可忽略。

## 4. 有人现场观察的低速探索

下面命令会让车运动；只有现场观察、周围可转一圈且可随时停止时运行。它使用用户已选择但尚未完整标定的相机外参，明确通过允许暂定几何选项，不自动把confirmed改成true。

```bash
cd ~/workspace/wheeltec_motion_ws
bash scripts/start_conav_exploration.sh --start --motion --allow-provisional-camera --duration 45 --map-size-cm 600 --input-stride 4 --forward-speed 0.05 --turn-speed 0.25 --max-distance 0.2 --max-rotation 8.0
```

预计先左转一圈建立周围地图，再由原版FMM选择前进/左/右。0.05m/s、0.25rad/s均低于本车0.15/0.35上限；不修改控制器参数。不是固定角度Action循环，动作变化可在运动中更新；一次新的RGB-D计算只发一次TwistStamped，请求时间在开始计算该决定时取得；图像采集时间独立保留在source_stamp_ns。处理上限20Hz，只有新观测才会计算/更新；不循环重发缓存决定。

最长45s；从初始底盘点的位移达到0.2m、累计绝对转角达到8rad、源/处理过期、定位失效、接口退出、reset或异常，均结束本次运行并调用stop。累计转角包含左右来回，位移预算使用定位估计且停车有机械迟滞，不是精确物理边界；操作者要观察。停止后退出导航、清运动缓存，必须显式重新运行再接受新观测。关闭目标识别时没有物体任务成功终点。

人工停止：运行终端Ctrl+C，或独立终端：

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/wheeltec_motion_ws/install/local_setup.bash
python3 -m wheeltec_motion_control.navigation_client stop
```

随后查看运动状态IDLE和输出零，并观察真实车体是否停稳。服务成功/零速度证明软件停止链路，不证明实测物理停车时间。若处理超时，不延长0.3s门槛、不提高速度、不自动重新启用；先静止诊断图像/定位源年龄与CPU负载。未来进入同步VLM请求/长等待前同样明确stop。

## 5. 记录与故障定位

主程序保留最多3组传感器原子帧，按时间从新到旧选择50ms内的位姿配对，只转换选中的帧；这不是运动指令队列。图像与决策时间不同，0.3s图像/计算检查和控制器0.3s请求检查均保留。

只读记录器不启用导航、不发布速度，可以在运行前开：

```bash
cd ~/workspace/wheeltec_motion_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
python3 scripts/record_conav_acceptance.py --duration 60 --output runtime_logs/conav/acceptance/my-run.json
```

记录命令上限、唯一/cmd_vel发布者、源码决策、位姿变化及最终输出。位姿累计路径/转角包含定位噪声，不等于真实里程；优先看前后变化和时序。Agent报告默认 runtime_logs/conav/latest/report.json，逐帧 decisions.jsonl、退出保存的 map.npz；每轮验收用 --output 绝对新目录。默认latest报告和历史文件在新一轮覆盖，不是归档目录。已知导航启用被拒绝或服务缺失时不会stop别人的Action；已取得控制或启用结果不确定时按停止策略清理。

PROCESSING_EXPIRED退出时，report.json的last_decision保留触发帧：input_age_sec为进入Agent时的图像年龄，agent_sec为Agent调用耗时，source_age_sec为处理完成时图像年龄，processing_sec含读取/配对/处理。先看哪一段超出预算；不能只根据错误名称认定FMM耗时过长。服务发现和服务响应各自有限等待（默认各2s），不影响0.3s导航请求有效期，失败不重发运动决定。

只读清地图服务：

```bash
ros2 service call /wheeltec_conav/reset std_srvs/srv/Trigger '{}'
```

FAST-LIO重建原点、定位跳变或模式退出后，结束旧探索，核对定位，再显式新运行。运动中reset直接停止本次运行。不会恢复旧导航请求。整个系统/cmd_vel只由wheeltec_motion_controller发布，原版Agent写/wheeltec_motion/navigation/cmd_vel；手柄测试前退出自动控制及其受管脚本。

软件验证、真实传感器/移动结果和不足见 VALIDATION 的“原版Agent适配”记录。完整房间覆盖、避障距离、投影准确性、物理停车、长时间稳定性仍需独立验收，不把接口接通称作完整Co-NavGPT2任务成功。

2026-10-12 本车已按上述6m/步长4/低速配置完成一次有限探索：实际左转一圈后进入原版FMM，继续转向和前进，再按预算停车。定位估计前后位移约21.5cm；操作者确认最终停稳、无异常。最终请求时间有效，无导航超时。收尾已关闭全部本轮程序；下次需重新启动基础栈与相机。该结果证明短程链路接通，下一轮仍需现场观察更长探索和地图/障碍的一致性。
