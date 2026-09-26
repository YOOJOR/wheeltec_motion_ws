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

### 实现阶段 1
- 建立独立 Git 仓库并提交初始核心：fad9010；提交身份为 Codex <codex@local>，未修改全局 Git 配置。
- 新增 ROS Action 服务节点、停止服务、测试客户端、启动文件及集中参数 YAML。
- 位姿进行完整三维外参组合后投影到平面；使用源时间戳与单调时钟同时检查新鲜度。
- 参数支持空闲时在线修改；话题名称、控制/反馈频率等需重启。执行中拒绝调参。
- 默认为 control_enabled=false、extrinsics_calibrated=false；禁用时完全不发布速度。
- 控制支持前后直行、连续展开航向的相对旋转、可选横移纠偏；完成判断必须经历新的静止位姿样本。
- 审查动作接受与回调并发：修正前一个结果影响下一个动作的完成事件问题，并处理接受期间 stop 请求。
- 镜像下载成功，开始检查容器内 colcon/pytest/CMake。容器不使用主机网络，不连接任何机器人。

### 实现阶段 2 与首轮测试
- 新增 CLI、集中 YAML、启动文件、README，以及纯核心测试和真实 ROS Action/Topic/Service 集成测试。
- 执行主机 Python unittest（命令：`PYTHONPATH=src/wheeltec_motion_control python3 -m unittest discover -s src/wheeltec_motion_control/test -p test_core.py -v`）：16 项中 1 项失败，原始输出 logs/core-host-first.log。
- 失败原因：10.3-10 的浮点表示略大于 0.3，恰好处于控制调度间隔阈值的过冲测试被误判。对比较增加 1e-9 数值容差，物理阈值不变。
- 容器环境确认：colcon、pytest 6.2.5、CMake 3.22.1 可用。
- 第一次编译脚本在 source Humble setup.bash 时失败：`AMENT_TRACE_SETUP_FILES: unbound variable`。原因是脚本启用了 nounset，而 ROS setup 会访问未设置环境变量；改为 `set -eo pipefail`，保留失败退出和管道错误传播。
- 正式编译/测试使用 Podman `--network=none`、`ROS_DOMAIN_ID=173`、`ROS_LOCALHOST_ONLY=1`，只挂载此工作空间；命令记录于 README。

### 编译通过后的测试发现修复
- 首轮真实 Humble 编译成功：2 packages finished。输出保存 logs/colcon-build-first.log。
- 但 colcon test 报告 0 tests（保留 logs/colcon-test-no-discovery.log），不能视为测试通过。
- 修复：setup.py 声明 tests_require=['pytest']，setup.cfg 指定 testpaths=test，并在脚本中检查 XML 测试用例数必须大于零。
- 改进正常 Ctrl-C 行为：节点及客户端使用 Python 信号处理，避免 rclpy 在清理/取消前提前关闭上下文。强制 kill 和链路中断仍不承诺停车时间。

### 核心及 ROS 集成验证结果
- 主机 Python 3.14.7：16 项核心 unittest 全部通过，输出 logs/core-host-final.log。
- ROS 2 Humble / Ubuntu 22.04 容器 / Python 3.10.12：两个包真实 colcon build 成功（包含 Action C/C++/Python 消息生成）。
- colcon test 实际发现并执行 22 项：16 项核心测试 + 6 项 ROS 集成测试；22 passed，0 errors，0 failures，0 skipped。完整输出 logs/colcon-test.log 与 logs/colcon-test-result.log。
- 集成测试涉及真实 Action 接受/拒绝/反馈线程、连续动作、取消、停止服务、位姿话题丢失/错误frame/旧源时间戳、超时、无进展以及在线参数和输出开关；合成底盘仅使用 /motion_test/*。
- 额外检查安装后的 launch 与 CLI 入口：使用输出禁用的默认配置；验证 move 被拒绝、stop 返回、Ctrl-C 清理，无实车连接。脚本 scripts/smoke_entrypoints.py，输出 logs/launch-smoke.log 和 logs/client-smoke.log。
- git diff --check 与 Python compileall 用于补充格式/语法检查；不替代上述 ROS 测试。

### 初版交付收尾
- 安装后入口检查通过：launch 正常启动；默认禁止运动的动作请求按预期拒绝（CLI 退出 1）；stop CLI 返回成功（退出 0）；Ctrl-C 后节点干净退出，无 traceback。
- 保存 pytest XML 到 logs/pytest-results.xml；添加 CHANGELOG.md。
- 本阶段成果将提交为 `feat: add configurable FAST-LIO motion action server and verified Humble tests`，并以 `v0.1.0` 标记；精确提交哈希由 Git 历史查询。
- 所有新增源文件、参数、说明、开发日志及编译/测试输出纳入该独立仓库；编译缓存不纳入。
- 没有启动实车、没有连接串口、没有更改厂商代码或固件，也没有上传仓库。未来实车参数、试验结果应继续追加日志并提交。

### 版本与可复现信息归档
- 初版实现提交 b4d2610，版本标签 v0.1.0；初始核心提交 fad9010。
- 使用 `podman image inspect` 记录镜像 ID 与仓库摘要到 logs/container-image.txt，便于固定同一编译环境。
- 最后的 git diff --check 只报告原始工具日志自身的行尾空格/结尾空行，源代码无此问题。为保留日志原文，添加 .gitattributes 对 logs/** 禁用空白检查，而不重写原始输出。
- 本次归档仅修改日志/版本属性，不改已测试代码；不重复运行已通过的测试。

## 2026-09-25 GitHub SSH 身份验证配置（本机完成，账号端待添加）

- 用户请求配置 GitHub 身份验证。检查本机无 gh、无 SSH 密钥文件；SSH 代理无 identities。本仓库没有远端。
- SSH 代理检查首次受沙箱限制，按权限流程后成功检查。
- 创建专用 Ed25519 密钥 ~/.ssh/id_ed25519_github（无额外口令），私钥权限 600；只显示公钥，没有读取或记录私钥内容。未覆盖已有密钥。
- 新建 ~/.ssh/config，仅对 github.com 指定该 IdentityFile、User git、IdentitiesOnly yes，权限 600；ssh -G 已确认配置生效。
- 公钥指纹 SHA256:4l75t0ok4HfRXYsJdhWoyLwPlQSdfPqxdX4mAUmbksI。密钥文件均在工作空间之外，不纳入 Git。
- 已打开 https://github.com/settings/ssh/new，页面重定向至 GitHub 登录页。需要用户登录并添加公钥；账号端尚未完成，不能标记认证成功。
- 用户添加后需执行 ssh -T git@github.com 验证，并核对官方主机指纹；没有创建远端仓库或推送代码。
- 补记：ssh -G 在沙箱内首次报告系统 ssh_config 文件权限问题；在正常主机权限下重试成功，确认 hostname=github.com、user=git、identitiesonly=yes、identityfile=~/.ssh/id_ed25519_github。没有修改系统 SSH 配置。用户已有未跟踪的 src/.vscode/ 保持不动。

## 2026-09-25 离线旋转外参辅助标定工具

- 用户授权开发：读取 FAST-LIO /Odometry 录包，计算 IMU 到底盘有效旋转中心的水平偏移，输出轨迹与残差；不修改控制器/STM32，不自动应用参数。
- 新增 calibration.py，支持 rosbag2/sqlite3 与标准 CSV；以每段独立固定旋转中心拟合 p_i + R_i t = c；左右转分别计算与交叉比较。
- Z 不估计：未提供时仅报告 Z=0 条件下的 X/Y；倾斜安装且高度未知则质量检查不通过。只有提供 body_z 且检查通过才输出禁用控制、标定未确认的 YAML 建议片段。
- 参数化采样数、转角覆盖、转速、时间间隔、跳变、条件数、残差与左右差异阈值；拒绝覆盖现有输出目录。
- 检查 Humble 容器依赖：numpy、rosbag2_py 已有，matplotlib 缺失。建立临时 wheeltec-calibration-check 容器，通过系统官方包仓库安装 python3-matplotlib（日志 calibration-dependencies.log），主机环境不变。
- 新增合成真值、噪声、倾斜、漂移、反向不一致、无观测/无效数据，以及真实 rosbag2 写入/读取/图表/报告端到端测试。
- 首轮编译通过，29 项测试中 28 通过、1 失败（logs/calibration-test-first.log）。失败为合成漂移数据每段重新从原点开始，产生 0.4 m 突变，触发预期的跳变拒绝而未进入残差检查；修正测试生成器为跨段连续漂移，保留原跳变保护。
- 控制包版本更新 0.2.0；接口包保持 0.1.0（消息定义未变化）。
- 修正数据后真实 Humble 全部 29 项测试通过（新增 7 项标定测试，原 22 项回归），两个包编译成功。保留日志及 XML。
- 安装后的 calibrate_rotation --help 已验证；合成噪声示例真值 [-0.2, 0.03, -0.4]，输出约 [-0.1998121, 0.0300212, -0.4]，左右差约 0.0003603 m。这只验证算法，不代表实车精度。
- 已生成并目视检查两栏轨迹/残差图，布局清晰，图与示例报告保存 logs/calibration-synthetic-example.*。
- README 更新测试镜像构建方式，新增 scripts/Containerfile 安装 matplotlib；既有基础镜像没有绘图库，不能直接跑新增绘图测试。
- 测试结束后移除本次专用临时容器 wheeltec-calibration-check，工作空间内输出保留。未连接机器人、未改动控制算法/固件、未应用任何外参、未推送远端。
- 本次作为 v0.2.0 提交并打标签；精确提交号以 Git 历史为准。
- 提交前空白检查发现测试文件多余尾部空行，已移除；只影响格式，无需重跑算法测试。提交命令因该检查未执行，随后校正本次尚未发布的 v0.2.0 本地标签到实际提交。

## 2026-09-25 首次用户实车录包分析

- 输入 /mnt/rosbag/imu_rotation_calibration，只读挂载；元数据 227 帧、约 22.6 秒。原文件哈希保存 analysis/20260925-rotation/source.json。
- 首次容器调用将 PYTHONPATH 替换为源码路径，导致 rosbag2_py 不可见；改为追加并保留 ROS 环境后成功。未修改分析算法。
- v0.2.0 默认参数，未提供 body_z：质量检查通过，使用 205 帧；X=-0.1079212461 m、Y=-0.0233502935 m（以 Z=0 为条件，Z 未测量）。左右转覆盖约 404.3°/418.2°，估计差 4.32 mm，共同参数中心残差 RMS 9.15/10.54 mm。
- 本机及基础 ROS 镜像缺 matplotlib，按已有 scripts/Containerfile 构建可复用 wheeltec-motion-test:humble，构建输出保留。通过同一 CSV 生成轨迹图并目视核对；结果见 analysis/20260925-rotation/with-plot/。
- 原始 IMU 轨迹呈明显圆弧；补偿后中心分布收缩到厘米量级。分段中心有差异，不将其解释为外参绝对精度；没有放宽阈值，没有生成完整可用 YAML。
- 仍需用户测量/换算 body Z、确认轴向及底盘参考点。不改动实车配置，不标记外参已完成。


## 2026-09-26 使用与独立调试手册

- 用户要求 README 讲清用法和调试，使其能独立判断问题；本次为文档维护，不启动实车、不发送速度、不改变控制算法或运行参数。
- 核对 Action 定义、客户端参数、控制器拒绝/失败分支、配置默认值、launch 参数以及本地厂家 base_serial.launch.py；未读取密钥或修改厂家代码。
- README 补充两个包及 Action/Topic/串口职责、20 Hz 控制与 5 Hz 反馈、配置加载/运行值检查、反馈及容差解释、按故障分层的检查命令和判断、参数调整原则、日志与 Git 同步流程。
- 修正文档旧信息：本车 senior_mec_bs；已有远端；用户已填写外参并完成一次低速直行。仓库默认配置仍禁用且零外参，不将用户实车配置复制覆盖到默认文件。
- 引用用户提供的实测输出：过期定位约 1.4 s；重新运行后平均 31 ms、最大 59 ms；0.2 m 目标最终剩余约 0.02097 m、6.50 s 成功。明确具体重启组件未完整记录、延迟根因未查明、结果不是外部精度测量。
- 包级 README 链接到完整手册；.gitignore 添加 runtime_logs/，日常原始日志不自动入库，重要结论/证据按需归档。
- 保留已有未跟踪 analysis/20260925-rotation/height-201mm/，不纳入本次提交。检查文档链接、代码块、Shell 语法与 git diff --check；本次无代码变化，不重复运行 ROS 控制测试。


## 2026-09-26 SSH 定位延迟复现排查

- 用户报告 Odometry 约 1.4 s，动作因源时间戳过旧被拒绝；授权 SSH 检查。连接时 Livox/FAST-LIO 已退出，用户随后重新启动底盘通信、lclivox 和 lclocal，故原异常运行状态未能直接测量。
- .bashrc alias：lclivox 启动 msg_MID360s_launch.py（10 Hz、自定义点云）；lclocal 启动 fast_lio mapping.launch.py config_file:=mid360.yaml（默认还启动 RViz）。正常和异常两次日志均显示 RViz 随后退出，不能由此证明 RViz 是根因。
- 确认各进程路径来自 vendor_ws/conavGPT_ws；Odometry 和 Livox 点云各只有一个发布者，laser_mapping use_sim_time=False。底盘包不生成 FAST-LIO 的 Odometry，暂无证据把问题归因于工作空间抽取。
- 新增独立只读脚本 scripts/diagnose_pose_delay.py，同时用 raw=True、best-effort/depth=1 订阅 IMU/点云/Odometry，避免 Python 逐点反序列化干扰；解析帧头与末点时间，并在测量结束用一帧完整 ROS 反序列化交叉核验 CDR 解析。仅被动订阅，没有打开串口、发布速度或修改控制参数。
- 首次干净 env -i 运行缺 HOME 导致 ROS 日志路径展开失败；指定独立 ROS_LOG_DIR 后成功，不修改 HOME、用户配置或控制器。
- 15 s 结果：IMU 平均 0.000581 s；点云帧头平均 0.114975 s，帧长约 0.1 s、末点通常仅旧 0.008~0.016 s；Odometry 平均 0.039588 s，150 帧。解析与 ROS 原始消息一致。随后标准 topic delay 约 0.042 s，与轻量测量一致。
- 本次只能证明重新启动后的链路正常，不能证明故障永久修复，也不能确定队列积压的具体位置。默认 pose_timeout=0.5 保留。故障再现时应先分段采样和留存状态，不立即重启或放宽阈值。
- 原始输出选择归档至 logs/20260926-delay；脚本语法/CLI 检查与实车被动采样验证，不重复运行运动测试。不记录 SSH 凭据，不修改 STM32/厂家代码，不发送动作。
- 最终脚本已传到小车 scripts/diagnose_pose_delay.py，--seconds 3 被动检查正常退出，位姿样本均值 0.042705 s。首次发现阶段可能显示 no data，不直接等同于定位中断。当前 analysis/ 已由用户忽略，原始证据改存 logs/20260926-delay 纳入版本。

## 2026-09-26 一键启动及自动恢复

- 用户授权实现：通信 → Livox → FAST-LIO → Motion 接口，运行期间异常自动重启；随后明确允许中断现有节点进行接管验证。脚本不发送动作目标，不重放旧请求。
- 新增 scripts/start_robot.sh、robot_stack/config.yaml、supervisor.py、health.py、使用说明及无 ROS/硬件的故障恢复测试。
- 显式加载系统 Humble 及四工作空间 local_setup.bash；旧 overlay 路径仅在脚本子环境清除，保留 ROS_DOMAIN_ID/RMW/网络设置，不更改 .bashrc、厂家驱动、STM32 或运动参数。
- 按实际新数据而非固定 sleep 判断就绪；PowerVoltage 回包、Livox 原始帧头/IMU、连续有效 FAST-LIO 位姿、Action 服务和唯一控制速度发布者。读取实车 motion YAML 中的源时间/接收超时门槛，不放宽 0.5 s。
- 受管进程退出、子节点退出但 launch 尚在、数据失效均进入恢复。定位/通信异常先撤下 motion；按依赖关系重启。底盘重启保留定位地图；Livox 或 FAST-LIO 重启重置定位，不恢复旧动作。SIGINT 正常退出优先，必要时仅升级自己创建的进程组；启动/恢复指数退避、单实例锁与已有手动节点检测。
- 本机 8 项测试通过：等待数据和启动顺序、Livox/通信/控制器退出、旧位姿撤下接口、启动超时及退避、退出顺序、真实子进程组清理。小车同 8 项测试通过；--check 确认环境/配置及四个现有手动节点；8 s 被动健康检测四组件均通过，未发送速度。
- 用户追加授权后停止当前动作并依次退出现有手动 launch，再用新脚本接管；验证记录随后补充。正常监测不自动发布额外 Twist，使用原控制器的正常退出路径请求零速度。已有物理停车能力不由脚本承诺。
- 实车接管验证完成：先 motion_client stop，再退出四个已核对包名的手动 launch；后台启动 supervisor PID 15624，原始运行目录 runtime_logs/robot_stack/20260926-214333-15624。
- 首轮启动又实际出现 FAST-LIO 源时间年龄约 1.345 s；接口没有启动，45 s 就绪超时自动重启定位后恢复，随后位姿平均约 42.9 ms 并进入 STACK READY。此为实际旧位姿恢复验证，不认为根因已修复。
- 在无动作状态对自己启动的四个子节点逐一 SIGTERM：motion 约 20.5 s、FAST-LIO 约 9.0 s、Livox 约 75.1 s、base 约 8.0 s 后恢复整体就绪。Livox 发生一次额外数据超时重试；恢复时间受驱动数据/ROS 发现/就绪门槛影响，不承诺固定时间。
- 各阶段没有发送 move/rotate/Action goal；base 恢复保留运行中的 Livox/FAST-LIO，定位链故障撤下 motion。验证后总控继续在后台运行，用户无需再并行手动启动四个节点。
- 选择归档到 logs/robot-stack：单元测试、环境检查、被动健康检查、故障注入结果、分段位姿延迟和总控日志。日常 runtime_logs 保持忽略；未推送 GitHub。
