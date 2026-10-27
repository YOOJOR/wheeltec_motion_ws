#!/usr/bin/env bash
# Isolated desktop simulator. Never calls the real robot startup scripts.
set -euo pipefail
sim_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sim_agent="$(cd "$sim_root/../conavgpt2_ws" && pwd)"
sim_runtime="$sim_root/runtime_logs/sim"
sim_name=conavbot-sim
sim_image=localhost/conavbot-sim:humble
sim_command="${1:-help}"
mkdir -p "$sim_runtime"

inside() { podman exec "$sim_name" bash /sim/ws/scripts/sim/inside.sh "$@"; }
session() { inside python3 -m conavbot_sim.session "$1"; }

create_container() {
  if podman container exists "$sim_name"; then
    if [[ "$(podman inspect -f '{{.State.Running}}' "$sim_name")" != true ]]; then
      podman start "$sim_name" >/dev/null
    fi
    return
  fi
  local display_args=()
  if [[ -n "${DISPLAY:-}" ]]; then
    display_args+=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
    if [[ -n "${XAUTHORITY:-}" && -f "$XAUTHORITY" ]]; then
      display_args+=(-e XAUTHORITY=/sim/xauthority -v "$XAUTHORITY:/sim/xauthority:ro")
    fi
  fi
  if [[ -d /dev/dri ]]; then
    display_args+=(--device /dev/dri --group-add keep-groups)
  fi
  podman run -d --init --name "$sim_name" --userns keep-id --user "$(id -u):$(id -g)" \
    --network private --ipc private --shm-size 1g --security-opt label=disable \
    -e ROS_DOMAIN_ID=71 -e ROS_LOCALHOST_ONLY=1 -e IGN_PARTITION=conavbot_sim \
    -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 \
    -e HOME=/sim/runtime/home -e XDG_RUNTIME_DIR=/sim/runtime/xdg \
    -e XDG_CACHE_HOME=/sim/runtime/cache \
    -v "$sim_root:/sim/ws:ro" -v "$sim_agent:/sim/agent:ro" \
    -v "$sim_runtime:/sim/runtime:rw" "${display_args[@]}" \
    "$sim_image" sleep infinity >/dev/null
  inside bash -c 'mkdir -p /sim/runtime/home /sim/runtime/xdg; chmod 700 /sim/runtime/xdg'
}

build_workspace() {
  create_container
  inside bash -c 'cd /sim/ws && colcon --log-base /sim/runtime/colcon-log build --base-paths src --packages-select wheeltec_motion_interfaces wheeltec_motion_control conavbot_sim --build-base /sim/runtime/build --install-base /sim/runtime/install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo'
  inside bash -c 'dpkg-query -W > /sim/runtime/apt-versions.txt; python3 -m pip freeze > /sim/runtime/python-versions.txt'
  podman image inspect -f '{{.Id}}' "$sim_image" > "$sim_runtime/image-id.txt"
}

start_session() {
  create_container
  if [[ ! -f "$sim_runtime/install/conavbot_sim/share/conavbot_sim/package.xml" ]]; then
    echo '请先执行 bash scripts/start_sim.sh build' >&2
    exit 1
  fi
  if [[ -S "$sim_runtime/session.sock" ]]; then
    session status
    return
  fi
  inside python3 -m conavbot_sim.scene \
    --config /sim/ws/src/conavbot_sim/config/simulation.yaml --output /sim/runtime/generated
  podman exec -d "$sim_name" bash /sim/ws/scripts/sim/inside.sh bash -c \
    'exec python3 -m conavbot_sim.session serve >> /sim/runtime/session.log 2>&1'
  echo 'Gazebo / RViz 正在启动；默认等待 RViz 点目标。日志：runtime_logs/sim/'
}

case "$sim_command" in
  build)
    mkdir -p "$sim_runtime/wheels"
    podman build --format docker -v "$sim_runtime/wheels:/opt/wheels:ro,Z" \
      --build-arg PIP_FIND_LINKS=/opt/wheels -t "$sim_image" \
      -f "$sim_root/scripts/sim/Containerfile" "$sim_root/scripts/sim"
    build_workspace ;;
  compile) build_workspace ;;
  up) start_session ;;
  goal|explore|preview|stop|estop|status) session "$sim_command" ;;
  export)
    session export ;;
  watch)
    if [[ -t 1 ]]; then
      podman exec -it "$sim_name" bash /sim/ws/scripts/sim/inside.sh python3 /sim/ws/scripts/sim/watch_status.py
    else
      inside python3 /sim/ws/scripts/sim/watch_status.py
    fi ;;
  reset)
    session shutdown
    for ((sim_wait=0; sim_wait<100; sim_wait++)); do
      [[ ! -S "$sim_runtime/session.sock" ]] && break
      sleep .2
    done
    if [[ -S "$sim_runtime/session.sock" ]]; then
      echo '旧会话仍在退出，检查 session.log 后重试。' >&2; exit 1
    fi
    start_session ;;
  down)
    if [[ -S "$sim_runtime/session.sock" ]] && ! session shutdown; then
      echo '会话控制端不可用；关闭该仿真容器并清理失效套接字。' >&2
    fi
    if podman container exists "$sim_name"; then podman stop -t 15 "$sim_name" >/dev/null; podman rm "$sim_name" >/dev/null; fi
    rm -f "$sim_runtime/session.sock" ;;
  logs) tail -n 60 "$sim_runtime/session.log" "$sim_runtime/stack.log" "$sim_runtime/task.log" ;;
  diagnose)
    podman exec -it "$sim_name" bash /sim/ws/scripts/sim/inside.sh \
      python3 /sim/ws/scripts/sim/record_timing.py "${@:2}" ;;
  record)
    podman exec -it "$sim_name" bash /sim/ws/scripts/sim/inside.sh ros2 bag record \
      -o "/sim/runtime/bags/$(date +%Y%m%d-%H%M%S)" \
      /cmd_vel /sim/motor_state /sim/ground_truth /wheeltec_motion/status \
      /sim/planned_path /conavbot/path /conavbot/decision ;;
  shell) podman exec -it "$sim_name" bash /sim/ws/scripts/sim/inside.sh bash ;;
  *)
    cat <<'USAGE'
Usage: bash scripts/start_sim.sh COMMAND
  build     构建容器镜像及 ROS 工作区
  compile   只重编工作区（已有镜像）
  up        启动 Gazebo/RViz，等待点击目标；车辆默认静止
  goal      退出探索，进入 RViz 点目标模式
  explore   开始持续 Agent 探索（会在仿真中运动，stop结束）
  preview   静止只读采集20秒并导出（不提交运动请求）
  export    请求当前 Agent 导出阶段，后台写入独立目录
  stop      结束当前任务，正常延迟制动
  estop     清除待执行指令并急停，保留物理制动
  reset     重启场景、时钟及全部任务，回到等待点目标
  down      关闭环境
  diagnose [--duration 120] [--label manual]  只读记录卡顿与时间链路
  watch     只读实时状态与事件；Ctrl+C 仅退出查看
  status | logs | record | shell
USAGE
    ;;
esac
