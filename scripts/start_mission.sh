#!/usr/bin/env bash
# 三窗口启动；运行前请确认场地允许小车自动行驶。
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
HELPER="${SCRIPT_DIR}/mission_startup.py"

usage() {
  echo '用法: ./scripts/start_mission.sh [--check | --dry-run]'
  echo '默认打开识别、导航、规划器窗口；规划界面就绪 5 秒后自动请求完整巡视。'
}

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then usage; exit 0; fi
mode="${1:-run}"
[[ $# -le 1 && ( "$mode" == run || "$mode" == --check || "$mode" == --dry-run ) ]] || {
  usage >&2; exit 2;
}

# ROS 的环境脚本不保证兼容 nounset。
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u
settings="$(python3 "$HELPER" settings)"
mapfile -t values <<< "$settings"
export ROS_DOMAIN_ID="${values[0]}"
[[ -z "${values[1]}" ]] || export RMW_IMPLEMENTATION="${values[1]}"
image_topic="${values[2]}"
port="${values[3]}"
interval="${values[4]}"
insight=("${WORKSPACE_ROOT}/build/insight_cup/insight-cup"
  --profile ui-recording --source "ros:${image_topic}" --provider cpu
  --port "$port" --no-safe)
navigation=("${SCRIPT_DIR}/start_final_navigation.sh")
planner=("${SCRIPT_DIR}/start_new_map_planner.sh")

if [[ "$mode" == --dry-run ]]; then
  printf 'ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID"
  printf '%q ' "${insight[@]}"; echo
  printf '%q ' "${navigation[@]}"; echo
  printf '%q ' "${planner[@]}"; echo
  echo '规划界面节点出现后倒计时，再等待识别、定位和任务服务就绪，发送一次 START。'
  exit 0
fi
python3 "$HELPER" check
[[ "$mode" != --check ]] || exit 0
command -v gnome-terminal >/dev/null || { echo '缺少 gnome-terminal' >&2; exit 1; }
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || { echo '请在桌面会话中运行。' >&2; exit 1; }

# 总脚本等待三个窗口关闭，在此期间防止重复打开整套程序。
lock_path="${XDG_RUNTIME_DIR:-/tmp}/mission-startup-${UID}.lock"
exec 9>"${lock_path}"
flock -n 9 || { echo '一键启动任务仍在运行。' >&2; exit 1; }
run_dir="$(mktemp -d /tmp/mission-startup.XXXXXX)"
terminal() {
  local title="$1" role="$2"; shift 2
  local command_text
  printf -v command_text '%q ' "$@"
  {
    echo '#!/usr/bin/env bash'
    echo 'set -uo pipefail'
    printf 'cd %q\n' "$WORKSPACE_ROOT"
    printf 'echo $$ > %q\n' "${run_dir}/${role}.pid"
    if [[ "$role" == planner ]]; then
      printf 'python3 %q activate %q &\nactivation_pid=$!\n' "$HELPER" "$run_dir"
      echo 'trap '\''kill "$activation_pid" 2>/dev/null || true'\'' EXIT'
    fi
    printf '%s\n' "$command_text"
    echo 'result=$?'
    printf 'rm -f %q\n' "${run_dir}/${role}.pid"
    if [[ "$role" == planner ]]; then
      echo 'kill "$activation_pid" 2>/dev/null || true'
    fi
    echo 'echo "进程已退出，状态码 ${result}；按回车关闭窗口。"'
    echo 'read -r _'
  } > "${run_dir}/${role}.sh"
  # --wait 留在后台，由总脚本持锁等待所有窗口结束。
  gnome-terminal --wait --window --title="$title" -- bash "${run_dir}/${role}.sh" &
}
cancel_startup() {
  touch "${run_dir}/cancel"
  echo '已取消尚未发送的自动启动；已打开的窗口独立运行，已接受的任务需通过总控停止。'
  exit 130
}
trap cancel_startup INT TERM
terminal insight_cup insight "${insight[@]}"
sleep "$interval"
terminal final_navigation navigation "${navigation[@]}"
sleep "$interval"
terminal planner planner "${planner[@]}"
echo '已打开三个终端；规划界面就绪后自动倒计时启动。'
wait
rm -rf -- "$run_dir"
