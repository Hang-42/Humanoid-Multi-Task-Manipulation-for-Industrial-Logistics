#!/usr/bin/env bash
# Qi entry for the upper camera stack and the local OpenPI inference service.
set -Eeuo pipefail

task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
task_upper="$task_dir/upper"
task_host=${PI05_UPPER_HOST:-leju_kuavo@192.168.26.12}
task_session=pi05-upper-camera
task_inference="$task_dir/../pi05/tools/start_inference.sh"

# Inference is explicit: it runs in the foreground and never publishes robot actions.
case "${1:-}" in
  --pi05-check|--pi05-start|--pi05-status)
    test "$#" -ge 3 || {
      echo 'Usage: ./deploy_pi05.sh --pi05-{check|start|status} --model {cz|kd12|lj|lp} [--port N] [--gpu N] [--host IP]' >&2
      exit 2
    }
    task_mode=${1#--pi05-}
    shift
    exec bash "$task_inference" "--$task_mode" "$@" ;;
  --pi05-install)
    test "$#" -eq 1 || { echo 'Usage: ./deploy_pi05.sh --pi05-install' >&2; exit 2; }
    exec bash "$task_dir/../pi05/tools/install_env.sh" --install ;;
esac

case "${1:-}" in
  ''|--start) task_start=true ;;
  --install) task_start=false ;;
  --check)
    bash -n "$task_dir/deploy_pi05.sh" "$task_upper/upper_start.sh"
    python3 - "$task_upper/start_camera.launch" <<'PY'
import sys
import xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
assert root.tag == 'launch'
print('PI05_UPPER_SOURCE_OK')
PY
    python3 -m py_compile "$task_upper/camera_codec.py" "$task_upper/camera_relay.py"
    exit 0 ;;
  *) echo 'Usage: ./deploy_pi05.sh [--check|--install|--start|--pi05-check|--pi05-install|--pi05-start|--pi05-status]' >&2; exit 2 ;;
esac

for task_file in upper_start.sh start_camera.launch camera_relay.py camera_codec.py package.xml CMakeLists.txt; do
  test -s "$task_upper/$task_file" || { echo "Missing $task_upper/$task_file" >&2; exit 2; }
done
[[ "$task_host" =~ ^[A-Za-z0-9_.@-]+$ ]] || { echo 'Invalid PI05_UPPER_HOST' >&2; exit 2; }
ip -4 addr show | grep -q 'inet 192.168.26.20/' || { echo 'Qi robot LAN address is missing' >&2; exit 2; }
ip -4 route get 192.168.26.12 | grep -q 'src 192.168.26.20' || { echo 'Upper computer route is not on robot LAN' >&2; exit 2; }

task_version=$(cd "$task_upper" && sha256sum upper_start.sh start_camera.launch camera_relay.py camera_codec.py package.xml CMakeLists.txt | sha256sum | cut -c1-16)
task_release="/home/leju_kuavo/pi05_upper/releases/$task_version"
task_remote="$task_release/upper"
task_ssh=(-o BatchMode=yes -o ConnectTimeout=12 -o StrictHostKeyChecking=yes)

ssh "${task_ssh[@]}" "$task_host" "install -d -- '$task_release'"
scp -r "${task_ssh[@]}" "$task_upper" "$task_host:$task_release/"
task_local_hashes=$(cd "$task_upper" && sha256sum upper_start.sh start_camera.launch camera_relay.py camera_codec.py package.xml CMakeLists.txt)
task_remote_hashes=$(ssh "${task_ssh[@]}" "$task_host" "cd '$task_remote' && sha256sum upper_start.sh start_camera.launch camera_relay.py camera_codec.py package.xml CMakeLists.txt")
test "$task_local_hashes" = "$task_remote_hashes" || { echo 'Upper computer transfer checksum mismatch' >&2; exit 3; }
ssh "${task_ssh[@]}" "$task_host" "chmod 755 '$task_remote/upper_start.sh' '$task_remote/camera_relay.py' && bash -n '$task_remote/upper_start.sh' && python3 -m py_compile '$task_remote/camera_relay.py' '$task_remote/camera_codec.py'"
echo "PI05_UPPER_INSTALLED $task_remote"
if test "$task_start" != true; then exit 0; fi

# Check the live ROS master and camera-node ownership before creating a session.
ssh "${task_ssh[@]}" "$task_host" "'$task_remote/upper_start.sh' --preflight"
# An existing tmux session belongs to the operator; do not replace it.
if ssh "${task_ssh[@]}" "$task_host" "tmux has-session -t '=$task_session' 2>/dev/null"; then
  echo "Session $task_session already exists; inspect it before starting another instance." >&2
  exit 3
fi
ssh "${task_ssh[@]}" "$task_host" "tmux new-session -d -s '$task_session' -c '$task_remote' './upper_start.sh'"
sleep 3
ssh "${task_ssh[@]}" "$task_host" "test \"\$(tmux display-message -p -t '$task_session' '#{pane_dead}')\" = 0 || { tmux capture-pane -pt '$task_session' -S -35; exit 4; }"
echo "PI05_UPPER_STARTED session=$task_session release=$task_version"
