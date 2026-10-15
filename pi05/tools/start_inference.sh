#!/usr/bin/env bash
# Start one OpenPI policy server in the foreground. No ROS publisher or robot actuation.
set -Eeuo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/../../src/common/profile.sh"
profile_args "$@"
set -- "${PI05_PROFILE_ARGS[@]}"

task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
task_mode=--check
task_model=
task_port=$PI05_INFERENCE_PORT
task_gpu=0
task_host=127.0.0.1
task_guard="$task_root/tools/vr_control_guard.py"

usage() {
  echo 'Usage: start_inference.sh [--check|--start|--status] --model {cz|kd12|lj|lp} [--port 8000] [--gpu 0] [--host 127.0.0.1|192.168.26.20]' >&2
}
while test "$#" -gt 0; do
  case "$1" in
    --check|--start|--status) task_mode=$1; shift ;;
    --model|--port|--gpu|--host)
      test "$#" -ge 2 || { usage; exit 2; }
      case "$1" in
        --model) task_model=$2 ;;
        --port) task_port=$2 ;;
        --gpu) task_gpu=$2 ;;
        --host) task_host=$2 ;;
      esac
      shift 2 ;;
    *) usage; exit 2 ;;
  esac
done

case "$task_model" in
  cz) task_config=pi05_kuavo_cz_fsdp8; task_asset=kuavo_cz ;;
  kd12) task_config=pi05_kuavo_kd_fsdp8; task_asset=kuavo_kd ;;
  lj) task_config=pi05_kuavo_lj_fsdp8; task_asset=kuavo_lj ;;
  lp) task_config=pi05_kuavo_lp_fsdp8; task_asset=kuavo_lp ;;
  *) usage; exit 2 ;;
esac
[[ "$task_port" =~ ^[0-9]+$ ]] && (( task_port >= 1024 && task_port <= 65535 )) || {
  echo 'Port must be between 1024 and 65535' >&2; exit 2;
}
[[ "$task_gpu" =~ ^[0-9]+$ ]] || { echo 'GPU must be a numeric device index' >&2; exit 2; }
case "$task_host" in
  127.0.0.1) ;;
  "$PI05_WORKSTATION_HOST")
    ip -4 addr show | grep -F "inet $PI05_WORKSTATION_HOST/" >/dev/null || {
      echo 'Robot LAN address 192.168.26.20 is not assigned' >&2; exit 2;
    }
    echo 'WARNING: OpenPI WebSocket has no authentication; use only on the isolated robot LAN.' >&2 ;;
  *) echo 'Host must be loopback or Qi robot LAN address' >&2; exit 2 ;;
esac

task_model_exports=$(python3 "$PI05_COMMON_DIR/robot_profile.py" --model-shell --model "$task_model" --project-root "$task_root/..")
eval "$task_model_exports"
task_config=$PI05_MODEL_CONFIG
task_asset=$PI05_MODEL_ASSET
task_checkpoint=$PI05_MODEL_CHECKPOINT
test -s "$task_checkpoint/params/manifest.ocdbt" || {
  echo "Missing checkpoint manifest: $task_checkpoint" >&2; exit 2;
}
test -s "$task_checkpoint/assets/$task_asset/norm_stats.json" || {
  echo "Missing normalization stats: $task_checkpoint/assets/$task_asset" >&2; exit 2;
}
test -s "$task_root/tools/serve_inference.py" || { echo 'Missing inference server' >&2; exit 2; }
test -s "$task_guard" || { echo 'Missing VR control guard' >&2; exit 2; }

if test "$task_mode" = --status; then
  ss -ltnp "( sport = :$task_port )"
  exit 0
fi

if test "$task_mode" = --start; then
  # Do this before JAX's GPU probe: a second probe can OOM when the first
  # instance has already reserved the device. A foreign listener is not success.
  task_listener=$(ss -ltnpH "( sport = :$task_port )")
  if test -n "$task_listener"; then
    task_pid=$(sed -nE 's/.*pid=([0-9]+).*/\1/p' <<< "$task_listener" | head -n 1)
    task_address=$(awk 'NR==1 {print $4}' <<< "$task_listener")
    task_same_instance=false
    if test "$(wc -l <<< "$task_listener")" -eq 1 && \
       [[ "$task_pid" =~ ^[0-9]+$ ]] && test -r "/proc/$task_pid/cmdline"; then
      mapfile -d '' -t task_cmdline < "/proc/$task_pid/cmdline"
      task_expected=("$task_root/envs/pi05/bin/python" -u "$task_root/tools/serve_inference.py"
                     --config "$task_config" --checkpoint "$task_checkpoint"
                     --port "$task_port" --host "$task_host" --robot-profile "$PI05_ROBOT_PROFILE" --profile-hash "$PI05_PROFILE_HASH")
      if test "$task_address" = "$task_host:$task_port" &&
         test "${#task_cmdline[@]}" -eq "${#task_expected[@]}"; then
        task_same_instance=true
        for ((task_i=0; task_i<${#task_expected[@]}; task_i++)); do
          if test "${task_cmdline[task_i]}" != "${task_expected[task_i]}"; then
            task_same_instance=false
            break
          fi
        done
      fi
      if test "$task_same_instance" = true; then
        task_actual_gpu=$(tr '\0' '\n' < "/proc/$task_pid/environ" |
                          sed -n 's/^CUDA_VISIBLE_DEVICES=//p' | head -n 1)
        test "$task_actual_gpu" = "$task_gpu" || task_same_instance=false
      fi
      if test "$task_same_instance" = true; then
        task_parent=$(ps -o ppid= -p "$task_pid" | tr -d '[:space:]')
        task_guard_found=false
        task_expected_guard=("$task_root/envs/pi05/bin/python" -u "$task_guard" --watch --robot-profile "$PI05_ROBOT_PROFILE" --profile-hash "$PI05_PROFILE_HASH")
        for task_peer in $(pgrep -P "$task_parent" || true); do
          test -r "/proc/$task_peer/cmdline" || continue
          mapfile -d '' -t task_peer_cmdline < "/proc/$task_peer/cmdline"
          if test "${#task_peer_cmdline[@]}" -eq "${#task_expected_guard[@]}"; then
            task_guard_found=true
            for ((task_i=0; task_i<${#task_expected_guard[@]}; task_i++)); do
              if test "${task_peer_cmdline[task_i]}" != "${task_expected_guard[task_i]}"; then
                task_guard_found=false
                break
              fi
            done
            test "$task_guard_found" = false || break
          fi
        done
        test "$task_guard_found" = true || task_same_instance=false
      fi
    fi
    if test "$task_same_instance" = true; then
      echo "PI05_INFERENCE_ALREADY_RUNNING model=$task_model pid=$task_pid host=$task_host port=$task_port gpu=$task_gpu (no new process started)"
      exit 0
    fi
    echo "Port $task_port is occupied by another or unverified process (pid=${task_pid:-unknown}); leaving it untouched" >&2
    exit 3
  fi
fi

"$task_root/envs/pi05/bin/python" - <<'PY'
import ctypes

try:
    driver = ctypes.CDLL('libcuda.so.1')
    status = driver.cuInit(0)
except OSError as exc:
    raise SystemExit(f'PI05_CUDA_DRIVER_UNAVAILABLE: {exc}')
if status != 0:
    raise SystemExit(f'PI05_CUDA_DRIVER_UNAVAILABLE: cuInit={status}; check the workstation GPU driver before retrying')
PY

# Existing GPU services can make JAX's default 75% preallocation fail even
# when the model itself would fit in the remaining memory.
export XLA_PYTHON_CLIENT_PREALLOCATE=false
bash "$task_root/tools/install_env.sh" --check
echo "PI05_INFERENCE_CHECK_OK model=$task_model config=$task_config checkpoint=$task_checkpoint"
if test "$task_mode" = --check; then exit 0; fi

if ss -ltnH "( sport = :$task_port )" | grep -q .; then
  echo "Port $task_port is already in use; leaving the existing service untouched" >&2
  exit 3
fi

# The vendor launcher registers a latched arm-target publisher even when VR is
# idle. Shut down that exact node before exposing inference, then keep checking
# for a controller restart. The lower bridge independently rejects competitors.
"$task_root/envs/pi05/bin/python" -u "$task_guard" --once --robot-profile "$PI05_ROBOT_PROFILE" --profile-hash "$PI05_PROFILE_HASH"

mkdir -p "$task_root/envs/tmp" "$task_root/envs/cache/jax"
export TMPDIR="$task_root/envs/tmp"
export XDG_CACHE_HOME="$task_root/envs/cache"
export OPENPI_DATA_HOME="$task_root/envs/cache/openpi"
export JAX_COMPILATION_CACHE_DIR="$task_root/envs/cache/jax"
export CUDA_VISIBLE_DEVICES="$task_gpu"
export JAX_PLATFORMS=cuda
export PYTHONUNBUFFERED=1
cd "$task_root/openpi"
task_server_pid=
task_guard_pid=
cleanup() {
  trap - EXIT INT TERM
  if test -n "$task_server_pid"; then kill "$task_server_pid" 2>/dev/null || true; fi
  if test -n "$task_guard_pid"; then kill "$task_guard_pid" 2>/dev/null || true; fi
  if test -n "$task_server_pid"; then wait "$task_server_pid" 2>/dev/null || true; fi
  if test -n "$task_guard_pid"; then wait "$task_guard_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
"$task_root/envs/pi05/bin/python" -u "$task_guard" --watch --robot-profile "$PI05_ROBOT_PROFILE" --profile-hash "$PI05_PROFILE_HASH" &
task_guard_pid=$!
"$task_root/envs/pi05/bin/python" -u "$task_root/tools/serve_inference.py" \
  --config "$task_config" --checkpoint "$task_checkpoint" \
  --port "$task_port" --host "$task_host" --robot-profile "$PI05_ROBOT_PROFILE" --profile-hash "$PI05_PROFILE_HASH" &
task_server_pid=$!

# Either a dead model server or a lost VR guard ends this inference session.
if wait -n "$task_server_pid" "$task_guard_pid"; then
  echo 'PI05 inference or VR guard exited; stopping the remaining process' >&2
else
  echo 'PI05 inference or VR guard failed; stopping the remaining process' >&2
fi
exit 1
