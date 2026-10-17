#!/usr/bin/env bash
# Qi entry for the camera stack, OpenPI inference, and guarded live bridge.
set -Eeuo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common/profile.sh"
profile_args "$@"
set -- "${PI05_PROFILE_ARGS[@]}"
if test "${1:-}" = --profile-check; then
  python3 "$PI05_COMMON_DIR/robot_profile.py"
  exit 0
fi

task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
task_upper="$task_dir/upper"
task_host=${PI05_UPPER_HOST:-leju_kuavo@192.168.26.12}
task_session=pi05-upper-camera
if test "$PI05_ROBOT_ID" != v53_qi; then task_session="pi05-$PI05_ROBOT_ID-upper-camera"; fi
task_inference="$task_dir/../pi05/tools/start_inference.sh"
task_proxy="$task_dir/../pi05/tools/model_proxy.py"
task_lower="$task_dir/lower"

# Inference is explicit: it runs in the foreground and never publishes robot actions.
case "${1:-}" in
  --pi05-live|--pi05-query)
    task_query_only=false
    if test "$1" = --pi05-query; then task_query_only=true; fi
    shift
    task_model=kd12
    task_steps=2
    task_gpu=0
    task_bridge_args=()
    task_execute_args=()
    task_retime=false
    task_sync_grasp=false
    task_single_step=false
    task_approach_only=false
    task_extra_value=0
    task_lookahead_value=0
    task_run_description='推理将持续运行，直到你按 Ctrl+C。'
    while test "$#" -gt 0; do
      case "$1" in
        --model)
          test "$#" -ge 2 || { echo '--model needs a value' >&2; exit 2; }
          task_model=$2; shift 2 ;;
        --steps)
          test "$#" -ge 2 || { echo '--steps needs a value' >&2; exit 2; }
          task_steps=$2; shift 2 ;;
        --gpu)
          test "$#" -ge 2 && [[ "$2" =~ ^[0-9]+$ ]] || { echo '--gpu needs a numeric device index' >&2; exit 2; }
          task_gpu=$2; shift 2 ;;
        --retime|--resume-current|--sync-grasp)
          if test "$1" = --retime; then task_retime=true; fi
          if test "$1" = --sync-grasp; then task_sync_grasp=true; fi
          task_bridge_args+=("$1"); shift ;;
        --single-step|--approach-only)
          if test "$1" = --single-step; then task_single_step=true; else task_approach_only=true; fi
          task_execute_args+=("$1"); shift ;;
        --grasp-lookahead)
          test "$#" -ge 2 && [[ "$2" =~ ^([0-9]|10)$ ]] || { echo '--grasp-lookahead must be 0..10' >&2; exit 2; }
          task_lookahead_value=$2
          task_bridge_args+=("$1" "$2"); shift 2 ;;
        --duration|--approach-extra-steps)
          test "$#" -ge 2 && [[ "$2" =~ ^[0-9]+$ ]] || { echo "$1 needs an integer" >&2; exit 2; }
          if test "$1" = --duration; then
            [[ "$2" =~ ^[1-9][0-9]{0,2}$ ]] && (( 10#$2 <= 600 )) || { echo '--duration must be 1..600' >&2; exit 2; }
            task_run_description="本轮限时 $2 秒，保护触发或 Ctrl+C 将提前停止。"
          else
            [[ "$2" =~ ^([0-9]|10)$ ]] || { echo '--approach-extra-steps must be 0..10' >&2; exit 2; }
            task_extra_value=$2
          fi
          task_execute_args+=("$1" "$2"); shift 2 ;;
        *) echo 'Usage: ./deploy_pi05.sh --pi05-live [--model {kd12|cz|lj|lp}] [--steps 1..50] [--sync-grasp] [--grasp-lookahead 0..10] [--duration 1..600]' >&2; exit 2 ;;
      esac
    done
    case "$task_model" in kd12|cz|lj|lp) ;; *) echo 'Unknown PI05 model' >&2; exit 2 ;; esac
    [[ "$task_steps" =~ ^([1-9]|[1-4][0-9]|50)$ ]] || {
      echo '--steps must be an integer from 1 to 50' >&2; exit 2;
    }
    if test "$task_single_step" = true && { test "$task_retime" = true || test "$task_approach_only" = true || test "$task_extra_value" != 0; }; then
      echo '--single-step cannot combine with retime/approach options' >&2; exit 2
    fi
    if test "$task_approach_only" = true && test "$task_retime" != true; then
      echo '--approach-only requires --retime' >&2; exit 2
    fi
    if test "$task_extra_value" != 0 && test "$task_approach_only" != true; then
      echo '--approach-extra-steps requires --approach-only' >&2; exit 2
    fi
    if test "$task_sync_grasp" = true && { test "$task_retime" = true || test "$task_single_step" = true || test "$task_approach_only" = true; }; then
      echo '--sync-grasp cannot combine with retime/single-step/approach-only' >&2; exit 2
    fi
    if test "$task_lookahead_value" != 0 && test "$task_sync_grasp" != true; then
      echo '--grasp-lookahead requires --sync-grasp' >&2; exit 2
    fi
    python3 "$PI05_COMMON_DIR/robot_profile.py" --model "$task_model"
    if test "$task_query_only" = true && test "${#task_execute_args[@]}" -gt 0; then
      echo '--pi05-query does not accept motion-only options' >&2; exit 2
    fi
    task_bridge_quoted=
    task_execute_quoted=
    if test "${#task_bridge_args[@]}" -gt 0; then printf -v task_bridge_quoted ' %q' "${task_bridge_args[@]}"; fi
    if test "${#task_execute_args[@]}" -gt 0; then printf -v task_execute_quoted ' %q' "${task_execute_args[@]}"; fi
    if test "$task_query_only" != true && { ! test -t 0 || ! test -r /dev/tty; }; then
      echo 'Live robot control requires an interactive terminal' >&2; exit 2;
    fi
    task_lower_host=${PI05_LOWER_HOST:-lab@192.168.26.1}
    [[ "$task_host" =~ ^[A-Za-z0-9_.@-]+$ && "$task_lower_host" =~ ^[A-Za-z0-9_.@-]+$ ]] || {
      echo 'Invalid upper or lower SSH host' >&2; exit 2;
    }
    ip -4 addr show | grep -q "inet $PI05_WORKSTATION_HOST/" || {
      echo 'Qi robot LAN address is missing' >&2; exit 2;
    }
    for task_peer in "$PI05_UPPER_IP" "$LOWER_ROS_IP"; do
      ip -4 route get "$task_peer" | grep -q "src $PI05_WORKSTATION_HOST" || {
        echo "Robot LAN route is missing for $task_peer" >&2; exit 2;
      }
    done
    task_ssh=(-o BatchMode=yes -o ConnectTimeout=12 -o StrictHostKeyChecking=yes)
    task_inference_session="pi05-$PI05_ROBOT_ID-$task_model-inference"
    task_proxy_session="pi05-$PI05_ROBOT_ID-$task_model-proxy"

    # Reuse the exact release only when its ROS registration and all three
    # live camera topics are healthy. A process can survive a ROS master
    # restart while its old registrations are gone, so the session itself is
    # not sufficient evidence that the camera relay is usable.
    task_upper_version=$( { (cd "$task_upper" && sha256sum upper_start.sh start_camera.launch camera_relay.py camera_codec.py package.xml CMakeLists.txt); profile_bundle_digest; } | sha256sum | cut -c1-16)
    task_upper_release="$PI05_UPPER_ROOT/releases/$task_upper_version/upper"
    if ssh "${task_ssh[@]}" "$task_host" "tmux has-session -t '$task_session' 2>/dev/null"; then
      task_upper_state=$(ssh "${task_ssh[@]}" "$task_host" "tmux display-message -p -t '$task_session' '#{pane_dead}|#{pane_current_path}'")
      if test "$task_upper_state" != "0|$task_upper_release"; then
        if [[ "$task_upper_state" =~ ^[01]\|$PI05_UPPER_ROOT/releases/[a-f0-9]{16}/upper$ ]]; then
          task_old_upper_release=${task_upper_state#*|}
          echo "PI05_UPPER_OLD_RELEASE session=$task_session release=${task_old_upper_release##*/releases/}; restarting the managed camera release" >&2
          ssh "${task_ssh[@]}" "$task_host" "tmux send-keys -t '$task_session:0' C-c"
          task_deadline=$(( $(date +%s) + 30 ))
          while true; do
            task_session_gone=false
            if ! ssh "${task_ssh[@]}" "$task_host" "tmux has-session -t '$task_session' 2>/dev/null"; then
              task_session_gone=true
            fi
            if test "$task_session_gone" = true &&
               ssh "${task_ssh[@]}" "$task_host" "bash '$task_old_upper_release/upper_start.sh' --preflight" >/dev/null 2>&1; then
              break
            fi
            (( $(date +%s) < task_deadline )) || {
              echo 'Old camera session did not stop cleanly; inspect the upper computer before retrying.' >&2
              exit 3
            }
            sleep 1
          done
          bash "$task_dir/deploy_pi05.sh" --robot-profile "$PI05_ROBOT_PROFILE" --start
        else
          echo "Existing camera session is not a managed PI05 release: $task_upper_state" >&2; exit 3
        fi
      elif ssh "${task_ssh[@]}" "$task_host" "bash '$task_upper_release/upper_start.sh' --health"; then
        echo "PI05_UPPER_ALREADY_RUNNING session=$task_session release=$task_upper_version"
      else
        echo "PI05_UPPER_STALE session=$task_session; restarting the managed camera release" >&2
        ssh "${task_ssh[@]}" "$task_host" "tmux send-keys -t '$task_session:0' C-c"
        task_deadline=$(( $(date +%s) + 30 ))
        while true; do
          task_session_gone=false
          if ! ssh "${task_ssh[@]}" "$task_host" "tmux has-session -t '$task_session' 2>/dev/null"; then
            task_session_gone=true
          fi
          if test "$task_session_gone" = true &&
             ssh "${task_ssh[@]}" "$task_host" "bash '$task_upper_release/upper_start.sh' --preflight" >/dev/null 2>&1; then
            break
          fi
          (( $(date +%s) < task_deadline )) || {
            echo 'Stale camera session did not stop cleanly; inspect the upper computer before retrying.' >&2
            exit 3
          }
          sleep 1
        done
        bash "$task_dir/deploy_pi05.sh" --robot-profile "$PI05_ROBOT_PROFILE" --start
      fi
    else
      bash "$task_dir/deploy_pi05.sh" --robot-profile "$PI05_ROBOT_PROFILE" --start
    fi

    # Install bridge files without starting a robot process. The reported path
    # is bound to the checksummed release rather than a mutable 'current' link.
    task_install_output=$(bash "$task_dir/deploy_pi05.sh" --robot-profile "$PI05_ROBOT_PROFILE" --lower-install)
    printf '%s\n' "$task_install_output"
    task_lower_release=${task_install_output##*PI05_LOWER_INSTALLED }
    task_lower_release=${task_lower_release%% *}
    [[ "$task_lower_release" =~ ^$PI05_LOWER_ROOT/releases/[a-f0-9]{16}/lower$ ]] || {
      echo 'Could not identify the installed lower release' >&2; exit 3;
    }

    # Check an existing model server before any GPU initialization. Otherwise
    # keep the new server in its own tmux session for later requests.
    if ss -ltnH "( sport = :$PI05_INFERENCE_PORT )" | grep -q .; then
      bash "$task_dir/deploy_pi05.sh" --robot-profile "$PI05_ROBOT_PROFILE" --pi05-start --model "$task_model" --gpu "$task_gpu"
    else
      if tmux has-session -t "=$task_inference_session" 2>/dev/null; then
        echo "Inference session exists without a listener: $task_inference_session" >&2; exit 3;
      fi
      task_log_dir="$task_dir/../pi05/envs/logs"
      mkdir -p -- "$task_log_dir"
      task_inference_log="$task_log_dir/$task_inference_session-$(date +%Y%m%d-%H%M%S).log"
      tmux new-session -d -s "$task_inference_session" -c "$task_dir/.." \
        "env PI05_WORKSTATION_IP='$PI05_WORKSTATION_HOST' bash '$task_dir/deploy_pi05.sh' --robot-profile '$PI05_ROBOT_PROFILE' --pi05-start --model '$task_model' --gpu '$task_gpu' > '$task_inference_log' 2>&1"
      task_deadline=$(( $(date +%s) + 180 ))
      until ss -ltnH "( sport = :$PI05_INFERENCE_PORT )" | grep -q .; do
        tmux has-session -t "=$task_inference_session" 2>/dev/null &&
          (( $(date +%s) < task_deadline )) || {
          echo "Inference did not become ready; log=$task_inference_log" >&2
          tail -n 35 -- "$task_inference_log" >&2 || true
          exit 3;
        }
        sleep 1
      done
      echo "PI05_INFERENCE_LOG $task_inference_log"
      bash "$task_dir/deploy_pi05.sh" --robot-profile "$PI05_ROBOT_PROFILE" --pi05-start --model "$task_model" --gpu "$task_gpu"
    fi

    # The proxy is a separate process. An occupied port is acceptable only
    # when its exact program and model match this request.
    if ! ss -ltnH "( sport = :$PI05_WORKSTATION_PORT )" | grep -q .; then
      if tmux has-session -t "=$task_proxy_session" 2>/dev/null; then
        echo "Proxy session exists without a listener: $task_proxy_session" >&2; exit 3;
      fi
      tmux new-session -d -s "$task_proxy_session" -c "$task_dir/.." \
        "env PI05_WORKSTATION_IP='$PI05_WORKSTATION_HOST' bash '$task_dir/deploy_pi05.sh' --robot-profile '$PI05_ROBOT_PROFILE' --pi05-proxy-start --model '$task_model'"
      task_deadline=$(( $(date +%s) + 180 ))
      until ss -ltnH "( sport = :$PI05_WORKSTATION_PORT )" | grep -q .; do
        tmux has-session -t "=$task_proxy_session" 2>/dev/null &&
          (( $(date +%s) < task_deadline )) || {
          echo "Proxy did not become ready; inspect tmux $task_proxy_session" >&2; exit 3;
        }
        sleep 1
      done
    fi
    task_proxy_listener=$(ss -ltnpH "( sport = :$PI05_WORKSTATION_PORT )")
    test "$(wc -l <<< "$task_proxy_listener")" -eq 1 || {
      echo 'Ambiguous proxy listener' >&2; exit 3;
    }
    task_proxy_pid=$(sed -nE 's/.*pid=([0-9]+).*/\1/p' <<< "$task_proxy_listener")
    task_proxy_bind=$(awk '{print $4}' <<< "$task_proxy_listener")
    [[ "$task_proxy_pid" =~ ^[0-9]+$ ]] &&
      test "$task_proxy_bind" = "$PI05_WORKSTATION_HOST:$PI05_WORKSTATION_PORT" &&
      test -r "/proc/$task_proxy_pid/cmdline" || {
      echo 'Proxy listener identity cannot be verified' >&2; exit 3;
    }
    mapfile -d '' -t task_proxy_argv < "/proc/$task_proxy_pid/cmdline"
    task_proxy_script=false
    task_proxy_model=false
    task_proxy_start=false
    task_proxy_profile=false
    for ((task_i=0; task_i<${#task_proxy_argv[@]}; task_i++)); do
      if test "${task_proxy_argv[task_i]}" = "$task_proxy" ||
         { test -f "${task_proxy_argv[task_i]}" &&
           test "$(readlink -f -- "${task_proxy_argv[task_i]}")" = "$(readlink -f -- "$task_proxy")"; }; then
        task_proxy_script=true
      fi
      if test "${task_proxy_argv[task_i]}" = --start; then
        task_proxy_start=true
      fi
      if test "${task_proxy_argv[task_i]}" = --profile-hash &&
         (( task_i + 1 < ${#task_proxy_argv[@]} )) &&
         test "${task_proxy_argv[task_i+1]}" = "$PI05_PROFILE_HASH"; then
        task_proxy_profile=true
      fi
      if test "${task_proxy_argv[task_i]}" = --model &&
         (( task_i + 1 < ${#task_proxy_argv[@]} )) &&
         test "${task_proxy_argv[task_i+1]}" = "$task_model"; then
        task_proxy_model=true
      fi
    done
    test "$task_proxy_script" = true && test "$task_proxy_model" = true &&
      test "$task_proxy_start" = true && test "$task_proxy_profile" = true || {
      echo 'Port 19092 belongs to a different or unverified model proxy' >&2; exit 3;
    }
    echo "PI05_PROXY_READY model=$task_model pid=$task_proxy_pid"

    # This query checks fresh observations, initialization and controller
    # ownership, but sends no robot command. Do not bypass a failed check.
    ssh "${task_ssh[@]}" "$task_lower_host" \
      "sudo -n bash '$task_lower_release/lower_start.sh' --robot-profile '$task_lower_release/common/robot_profile.json' --model '$task_model' --steps '$task_steps'$task_bridge_quoted"
    if test "$task_query_only" = true; then
      echo "PI05_QUERY_COMPLETED robot=$PI05_ROBOT_ID model=$task_model (no robot action sent)"
      exit 0
    fi
    printf '%s\n' \
      "就绪：模型=$task_model；$task_run_description" \
      '请现场确认机器人稳定站立、有人持续值守、急停可立即操作，运动范围内无人和障碍。' \
      '同意开始动作请输入 o 并按 Enter；其他输入取消。' >&2
    task_confirmation=
    IFS= read -r task_confirmation </dev/tty || exit 3
    test "$task_confirmation" = o || { echo '未确认，未启动机器人动作' >&2; exit 3; }
    exec ssh -tt "${task_ssh[@]}" "$task_lower_host" \
      "sudo -n bash '$task_lower_release/lower_start.sh' --robot-profile '$task_lower_release/common/robot_profile.json' --model '$task_model' --execute --steps '$task_steps'$task_bridge_quoted$task_execute_quoted"
    ;;
  --lower-install)
    test "$#" -eq 1 || { echo 'Usage: ./deploy_pi05.sh --lower-install' >&2; exit 2; }
    task_lower_host=${PI05_LOWER_HOST:-lab@192.168.26.1}
    [[ "$task_lower_host" =~ ^[A-Za-z0-9_.@-]+$ ]] || { echo 'Invalid PI05_LOWER_HOST' >&2; exit 2; }
    task_files=(lower_start.sh robot_bridge.py grasp_sync.py camera_observations.py camera_codec.py
                observation.py protocol.py action_safety.py arm_ack.py package.xml CMakeLists.txt)
    for task_file in "${task_files[@]}"; do
      test -s "$task_lower/$task_file" || { echo "Missing $task_lower/$task_file" >&2; exit 2; }
    done
    bash -n "$task_lower/lower_start.sh"
    python3 -m py_compile "$task_lower/robot_bridge.py" "$task_lower/camera_observations.py" \
      "$task_lower/action_safety.py" "$task_lower/protocol.py"
    task_version=$( { (cd "$task_lower" && sha256sum "${task_files[@]}"); profile_bundle_digest; } | sha256sum | cut -c1-16)
    task_remote="$PI05_LOWER_ROOT/releases/$task_version/lower"
    task_ssh=(-o BatchMode=yes -o ConnectTimeout=12 -o StrictHostKeyChecking=yes)
    ssh "${task_ssh[@]}" "$task_lower_host" "install -d -- '$task_remote'"
    task_sources=()
    for task_file in "${task_files[@]}"; do task_sources+=("$task_lower/$task_file"); done
    scp "${task_ssh[@]}" "${task_sources[@]}" "$task_lower_host:$task_remote/"
    profile_install_common "$task_lower_host" "$task_remote"
    task_local_hashes=$(cd "$task_lower" && sha256sum "${task_files[@]}")
    task_remote_hashes=$(ssh "${task_ssh[@]}" "$task_lower_host" \
      "cd '$task_remote' && sha256sum ${task_files[*]}")
    test "$task_local_hashes" = "$task_remote_hashes" || { echo 'Lower bridge transfer checksum mismatch' >&2; exit 3; }
    ssh "${task_ssh[@]}" "$task_lower_host" \
      "bash -n '$task_remote/lower_start.sh' && python3 -m py_compile '$task_remote/robot_bridge.py'"
    echo "PI05_LOWER_INSTALLED $task_remote (no robot process started)"
    exit 0 ;;
  --pi05-proxy-check|--pi05-proxy-self-test|--pi05-proxy-start)
    task_mode=${1#--pi05-proxy-}
    shift
    exec "$task_dir/../pi05/envs/pi05/bin/python" -u "$task_proxy" --robot-profile "$PI05_ROBOT_PROFILE" --profile-hash "$PI05_PROFILE_HASH" "--$task_mode" "$@" ;;
  --pi05-check|--pi05-start|--pi05-status)
    test "$#" -ge 3 || {
      echo 'Usage: ./deploy_pi05.sh --pi05-{check|start|status} --model {cz|kd12|lj|lp} [--port N] [--gpu N] [--host IP]' >&2
      exit 2
    }
    task_mode=${1#--pi05-}
    shift
    exec bash "$task_inference" --robot-profile "$PI05_ROBOT_PROFILE" "--$task_mode" "$@" ;;
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
    python3 -m py_compile "$task_upper/camera_codec.py" "$task_upper/camera_relay.py" \
      "$task_proxy" "$task_dir/../pi05/tools/serve_inference.py" \
      "$task_dir/../pi05/tools/vr_control_guard.py"
    exit 0 ;;
  *) echo 'Usage: ./deploy_pi05.sh [--check|--install|--start|--lower-install|--pi05-install|--pi05-check|--pi05-start|--pi05-status|--pi05-proxy-check|--pi05-proxy-self-test|--pi05-proxy-start|--pi05-live]' >&2; exit 2 ;;
esac

for task_file in upper_start.sh start_camera.launch camera_relay.py camera_codec.py package.xml CMakeLists.txt; do
  test -s "$task_upper/$task_file" || { echo "Missing $task_upper/$task_file" >&2; exit 2; }
done
[[ "$task_host" =~ ^[A-Za-z0-9_.@-]+$ ]] || { echo 'Invalid PI05_UPPER_HOST' >&2; exit 2; }
ip -4 addr show | grep -q "inet $PI05_WORKSTATION_HOST/" || { echo 'Qi robot LAN address is missing' >&2; exit 2; }
ip -4 route get "$PI05_UPPER_IP" | grep -q "src $PI05_WORKSTATION_HOST" || { echo 'Upper computer route is not on robot LAN' >&2; exit 2; }

task_version=$( { (cd "$task_upper" && sha256sum upper_start.sh start_camera.launch camera_relay.py camera_codec.py package.xml CMakeLists.txt); profile_bundle_digest; } | sha256sum | cut -c1-16)
task_release="$PI05_UPPER_ROOT/releases/$task_version"
task_remote="$task_release/upper"
task_ssh=(-o BatchMode=yes -o ConnectTimeout=12 -o StrictHostKeyChecking=yes)

ssh "${task_ssh[@]}" "$task_host" "install -d -- '$task_release'"
scp -r "${task_ssh[@]}" "$task_upper" "$task_host:$task_release/"
profile_install_common "$task_host" "$task_remote"
task_local_hashes=$(cd "$task_upper" && sha256sum upper_start.sh start_camera.launch camera_relay.py camera_codec.py package.xml CMakeLists.txt)
task_remote_hashes=$(ssh "${task_ssh[@]}" "$task_host" "cd '$task_remote' && sha256sum upper_start.sh start_camera.launch camera_relay.py camera_codec.py package.xml CMakeLists.txt")
test "$task_local_hashes" = "$task_remote_hashes" || { echo 'Upper computer transfer checksum mismatch' >&2; exit 3; }
ssh "${task_ssh[@]}" "$task_host" "chmod 755 '$task_remote/upper_start.sh' '$task_remote/camera_relay.py' && bash -n '$task_remote/upper_start.sh' && python3 -m py_compile '$task_remote/camera_relay.py' '$task_remote/camera_codec.py'"
echo "PI05_UPPER_INSTALLED $task_remote"
if test "$task_start" != true; then exit 0; fi

# Check the live ROS master and camera-node ownership before creating a session.
ssh "${task_ssh[@]}" "$task_host" "'$task_remote/upper_start.sh' --preflight"
# An existing tmux session belongs to the operator; do not replace it.
if ssh "${task_ssh[@]}" "$task_host" "tmux has-session -t '$task_session' 2>/dev/null"; then
  echo "Session $task_session already exists; inspect it before starting another instance." >&2
  exit 3
fi
ssh "${task_ssh[@]}" "$task_host" "tmux new-session -d -s '$task_session' -c '$task_remote' \"env PI05_WORKSTATION_IP='$PI05_WORKSTATION_HOST' bash '$task_remote/upper_start.sh' --robot-profile '$task_remote/common/robot_profile.json'\""
sleep 3
ssh "${task_ssh[@]}" "$task_host" "test \"\$(tmux display-message -p -t '$task_session' '#{pane_dead}')\" = 0 || { tmux capture-pane -pt '$task_session' -S -35; exit 4; }"
echo "PI05_UPPER_STARTED session=$task_session release=$task_version"
