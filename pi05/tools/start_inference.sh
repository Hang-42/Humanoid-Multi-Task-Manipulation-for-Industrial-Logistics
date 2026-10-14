#!/usr/bin/env bash
# Start one OpenPI policy server in the foreground. No ROS publisher or robot actuation.
set -Eeuo pipefail

task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
task_mode=--check
task_model=
task_port=8000
task_gpu=0
task_host=127.0.0.1

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
  192.168.26.20)
    ip -4 addr show | grep -F 'inet 192.168.26.20/' >/dev/null || {
      echo 'Robot LAN address 192.168.26.20 is not assigned' >&2; exit 2;
    }
    echo 'WARNING: OpenPI WebSocket has no authentication; use only on the isolated robot LAN.' >&2 ;;
  *) echo 'Host must be loopback or Qi robot LAN address' >&2; exit 2 ;;
esac

task_checkpoint="$task_root/models/$task_model/19999"
test -s "$task_checkpoint/params/manifest.ocdbt" || {
  echo "Missing checkpoint manifest: $task_checkpoint" >&2; exit 2;
}
test -s "$task_checkpoint/assets/$task_asset/norm_stats.json" || {
  echo "Missing normalization stats: $task_checkpoint/assets/$task_asset" >&2; exit 2;
}
test -s "$task_root/tools/serve_inference.py" || { echo 'Missing inference server' >&2; exit 2; }

if test "$task_mode" = --status; then
  ss -ltnp "( sport = :$task_port )"
  exit 0
fi

bash "$task_root/tools/install_env.sh" --check
echo "PI05_INFERENCE_CHECK_OK model=$task_model config=$task_config checkpoint=$task_checkpoint"
if test "$task_mode" = --check; then exit 0; fi

if ss -ltnH "( sport = :$task_port )" | grep -q .; then
  echo "Port $task_port is already in use; leaving the existing service untouched" >&2
  exit 3
fi

mkdir -p "$task_root/envs/tmp" "$task_root/envs/cache/jax"
export TMPDIR="$task_root/envs/tmp"
export XDG_CACHE_HOME="$task_root/envs/cache"
export OPENPI_DATA_HOME="$task_root/envs/cache/openpi"
export JAX_COMPILATION_CACHE_DIR="$task_root/envs/cache/jax"
export CUDA_VISIBLE_DEVICES="$task_gpu"
export JAX_PLATFORMS=cuda
export PYTHONUNBUFFERED=1
cd "$task_root/openpi"
exec "$task_root/envs/pi05/bin/python" -u "$task_root/tools/serve_inference.py" \
  --config "$task_config" --checkpoint "$task_checkpoint" \
  --port "$task_port" --host "$task_host"
