#!/usr/bin/env bash
# Install or validate the OpenPI inference environment under ../envs/pi05.
set -Eeuo pipefail

task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
task_openpi="$task_root/openpi"
task_python="$task_root/envs/pi05/bin/python"

task_require_writable_dir() {
  local task_directory=$1 task_probe
  if mkdir -p -- "$task_directory" &&
     task_probe=$(mktemp "$task_directory/.pi05-write-check.XXXXXX"); then
    rm -f -- "$task_probe"
    return 0
  fi
  printf 'PI05_ENV_NOT_WRITABLE directory=%s user=%s\n' "$task_directory" "$(id -un)" >&2
  echo 'Repair the local runtime directory, then rerun the installer as your normal user:' >&2
  printf '  sudo mkdir -p -- %q\n' "$task_root/envs" >&2
  printf '  sudo chown -R -- %q %q\n' "$(id -un):$(id -gn)" "$task_root/envs" >&2
  printf '  sudo chmod -R u+rwX -- %q\n' "$task_root/envs" >&2
  return 2
}

case "${1:---check}" in
  --check) ;;
  --install)
    test -s "$task_openpi/setup_kuavo_inference.sh" || {
      echo "Missing $task_openpi/setup_kuavo_inference.sh" >&2; exit 2;
    }
    # Check actual writes before uv starts; copied runtime trees may belong
    # to another user even when the repository itself is writable.
    for task_directory in "$task_root/envs" "$task_root/envs/cache/uv" \
                          "$task_root/envs/cache/openpi" "$task_root/envs/cache/jax" \
                          "$task_root/envs/tmp"; do
      task_require_writable_dir "$task_directory"
    done
    if test -e "$task_root/envs/pi05"; then
      task_require_writable_dir "$task_root/envs/pi05"
    fi
    # The upstream installer derives its environment prefix from its own path.
    bash "$task_openpi/setup_kuavo_inference.sh" ;;
  *) echo 'Usage: install_env.sh [--check|--install]' >&2; exit 2 ;;
esac

test -x "$task_python" || { echo "Missing $task_python; run --install" >&2; exit 2; }
test -s "$task_openpi/requirements-kuavo-inference.lock.txt" || {
  echo 'Missing OpenPI inference lockfile' >&2; exit 2;
}
test -s "$task_root/envs/cache/openpi/big_vision/paligemma_tokenizer.model" || {
  echo 'Missing local PaliGemma tokenizer asset' >&2; exit 2;
}

export PYTHONPATH="$task_openpi${PYTHONPATH:+:$PYTHONPATH}"
"$task_python" -m pip check
"$task_python" - <<'PY'
import pyarrow
import jax
import flax
import orbax.checkpoint
import openpi
import openpi_client
import sys
from openpi.training.config import get_config

for name in (
    'pi05_kuavo_cz_fsdp8',
    'pi05_kuavo_kd_fsdp8',
    'pi05_kuavo_lj_fsdp8',
    'pi05_kuavo_lp_fsdp8',
):
    get_config(name)
print(f'PI05_ENV_OK python={sys.version_info.major}.{sys.version_info.minor} jax={jax.__version__} backend={jax.default_backend()}')
PY
