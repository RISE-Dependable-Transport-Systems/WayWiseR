#!/usr/bin/env bash
# Create a WayWiseR runtime venv from the installed pyproject.toml.
set -euo pipefail

ros_distro="${ROS_DISTRO:-humble}"
venv_dir="${WAYWISER_VENV_PATH:-$HOME/.waywiser/venv}"
pyproject="${WAYWISER_PYPROJECT:-}"
repo_dir="${WAYWISER_REPO_DIR:-}"
skipped_packages="${WAYWISER_SKIPPED_PACKAGES:-}"
explicit_extras="${WAYWISER_PYTHON_EXTRAS:-}"
python_version="${WAYWISER_PYTHON_VERSION:-}"
torch_cpu_index="${WAYWISER_TORCH_CPU_INDEX_URL:-https://download.pytorch.org/whl/cpu}"
OPT_YES=false
print_only=false
editable=false
clear_venv=false
system_site_packages="${WAYWISER_SYSTEM_SITE_PACKAGES:-true}"

usage() {
    cat <<'EOF'
Usage: setup_venv.bash [options]

Options:
  --venv PATH         virtual environment path (default: ~/.waywiser/venv)
  --pyproject PATH    pyproject.toml to install from
  --repo-dir PATH     source repo root; installs editable source checkout
  --skipped-packages  space-separated source packages to skip
  --extras LIST       comma/space-separated pyproject extras to install
  --python VERSION    Python version/path to use when creating the venv
  --editable          install --repo-dir in editable mode
  --clear             recreate the venv
  --system-site-packages
  --no-system-site-packages
  --print-extras      print detected extras and exit
  -y, --yes, --quiet  non-interactive
  -h, --help          show this help

Environment overrides:
  WAYWISER_VENV_PATH
  WAYWISER_PYPROJECT
  WAYWISER_REPO_DIR
  WAYWISER_SKIPPED_PACKAGES
  WAYWISER_PYTHON_EXTRAS
  WAYWISER_PYTHON_VERSION
    WAYWISER_TORCH_CPU_INDEX_URL
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --venv)
            venv_dir="${2:?--venv requires a path}"; shift 2 ;;
        --pyproject)
            pyproject="${2:?--pyproject requires a path}"; shift 2 ;;
        --repo-dir)
            repo_dir="${2:?--repo-dir requires a path}"; shift 2 ;;
        --skipped-packages)
            skipped_packages="${2:-}"; shift 2 ;;
        --extras)
            explicit_extras="${2:-}"; shift 2 ;;
        --python)
            python_version="${2:?--python requires a version or path}"; shift 2 ;;
        --editable)
            editable=true; shift ;;
        --clear)
            clear_venv=true; shift ;;
        --system-site-packages)
            system_site_packages=true; shift ;;
        --no-system-site-packages)
            system_site_packages=false; shift ;;
        --print-extras)
            print_only=true; shift ;;
        --)
            shift ;;
        -y|--yes|--quiet)
            OPT_YES=true; shift ;;
        -h|--help)
            usage; exit 0 ;;
        *)
            echo "ERROR: unknown option: $1" >&2
            usage >&2
            exit 2 ;;
    esac
done

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
prefix_dir="$(cd "$script_dir/../.." 2>/dev/null && pwd || true)"

info() { echo "  $*"; }
warn() { echo "  WARNING: $*" >&2; }
die() { echo "ERROR: $*" >&2; exit 1; }

find_pyproject() {
    local candidate
    for candidate in \
        "$pyproject" \
        "${repo_dir:+$repo_dir/pyproject.toml}" \
        "$prefix_dir/share/waywiser/python/pyproject.toml" \
        "/opt/ros/$ros_distro/share/waywiser/python/pyproject.toml" \
        "$script_dir/../../pyproject.toml" \
        "${WAYWISER_REPO_DIR:-}/pyproject.toml" \
        "$PWD/pyproject.toml"
    do
        [[ -n "$candidate" ]] || continue
        if [[ -f "$candidate" ]]; then
            cd "$(dirname "$candidate")" && printf '%s/%s\n' "$(pwd)" "$(basename "$candidate")"
            return 0
        fi
    done
    return 1
}

has_nvidia_gpu() {
    if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null 2>&1; then
        return 0
    fi
    [[ -e /dev/nvidia0 ]]
}

is_jetson() {
    [[ -f /etc/nv_tegra_release ]] && return 0
    dpkg -s nvidia-l4t-core &>/dev/null && return 0
    if [[ -r /proc/device-tree/model ]] \
        && tr -d '\0' < /proc/device-tree/model 2>/dev/null | grep -qi 'nvidia.*jetson'; then
        return 0
    fi
    return 1
}

append_unique() {
    local current="$1"
    shift
    local item
    for item in "$@"; do
        if [[ " $current " != *" $item "* ]]; then
            current+="${current:+ }$item"
        fi
    done
    printf '%s\n' "$current"
}

remove_path() {
    local path="$1"
    rm -rf "$path" 2>/dev/null && return 0

    if command -v sudo &>/dev/null; then
        warn "Permission denied removing $path; retrying with sudo for files created by Docker/QEMU."
        sudo rm -rf "$path"
        return 0
    fi

    die "Permission denied removing $path. Remove it manually or install sudo."
}

install_activate_hook() {
    local activate_file="$venv_dir/bin/activate"
    local marker_begin="# >>> WayWiseR Environment Setup >>>"
    local marker_end="# <<< WayWiseR Environment Setup <<<"
    local hook_workspace="${WAYWISER_WS:-}"
    local source_workspace_install=true

    [[ -f "$activate_file" ]] || return 0

    if [[ "${WAYWISER_NO_WORKSPACE_VENV_LINK:-}" == "true" ]]; then
        source_workspace_install=false
    fi

    if [[ -z "$hook_workspace" && -n "$repo_dir" ]]; then
        hook_workspace="$(cd "$repo_dir/../.." && pwd)"
    fi

    if grep -qF "$marker_begin" "$activate_file" 2>/dev/null; then
        sed -i "\|$marker_begin|,\|$marker_end|d" "$activate_file"
    elif grep -q "WayWiseR Environment Setup" "$activate_file" 2>/dev/null; then
        sed -i '/# WayWiseR Environment Setup/,$d' "$activate_file"
    fi

    cat >> "$activate_file" <<EOF

$marker_begin
# Load runtime config: prefer source workspace .env, fall back to deb conffile.
if [ -z "\${WAYWISER_WS:-}" ] && [ -n "$hook_workspace" ]; then
    export WAYWISER_WS="$hook_workspace"
fi
if [ -n "\${WAYWISER_WS:-}" ] && [ -f "\$WAYWISER_WS/src/WayWiseR/.env" ]; then
    set -a; source "\$WAYWISER_WS/src/WayWiseR/.env"; set +a
elif [ -f /etc/waywiser/waywiser.env ]; then
    set -a; source /etc/waywiser/waywiser.env; set +a
fi
if [ -z "\${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/$ros_distro/setup.bash ]; then
        set +u; source /opt/ros/$ros_distro/setup.bash; set -u
    else
        echo "WARNING: ROS2 is not sourced. Source your ROS2 environment before using WayWiseR." >&2
    fi
fi
if [ -n "\${VIRTUAL_ENV:-}" ]; then
    _waywiser_venv_site="\$("\$VIRTUAL_ENV/bin/python" -c 'import site; print(site.getsitepackages()[0])' 2>/dev/null || true)"
    if [ -n "\$_waywiser_venv_site" ]; then
        export PYTHONPATH="\$_waywiser_venv_site:\${PYTHONPATH:-}"
    fi
    unset _waywiser_venv_site
fi
if ${source_workspace_install} && [ -n "\${WAYWISER_WS:-}" ] && [ -f "\$WAYWISER_WS/install/setup.bash" ]; then
    set +u; source "\$WAYWISER_WS/install/setup.bash"; set -u
fi
$marker_end
EOF
}

extra_exists() {
    local extra="$1"
    grep -Eq "^[[:space:]]*${extra}[[:space:]]*=" "$pyproject"
}

normalize_extras() {
    local value="$1"
    printf '%s\n' "${value//,/ }" | xargs
}

source_extras() {
    local extras="build px4"

    [[ " $skipped_packages " == *" waywiser_carla "* ]] \
        || extras="$(append_unique "$extras" waywiser_carla)"
    [[ " $skipped_packages " == *" waywiser_hwbringup "* ]] \
        || extras="$(append_unique "$extras" waywiser_hwbringup)"
    if [[ " $skipped_packages " != *" waywiser_perception "* ]]; then
        extras="$(append_unique "$extras" waywiser_perception)"
        if is_jetson; then
            extras="$(append_unique "$extras" waywiser_perception_jetson)"
        elif [[ "$(uname -m)" == "aarch64" ]]; then
            warn "Detected aarch64 without Jetson markers; installing common perception dependencies only."
        else
            extras="$(append_unique "$extras" waywiser_perception_x86)"
            if has_nvidia_gpu; then
                extras="$(append_unique "$extras" waywiser_perception_x86_cuda waywiser_perception_tensorrt)"
            else
                info "No NVIDIA GPU detected; skipping CUDA-specific perception extras and using CPU-only torch/torchvision." >&2
            fi
        fi
    fi
    [[ " $skipped_packages " == *" waywiser_teleop "* ]] \
        || extras="$(append_unique "$extras" waywiser_teleop)"
    if [[ " $skipped_packages " != *" waywiser_test_runner "* ]]; then
        extras="$(append_unique "$extras" waywiser_test_runner email)"
    fi

    printf '%s\n' "$extras"
}

deb_extras() {
    local deb_prefix="ros-${ros_distro}-"
    local extras=""
    local deb package extra

    while IFS= read -r deb; do
        [[ -n "$deb" ]] || continue
        package="${deb#${deb_prefix}}"
        extra="${package//-/_}"
        if extra_exists "$extra"; then
            extras="$(append_unique "$extras" "$extra")"
        fi
    done < <(dpkg-query -W -f='${binary:Package}\n' "${deb_prefix}waywiser*" 2>/dev/null || true)

    if [[ " $extras " == *" waywiser_test_runner "* ]]; then
        extras="$(append_unique "$extras" waywiser_test_runner email)"
    fi

    if [[ " $extras " == *" waywiser_perception "* ]]; then
        extras="$(append_unique "$extras" waywiser_perception)"
        if is_jetson; then
            extras="$(append_unique "$extras" waywiser_perception_jetson)"
        elif [[ "$(uname -m)" == "x86_64" || "$(dpkg --print-architecture 2>/dev/null)" == "amd64" ]]; then
            extras="$(append_unique "$extras" waywiser_perception_x86)"
            if has_nvidia_gpu; then
                extras="$(append_unique "$extras" waywiser_perception_x86_cuda waywiser_perception_tensorrt)"
            fi
        elif [[ "$(uname -m)" == "aarch64" ]]; then
            warn "Detected aarch64 without Jetson markers; installing common perception dependencies only."
        fi
    fi

    printf '%s\n' "$extras"
}

needs_cpu_only_torch() {
    [[ "$(uname -m)" == "x86_64" ]] || return 1
    [[ " $extras " == *" waywiser_perception "* ]] || return 1
    [[ " $extras " == *" waywiser_perception_x86_cuda "* ]] && return 1
    [[ " $extras " == *" waywiser_perception_tensorrt "* ]] && return 1
    has_nvidia_gpu && return 1
    return 0
}

install_cpu_only_torch() {
    needs_cpu_only_torch || return 0

    info "Preinstalling CPU-only torch/torchvision from $torch_cpu_index"
    uv pip install \
        --python "$venv_dir/bin/python" \
        --default-index "$torch_cpu_index" \
        torch torchvision
}

pyproject="$(find_pyproject || true)"
[[ -n "$pyproject" ]] || die "pyproject.toml not found. Pass --pyproject PATH or install ros-$ros_distro-waywiser."

if [[ -n "$repo_dir" ]]; then
    repo_dir="$(cd "$repo_dir" && pwd)"
    editable=true
    if [[ -z "$python_version" && -f "$repo_dir/.python-version" ]]; then
        python_version="$(cat "$repo_dir/.python-version")"
    fi
    if [[ -z "${WAYWISER_SYSTEM_SITE_PACKAGES+x}" ]]; then
        system_site_packages=false
    fi
fi

if [[ -n "$explicit_extras" ]]; then
    extras="$(normalize_extras "$explicit_extras")"
elif [[ -n "$repo_dir" ]]; then
    extras="$(source_extras)"
else
    extras="$(deb_extras)"
fi
valid_extras=""
for extra in $extras; do
    if extra_exists "$extra"; then
        valid_extras="$(append_unique "$valid_extras" "$extra")"
    else
        warn "Skipping unknown pyproject extra: $extra"
    fi
done
extras="$valid_extras"

if [[ "$print_only" == true ]]; then
    printf '%s\n' "$extras"
    exit 0
fi

command -v uv &>/dev/null || die "uv is required. Install it first: https://github.com/astral-sh/uv"

info "Using pyproject: $pyproject"
info "Using venv: $venv_dir"
[[ -n "$repo_dir" ]] && info "Using source repo: $repo_dir"
if [[ -n "$extras" ]]; then
    info "Detected extras: $extras"
else
    info "No optional WayWiseR extras detected; installing base dependencies only."
fi

if [[ "$clear_venv" == true ]]; then
    remove_path "$venv_dir"
elif [[ -d "$venv_dir" && "$OPT_YES" != true ]]; then
    read -rp "  Reuse existing venv at $venv_dir? [Y/n]: " answer
    answer="${answer:-y}"
    [[ "${answer,,}" =~ ^y ]] || remove_path "$venv_dir"
fi

venv_args=(venv)
[[ -n "$python_version" ]] && venv_args+=(--python "$python_version")
[[ "$system_site_packages" == true ]] && venv_args+=(--system-site-packages)
if [[ -d "$venv_dir" && "$clear_venv" != true ]]; then
    venv_args+=(--allow-existing)
fi
uv "${venv_args[@]}" "$venv_dir"
install_cpu_only_torch

if [[ "$editable" == true ]]; then
    extras_csv="${extras// /,}"
    install_target="$repo_dir"
    [[ -n "$extras_csv" ]] && install_target="$repo_dir[$extras_csv]"
    install_args=(--python "$venv_dir/bin/python" -e "$install_target")
else
    install_args=(--python "$venv_dir/bin/python" -r "$pyproject")
fi
for extra in $extras; do
    [[ "$editable" == true ]] || install_args+=(--extra "$extra")
done
uv pip install "${install_args[@]}"
install_activate_hook

info "WayWiseR venv ready. Activate it with:"
info "source $venv_dir/bin/activate"
