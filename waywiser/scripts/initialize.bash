#!/usr/bin/env bash
# Configure WayWiseR runtime settings and create the Python virtual environment.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
configure_script="$script_dir/configure_env.bash"
setup_venv_script="$script_dir/setup_venv.bash"

usage() {
    cat <<'EOF'
Usage: initialize.bash [options]

Runs configure_env.bash first, then setup_venv.bash.

Common options:
  -y, --yes, --quiet  non-interactive
  -h, --help          show this help

Most configure_env.bash and setup_venv.bash options are forwarded to the
matching helper. Use those commands directly for advanced one-step workflows.
EOF
}

configure_args=()
setup_venv_args=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        --env-file|--example)
            configure_args+=("$1" "${2:?$1 requires a path}")
            shift 2
            ;;
        --repo-dir)
            configure_args+=("$1" "${2:?$1 requires a path}")
            setup_venv_args+=("$1" "$2")
            shift 2
            ;;
        --venv|--pyproject|--skipped-packages|--extras|--python)
            setup_venv_args+=("$1" "${2:?$1 requires a value}")
            shift 2
            ;;
        --editable|--clear|--system-site-packages|--no-system-site-packages|--print-extras)
            setup_venv_args+=("$1")
            shift
            ;;
        -y|--yes|--quiet)
            configure_args+=("$1")
            setup_venv_args+=("$1")
            shift
            ;;
        *)
            echo "ERROR: unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ " ${setup_venv_args[*]} " != *" --print-extras "* ]]; then
    "$configure_script" "${configure_args[@]}"
fi

"$setup_venv_script" "${setup_venv_args[@]}"
