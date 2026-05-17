#!/bin/bash

# Load WayWiseR runtime configuration when a ROS setup file is sourced.

_waywiser_source_env_file() {
    local env_file="${WAYWISER_ENV_FILE:-}"

    if [[ -z "$env_file" ]]; then
        if [[ -n "${WAYWISER_WS:-}" && -f "$WAYWISER_WS/src/WayWiseR/.env" ]]; then
            env_file="$WAYWISER_WS/src/WayWiseR/.env"
        elif [[ -f "$PWD/src/WayWiseR/.env" ]]; then
            env_file="$PWD/src/WayWiseR/.env"
        elif [[ -f /etc/waywiser/waywiser.env ]]; then
            env_file="/etc/waywiser/waywiser.env"
        fi
    fi

    [[ -n "$env_file" && -f "$env_file" ]] || return 0

    set -a
    # shellcheck disable=SC1090
    source "$env_file"
    set +a
    export WAYWISER_ENV_FILE="$env_file"
}

_waywiser_source_env_file
unset -f _waywiser_source_env_file
