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

_waywiser_make_package_completion() {
    local current first_goal package
    current="${COMP_WORDS[COMP_CWORD]}"
    first_goal="${COMP_WORDS[1]:-}"

    if [[ "$first_goal" != "build" && "$first_goal" != "rebuild" ]]; then
        if declare -F _make >/dev/null; then
            _make
        fi
        return
    fi

    if [[ -z "${_WAYWISER_MAKE_PACKAGES:-}" ]]; then
        local workspace="${WAYWISER_WS:-}"
        local makefile
        if [[ -n "$workspace" && -d "$workspace/src/WayWiseR" ]]; then
            makefile="$workspace/src/WayWiseR/Makefile"
        elif [[ -d "$PWD/src/WayWiseR" ]]; then
            makefile="$PWD/src/WayWiseR/Makefile"
        elif [[ -f "$PWD/Makefile" && -d "$PWD/waywiser" ]]; then
            makefile="$PWD/Makefile"
        else
            return
        fi

        _WAYWISER_MAKE_PACKAGES="$(
            make --no-print-directory --silent -f "$makefile" list-packages 2>/dev/null
        )"
    fi

    local candidates=()
    while IFS= read -r package; do
        [[ -n "$package" ]] || continue
        if [[ " ${COMP_WORDS[*]:2} " != *" $package "* || "$package" == "$current" ]]; then
            candidates+=("$package")
        fi
    done <<< "$_WAYWISER_MAKE_PACKAGES"

    mapfile -t COMPREPLY < <(compgen -W "${candidates[*]}" -- "$current")
}

if [[ -n "${BASH_VERSION:-}" && "$-" == *i* ]]; then
    complete -o bashdefault -o default -F _waywiser_make_package_completion make
fi
