#!/usr/bin/env bash
# Configure a WayWiseR .env file for source workspaces and deb installs.
set -euo pipefail

OPT_YES=false
ENV_FILE="${WAYWISER_ENV_FILE:-}"
ENV_EXAMPLE="${WAYWISER_ENV_FILE_EXAMPLE:-}"
REPO_DIR="${WAYWISER_REPO_DIR:-}"

usage() {
    cat <<'EOF'
Usage: configure_env.bash [options]

Options:
  --env-file PATH     .env file to create/update
  --example PATH      template env file to copy/read defaults from
  --repo-dir PATH     source repo root; enables package selection UI
  -y, --yes, --quiet  keep current/default values without prompting
  -h, --help          show this help

Environment overrides:
  WAYWISER_ENV_FILE
  WAYWISER_ENV_FILE_EXAMPLE
  WAYWISER_REPO_DIR
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --env-file)
            ENV_FILE="${2:?--env-file requires a path}"; shift 2 ;;
        --example)
            ENV_EXAMPLE="${2:?--example requires a path}"; shift 2 ;;
        --repo-dir)
            REPO_DIR="${2:?--repo-dir requires a path}"; shift 2 ;;
        -y|--yes|--quiet)
            OPT_YES=true; shift ;;
        --prereqs-only|--setup-only|--configure|--build-only|--post-build-only|\
        --skip-prereqs|--skip-mavsdk|--skip-px4-drone)
            # Accepted as no-ops so Makefile/bootstrap ARGS can be shared.
            shift ;;
        -h|--help)
            usage; exit 0 ;;
        *)
            echo "ERROR: unknown option: $1" >&2
            usage >&2
            exit 2 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREFIX_DIR="$(cd "$SCRIPT_DIR/../.." 2>/dev/null && pwd || true)"

find_source_repo() {
    local candidate
    for candidate in \
        "$SCRIPT_DIR/../.." \
        "${WAYWISER_WS:-}/src/WayWiseR" \
        "$PWD/src/WayWiseR" \
        "$PREFIX_DIR/../../src/WayWiseR"
    do
        [[ -n "$candidate" ]] || continue
        if [[ -f "$candidate/.env.example" ]]; then
            cd "$candidate" && pwd
            return 0
        fi
    done
    return 1
}

if [[ -z "$REPO_DIR" ]]; then
    REPO_DIR="$(find_source_repo || true)"
fi

ros_distro="${ROS_DISTRO:-humble}"
if [[ -z "$ENV_EXAMPLE" ]]; then
    if [[ -n "$REPO_DIR" && -f "$REPO_DIR/.env.example" ]]; then
        ENV_EXAMPLE="$REPO_DIR/.env.example"
    elif [[ -n "$PREFIX_DIR" && -f "$PREFIX_DIR/share/waywiser/waywiser.env.example" ]]; then
        ENV_EXAMPLE="$PREFIX_DIR/share/waywiser/waywiser.env.example"
    else
        ENV_EXAMPLE="/opt/ros/$ros_distro/share/waywiser/waywiser.env.example"
    fi
fi

if [[ -z "$ENV_FILE" ]]; then
    if [[ -n "$REPO_DIR" ]]; then
        ENV_FILE="$REPO_DIR/.env"
    else
        ENV_FILE="/etc/waywiser/waywiser.env"
    fi
fi

header() { echo; echo "===> $*"; echo; }
info() { echo "  $*"; }
warn() { echo "  WARNING: $*" >&2; }
die() { echo "ERROR: $*" >&2; exit 1; }

as_root() {
    if [[ "$(id -u)" -eq 0 ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

write_file() {
    local src="$1" dst="$2" dst_dir
    dst_dir="$(dirname "$dst")"
    mkdir -p "$dst_dir" 2>/dev/null || as_root mkdir -p "$dst_dir"
    if install -m 0644 "$src" "$dst" 2>/dev/null; then
        :
    elif [[ -w "$dst_dir" && ( ! -e "$dst" || -w "$dst" ) ]]; then
        install -m 0644 "$src" "$dst"
    else
        as_root install -m 0644 "$src" "$dst"
    fi
}

ensure_env_file() {
    [[ -f "$ENV_EXAMPLE" ]] || die "Env template not found: $ENV_EXAMPLE"
    if [[ ! -f "$ENV_FILE" ]]; then
        write_file "$ENV_EXAMPLE" "$ENV_FILE"
        info "Created $ENV_FILE from $ENV_EXAMPLE."
    fi
}

env_get() {
    grep -m1 "^${1}=" "${2:-$ENV_FILE}" 2>/dev/null \
        | cut -d= -f2- | tr -d '"' || true
}

env_set() {
    local key="$1" val="$2" tmp
    tmp="$(mktemp)"
    if [[ -f "$ENV_FILE" ]]; then
        awk -v key="$key" -v val="$val" '
            BEGIN { done = 0 }
            $0 ~ "^" key "=" { print key "=" val; done = 1; next }
            { print }
            END { if (!done) print key "=" val }
        ' "$ENV_FILE" > "$tmp"
    else
        printf '%s=%s\n' "$key" "$val" > "$tmp"
    fi
    write_file "$tmp" "$ENV_FILE"
    rm -f "$tmp"
}

cfg_get() {
    local value
    value="$(env_get "$1")"
    [[ -z "$value" ]] && value="$(env_get "$1" "$ENV_EXAMPLE")"
    printf '%s\n' "${value:-${2:-}}"
}

prompt_yn() {
    local question="$1" default="${2:-false}" answer
    if [[ "$OPT_YES" == true ]]; then
        [[ "$default" == true ]]
        return
    fi
    if [[ "$default" == true ]]; then
        read -rp "  ${question} [Y/n]: " answer
        answer="${answer:-y}"
    else
        read -rp "  ${question} [y/N]: " answer
        answer="${answer:-n}"
    fi
    [[ "${answer,,}" =~ ^y ]]
}

wt_form() {
    local -n _wf_result="$1"; local title="$2" body="$3"; shift 3
    local labels=() defaults=()
    while [[ $# -ge 2 ]]; do
        labels+=("$1"); defaults+=("$2"); shift 2
    done
    local n=${#labels[@]}
    if [[ "$OPT_YES" == true ]]; then
        _wf_result=("${defaults[@]}")
        return 0
    fi

    local w body_lines h form_h max_lbl=0
    w=$(( $(tput cols 2>/dev/null || echo 80) - 4 ))
    (( w < 70 )) && w=70
    body_lines=$(printf '%s' "$body" | wc -l)
    h=$(( body_lines + n + 9 ))
    (( h > 40 )) && h=40
    form_h=$(( n < 14 ? n : 14 ))
    for lbl in "${labels[@]}"; do
        (( ${#lbl} > max_lbl )) && max_lbl=${#lbl}
    done
    local field_x=$(( max_lbl + 3 ))
    local field_len=$(( w - field_x - 4 ))
    (( field_len < 10 )) && field_len=10

    if command -v dialog &>/dev/null && [[ -t 1 ]]; then
        local args=()
        for (( i=0; i<n; i++ )); do
            args+=("${labels[$i]}" $(( i + 1 )) 1 "${defaults[$i]}" \
                $(( i + 1 )) "$field_x" "$field_len" 0)
        done
        local result rc
        result=$(dialog --keep-tite --title "$title" --form "$body" "$h" "$w" "$form_h" \
            "${args[@]}" 3>&1 1>&2 2>&3) || rc=$?
        tput cnorm 2>/dev/null || true
        [[ "${rc:-0}" -eq 0 ]] || return 1
        mapfile -t _wf_result <<< "$result"
        while (( ${#_wf_result[@]} < n )); do _wf_result+=(""); done
    else
        if ! command -v dialog &>/dev/null; then
            warn "'dialog' is not installed; using line-by-line prompts."
            warn "Install it with: sudo apt install dialog"
        elif [[ ! -t 1 ]]; then
            warn "No interactive terminal detected; using line-by-line prompts."
        fi
        echo; echo "  === $title ==="
        [[ -n "$body" ]] && echo "  $body"
        _wf_result=()
        for (( i=0; i<n; i++ )); do
            local value
            read -rp "  ${labels[$i]} [${defaults[$i]}]: " value
            _wf_result+=("${value:-${defaults[$i]}}")
        done
    fi
}

package_checklist() {
    local -n _pc_ref="$1"
    local cur_skip="$2"
    local -r locked="waywiser waywiser_core waywiser_description waywiser_twist_safety"

    local search_dir="${WAYWISER_WS:-$(cd "${REPO_DIR:-$PWD}"/../.. 2>/dev/null && pwd)}/src"
    if [[ ! -d "$search_dir" ]]; then
        search_dir="${REPO_DIR:-$PWD}"
    fi

    if [[ -z "$search_dir" || ! -d "$search_dir" ]]; then
        _pc_ref="$cur_skip"
        return 0
    fi

    local all_pkgs=()
    while IFS= read -r name; do
        [[ -n "$name" ]] && all_pkgs+=("$name")
    done < <(find "$search_dir" -name package.xml \
        ! -path "*/external/*" ! -path "*/.git/*" \
        -exec grep -m1 '<name>' {} \; \
        | sed 's|.*<name>||;s|</name>.*||' | sort -u)

    local pkgs=()
    for pkg in "${all_pkgs[@]}"; do
        [[ " $locked " == *" $pkg "* ]] || pkgs+=("$pkg")
    done

    if [[ ${#pkgs[@]} -eq 0 || "$OPT_YES" == true ]]; then
        _pc_ref="$cur_skip"
        return 0
    fi

    local pkg_list_str=" ${all_pkgs[*]} "
    declare -A rev_deps
    while IFS= read -r xml; do
        local depender
        depender=$(grep -m1 '<name>' "$xml" | sed 's|.*<name>||;s|</name>.*||')
        while IFS= read -r dep; do
            [[ "$pkg_list_str" == *" $dep "* ]] || continue
            rev_deps["$dep"]+="$depender "
        done < <(grep -E '<(depend|build_depend|exec_depend|run_depend)>' "$xml" \
            | sed 's|^[[:space:]]*<[^>]*>||;s|</[^>]*>.*||' | sort -u)
    done < <(find "$search_dir" -name package.xml \
        ! -path "*/external/*" ! -path "*/.git/*")

    if command -v dialog &>/dev/null && [[ -t 1 ]]; then
        local locked_lines="" items=()
        for pkg in $locked; do
            [[ " ${all_pkgs[*]} " == *" $pkg "* ]] && locked_lines+="  * $pkg"$'\n'
        done
        for pkg in "${pkgs[@]}"; do
            local state="ON" desc="" rdeps="${rev_deps[$pkg]:-}"
            [[ " $cur_skip " == *" $pkg "* ]] && state="OFF"
            [[ -n "$rdeps" ]] && desc="[needed by: ${rdeps% }]"
            items+=("$pkg" "$desc" "$state")
        done

        local n=${#pkgs[@]}
        local height=$(( n + 12 )) width result exit_status=0
        (( height > 34 )) && height=34
        width=$(( $(tput cols 2>/dev/null || echo 80) - 4 ))
        (( width < 60 )) && width=60
        result=$(dialog --keep-tite \
            --title "WayWiseR - Build Packages" \
            --checklist \
"Always built (cannot be skipped):
${locked_lines}
Check packages to BUILD (uncheck to skip).
(Space = toggle  |  Arrow keys = navigate  |  Enter = confirm)" \
            "$height" "$width" "$n" \
            "${items[@]}" \
            3>&1 1>&2 2>&3) || exit_status=$?
        tput cnorm 2>/dev/null || true
        [[ $exit_status -eq 0 ]] || return 1

        local selected new_skip=()
        selected="$(echo "$result" | tr -d '"')"
        for pkg in "${pkgs[@]}"; do
            [[ " $selected " == *" $pkg "* ]] || new_skip+=("$pkg")
        done
        _pc_ref="${new_skip[*]:-}"
    else
        local new_skip=()
        echo "  Always built (cannot be skipped):"
        for pkg in $locked; do
            [[ " ${all_pkgs[*]} " == *" $pkg "* ]] && echo "    * $pkg"
        done
        echo
        echo "  Select packages to build (answer n to skip):"
        for pkg in "${pkgs[@]}"; do
            local def=true rdeps="${rev_deps[$pkg]:-}" hint=""
            [[ " $cur_skip " == *" $pkg "* ]] && def=false
            [[ -n "$rdeps" ]] && hint=" [needed by: ${rdeps% }]"
            prompt_yn "Build $pkg?$hint" "$def" || new_skip+=("$pkg")
        done
        _pc_ref="${new_skip[*]:-}"
    fi
}

ensure_dialog() {
    if command -v dialog &>/dev/null || [[ "$OPT_YES" == true ]] || [[ ! -t 1 ]]; then
        return 0
    fi

    info "'dialog' package is not installed (required for interactive configuration UI)."
    if prompt_yn "Install missing 'dialog' package (sudo apt-get install -y dialog)?" true; then
        info "Installing 'dialog'..."
        if as_root apt-get update -qq && as_root apt-get install -y -qq dialog; then
            info "'dialog' installed successfully."
        else
            warn "Failed to install 'dialog'. Falling back to line-by-line prompts."
        fi
    fi
}

configure_env() {
    header "Configuring WayWiseR environment"
    ensure_env_file

    ensure_dialog

    if [[ -f "$ENV_FILE" ]]; then
        info "Using env file: $ENV_FILE"
        info "Defaults/template: $ENV_EXAMPLE"
    fi

    local cur_skipped new_skipped
    cur_skipped="$(env_get WAYWISER_SKIPPED_PACKAGES)"
    if [[ -n "$REPO_DIR" && -d "$REPO_DIR" ]]; then
        package_checklist new_skipped "$cur_skipped" || return 1
    else
        new_skipped="$cur_skipped"
        info "No source repo detected; preserving package selection."
    fi

    local current_rmw
    current_rmw="$(cfg_get RMW_IMPLEMENTATION rmw_fastrtps_cpp)"
    case "$current_rmw" in
        rmw_fastrtps_cpp|rmw_zenoh_cpp) ;;
        *) current_rmw="rmw_fastrtps_cpp" ;;
    esac
    local dds_default="fastdds"
    if [[ "$current_rmw" == "rmw_zenoh_cpp" ]]; then
        dds_default="zenoh"
    fi

    local form_args=(
        "WAYWISER_VENV_PATH (empty = use default)"            "$(cfg_get WAYWISER_VENV_PATH "")"
        "Build PX4/drone support (1/0)"                       "$(cfg_get WAYWISER_BUILD_PX4_DRONE 1)"
        "DDS middleware (fastdds|zenoh)"                      "$dds_default"
        "ROS_DOMAIN_ID (0-232)"                               "$(cfg_get ROS_DOMAIN_ID 0)"
        "RCUTILS_LOGGING_USE_STDOUT (1/0)"                    "$(cfg_get RCUTILS_LOGGING_USE_STDOUT 1)"
        "RCUTILS_LOGGING_BUFFERED_STREAM (1/0)"               "$(cfg_get RCUTILS_LOGGING_BUFFERED_STREAM 1)"
        "PYTHONUNBUFFERED (1/0)"                              "$(cfg_get PYTHONUNBUFFERED 1)"
        "RCUTILS_COLORIZED_OUTPUT (1/0)"                      "$(cfg_get RCUTILS_COLORIZED_OUTPUT 1)"
        "FASTDDS_USE_DISCOVERY_SERVER (1/0)"                  "$(cfg_get FASTDDS_USE_DISCOVERY_SERVER 0)"
        "FASTDDS_REMOTE_DISCOVERY_SERVER_IP"                  "$(cfg_get FASTDDS_REMOTE_DISCOVERY_SERVER_IP "")"
        "FASTDDS_REMOTE_DISCOVERY_CLIENT_IP"                  "$(cfg_get FASTDDS_REMOTE_DISCOVERY_CLIENT_IP "")"
        "FASTDDS_SUPER_CLIENT (1/0)"                          "$(cfg_get FASTDDS_SUPER_CLIENT 0)"
        "ZENOH_USE_LOCAL_ROUTER (1/0)"                        "$(cfg_get ZENOH_USE_LOCAL_ROUTER 1)"
        "ZENOH_REMOTE_ROUTER_IP"                              "$(cfg_get ZENOH_REMOTE_ROUTER_IP "")"
        "ZENOH_REMOTE_ROUTER_PORT"                            "$(cfg_get ZENOH_REMOTE_ROUTER_PORT 7447)"
        "ZENOH_ROUTER_CHECK_ATTEMPTS"                         "$(cfg_get ZENOH_ROUTER_CHECK_ATTEMPTS "-1")"
        "ZENOH_SESSION_CONFIG_URI"                            "$(cfg_get ZENOH_SESSION_CONFIG_URI "")"
        "ZENOH_CONFIG_OVERRIDE"                               "$(cfg_get ZENOH_CONFIG_OVERRIDE "")"
        "EMAIL_USER"                                          "$(cfg_get EMAIL_USER "")"
        "EMAIL_PASSWORD"                                      "$(env_get EMAIL_PASSWORD)"
        "EMAIL_RECIPIENT"                                     "$(cfg_get EMAIL_RECIPIENT "")"
        "SMTP_SERVER"                                         "$(cfg_get SMTP_SERVER smtp.gmail.com)"
        "SMTP_PORT"                                           "$(cfg_get SMTP_PORT 465)"
    )

    local search_dir="${WAYWISER_WS:-$(cd "${REPO_DIR:-$PWD}"/../.. 2>/dev/null && pwd)}/src"
    local carla_in_ws=false
    if [[ -d "$search_dir" ]] && \
       find "$search_dir" -maxdepth 4 -name package.xml ! -path '*/.git/*' \
            -exec grep -qlm1 '<name>waywiser_carla</name>' {} \; -print -quit 2>/dev/null \
       | grep -q .; then
        carla_in_ws=true
    fi

    if [[ "$carla_in_ws" == true && " $new_skipped " != *" waywiser_carla "* ]]; then
        form_args+=( "WAYWISER_CUSTOM_CARLA_ROOT" "$(cfg_get WAYWISER_CUSTOM_CARLA_ROOT "")" )
    fi

    local cfg_vals=()
    wt_form cfg_vals "WayWiseR Runtime Configuration" \
        "Edit settings. Booleans use 1=yes, 0=no. DDS middleware options: fastdds or zenoh." \
        "${form_args[@]}" \
        || return 1

    local new_rmw
    case "${cfg_vals[2],,}" in
        fastdds|fast|rmw_fastrtps_cpp)
            new_rmw="rmw_fastrtps_cpp" ;;
        zenoh|rmw_zenoh_cpp)
            new_rmw="rmw_zenoh_cpp" ;;
        *)
            warn "Invalid DDS middleware '${cfg_vals[2]}'. Use fastdds or zenoh."
            return 1 ;;
    esac

    if [[ -n "$new_skipped" ]]; then
        env_set WAYWISER_SKIPPED_PACKAGES "\"$new_skipped\""
    else
        env_set WAYWISER_SKIPPED_PACKAGES ""
    fi
    env_set WAYWISER_VENV_PATH                  "${cfg_vals[0]}"
    env_set WAYWISER_BUILD_PX4_DRONE            "${cfg_vals[1]}"
    env_set ROS_DOMAIN_ID                       "${cfg_vals[3]}"
    env_set RMW_IMPLEMENTATION                  "$new_rmw"
    env_set RCUTILS_LOGGING_USE_STDOUT          "${cfg_vals[4]}"
    env_set RCUTILS_LOGGING_BUFFERED_STREAM     "${cfg_vals[5]}"
    env_set PYTHONUNBUFFERED                    "${cfg_vals[6]}"
    env_set RCUTILS_COLORIZED_OUTPUT            "${cfg_vals[7]}"
    env_set FASTDDS_USE_DISCOVERY_SERVER        "${cfg_vals[8]}"
    env_set FASTDDS_REMOTE_DISCOVERY_SERVER_IP  "${cfg_vals[9]}"
    env_set FASTDDS_REMOTE_DISCOVERY_CLIENT_IP  "${cfg_vals[10]}"
    env_set FASTDDS_SUPER_CLIENT                "${cfg_vals[11]}"
    env_set ZENOH_USE_LOCAL_ROUTER              "${cfg_vals[12]}"
    env_set ZENOH_REMOTE_ROUTER_IP              "${cfg_vals[13]}"
    env_set ZENOH_REMOTE_ROUTER_PORT            "${cfg_vals[14]}"
    env_set ZENOH_ROUTER_CHECK_ATTEMPTS         "${cfg_vals[15]}"
    env_set ZENOH_SESSION_CONFIG_URI            "${cfg_vals[16]}"
    env_set ZENOH_CONFIG_OVERRIDE               "${cfg_vals[17]}"
    env_set EMAIL_USER                          "${cfg_vals[18]}"
    env_set EMAIL_PASSWORD                      "${cfg_vals[19]}"
    env_set EMAIL_RECIPIENT                     "${cfg_vals[20]}"
    env_set SMTP_SERVER                         "${cfg_vals[21]}"
    env_set SMTP_PORT                           "${cfg_vals[22]}"

    if [[ "$carla_in_ws" == true && " $new_skipped " != *" waywiser_carla "* ]]; then
        env_set WAYWISER_CUSTOM_CARLA_ROOT      "${cfg_vals[23]}"
    fi

    echo
    info "Saved to $ENV_FILE"
    info "  RMW_IMPLEMENTATION=$new_rmw"
    info "  ROS_DOMAIN_ID=${cfg_vals[3]}"
    info "  WAYWISER_BUILD_PX4_DRONE=${cfg_vals[1]}"
    if [[ -n "$new_skipped" ]]; then
        info "  WAYWISER_SKIPPED_PACKAGES=\"$new_skipped\""
    elif [[ -n "$REPO_DIR" ]]; then
        info "  WAYWISER_SKIPPED_PACKAGES= (all selectable packages will be built)"
    fi

    echo
    if [[ "$ENV_FILE" == /etc/waywiser/* ]]; then
        info "To apply changes in this shell, run:"
        info "  source /opt/ros/${ros_distro}/setup.bash"
    else
        local workspace_hint="${WAYWISER_WS:-}"
        local venv_activate=""
        if [[ -z "$workspace_hint" && -n "$REPO_DIR" ]]; then
            workspace_hint="$(cd "$REPO_DIR/../.." 2>/dev/null && pwd || true)"
        fi
        if [[ -n "$workspace_hint" ]]; then
            venv_activate="$workspace_hint/.venv/bin/activate"
        fi
        if [[ -n "$venv_activate" && -f "$venv_activate" ]]; then
            info "To apply changes in this shell, run:"
            info "  source $venv_activate"
        elif [[ -f "$ENV_FILE" ]]; then
            info "To apply changes in this shell right now, run:"
            info "  set -a; source $ENV_FILE; set +a"
        else
            info "To apply changes in this shell, re-source your WayWiseR workspace environment."
        fi
    fi
}

configure_env
