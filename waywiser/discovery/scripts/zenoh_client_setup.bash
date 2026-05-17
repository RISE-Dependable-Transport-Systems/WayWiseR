#!/bin/bash

# Zenoh Client (Session) Setup Script for WayWiseR
# This script is registered as an ament environment hook and is automatically
# sourced when `source install/setup.bash` is called.
#
# It configures the Zenoh session for ROS 2 nodes when using rmw_zenoh_cpp.
# If RMW_IMPLEMENTATION is not set to rmw_zenoh_cpp, this script does nothing.

# Only run when sourced
[[ "${BASH_SOURCE[0]}" != "${0}" ]] && is_sourced=true || is_sourced=false
if [[ "$is_sourced" != "true" ]]; then
    echo "This script must be sourced, not executed."
    exit 1
fi

source_waywiser_env_file() {
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

source_waywiser_env_file
unset -f source_waywiser_env_file

# Skip if not using Zenoh
if [[ "$RMW_IMPLEMENTATION" != "rmw_zenoh_cpp" ]]; then
    OPTIND=1
    return 0
fi

# Guard against duplicate sourcing
if [[ "$_WAYWISER_ZENOH_CLIENT_CONFIGURED" == "1" ]]; then
    OPTIND=1
    return 0
fi

# Function to display usage information
display_usage() {
    echo "Zenoh Client Setup for WayWiseR"
    echo ""
    echo "Configured via environment variables (set in .env):"
    echo "  RMW_IMPLEMENTATION=rmw_zenoh_cpp    (required to activate)"
    echo "  ZENOH_USE_LOCAL_ROUTER              Set to 1 to force nodes to use a local router"
    echo "  ZENOH_REMOTE_ROUTER_IP              IP of remote Zenoh router"
    echo "  ZENOH_REMOTE_ROUTER_PORT            Port of remote router (default: 7447)"
    echo "  ZENOH_SESSION_CONFIG_URI            Path to custom session config"
    echo "  ZENOH_CONFIG_OVERRIDE               Override specific config fields"
    echo "  ZENOH_ROUTER_CHECK_ATTEMPTS         Router check behavior (-1=skip, 0=infinite, N=attempts)"
}

# Function to check if input contains a valid IP address
is_valid_ip() {
    local ip_pattern='^([0-9]{1,3}\.){3}[0-9]{1,3}$'
    if [[ $1 =~ $ip_pattern ]]; then
        return 0
    else
        return 1
    fi
}

########################################################################

echo "--- Zenoh Middleware Active ---"

# Read remote router settings from environment
remote_router_ip=""
remote_router_port="7447"

if [[ -n "$ZENOH_REMOTE_ROUTER_IP" ]] && [[ "$ZENOH_REMOTE_ROUTER_IP" != '""' ]] && [[ "$ZENOH_REMOTE_ROUTER_IP" != "''" ]]; then
    remote_router_ip=${ZENOH_REMOTE_ROUTER_IP//\"/}
    remote_router_ip=${remote_router_ip//\'/}
fi

if [[ -n "$ZENOH_REMOTE_ROUTER_PORT" ]] && [[ "$ZENOH_REMOTE_ROUTER_PORT" != '""' ]]; then
    remote_router_port=${ZENOH_REMOTE_ROUTER_PORT//\"/}
    remote_router_port=${remote_router_port//\'/}
fi

if [[ "$ZENOH_USE_LOCAL_ROUTER" == "1" ]]; then
    echo "    Mode: local (using local router, ignoring remote IP for nodes)"
    # If the local router runs on a non-standard port or needs explicit connecting, we could set connect/endpoints here
    # However, rmw_zenoh_cpp's default behavior will natively find the local router via UDP/shm.
elif [[ -n "$remote_router_ip" ]]; then
    if is_valid_ip "$remote_router_ip"; then
        echo "    Remote router: tcp/$remote_router_ip:$remote_router_port"

        # Set client mode and connect to remote router
        zenoh_override='mode="client";connect/endpoints=["tcp/'"$remote_router_ip"':'"$remote_router_port"'"]'

        # Merge with any existing ZENOH_CONFIG_OVERRIDE
        if [[ -n "$ZENOH_CONFIG_OVERRIDE" ]]; then
            export ZENOH_CONFIG_OVERRIDE="$zenoh_override;$ZENOH_CONFIG_OVERRIDE"
        else
            export ZENOH_CONFIG_OVERRIDE="$zenoh_override"
        fi
    else
        echo "    Warning: ZENOH_REMOTE_ROUTER_IP='$remote_router_ip' is not a valid IP. Using default (local) config."
    fi
else
    echo "    Mode: local (connecting to router on localhost)"
fi

# Print configuration summary
echo "### Zenoh session configured ###"
echo "  RMW_IMPLEMENTATION=\"$RMW_IMPLEMENTATION\""
echo "  ZENOH_SESSION_CONFIG_URI=\"${ZENOH_SESSION_CONFIG_URI:-<default>}\""
echo "  ZENOH_CONFIG_OVERRIDE=\"${ZENOH_CONFIG_OVERRIDE:-<none>}\""
echo "  ZENOH_ROUTER_CHECK_ATTEMPTS=\"${ZENOH_ROUTER_CHECK_ATTEMPTS:-<unset (default: 1)>}\""

# Mark as configured to prevent duplicate output
export _WAYWISER_ZENOH_CLIENT_CONFIGURED=1

# Reset getopts index for next sourcing
OPTIND=1
