#!/bin/bash

# Zenoh Router Setup Script for WayWiseR
# Analogous to server_setup.bash for FastDDS Discovery Server.
#
# This script starts a Zenoh router (rmw_zenohd) which is required for
# ROS 2 node discovery when using rmw_zenoh_cpp.

# Check if the script is being sourced
[[ "${BASH_SOURCE[0]}" != "${0}" ]] && is_sourced=true || is_sourced=false

# Function to safely exit or return
safe_exit() {
    local exit_code=${1:-0}
    if [[ "$is_sourced" == "true" ]]; then
        return "$exit_code" 2>/dev/null || exit "$exit_code"
    else
        exit "$exit_code"
    fi
}

# Function to display usage information
display_usage() {
    echo "Usage: $(basename "$0") [-h] [-r remote_router_ip] [-p port]"
    echo ""
    echo "Starts a Zenoh router for ROS 2 node discovery."
    echo ""
    echo "Options:"
    echo "  -h, --help            Display this help message"
    echo "  -r, --remote <ip>     Connect to a remote Zenoh router at this IP"
    echo "  -p, --port <port>     Port for the remote router (default: 7447)"
    echo "  -4, --ipv4            Use IPv4 only (for systems without IPv6)"
    echo ""
    echo "Environment variables (can be set in .env):"
    echo "  ZENOH_ROUTER_CONFIG_URI     Path to custom router config file"
    echo "  ZENOH_CONFIG_OVERRIDE       Override specific config fields"
    echo "  ZENOH_REMOTE_ROUTER_IP      IP of remote Zenoh router to connect to"
    echo "  ZENOH_REMOTE_ROUTER_PORT    Port of remote Zenoh router (default: 7447)"
    echo "  RUST_LOG                    Zenoh logging level (e.g. zenoh=info)"
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

# Default values
remote_router_ip=""
remote_router_port="7447"
ipv4_only=false

# Read from environment if set
if [[ -n "$ZENOH_REMOTE_ROUTER_IP" ]] && [[ "$ZENOH_REMOTE_ROUTER_IP" != '""' ]] && [[ "$ZENOH_REMOTE_ROUTER_IP" != "''" ]]; then
    remote_router_ip=${ZENOH_REMOTE_ROUTER_IP//\"/}
    remote_router_ip=${remote_router_ip//\'/}
fi

if [[ -n "$ZENOH_REMOTE_ROUTER_PORT" ]] && [[ "$ZENOH_REMOTE_ROUTER_PORT" != '""' ]]; then
    remote_router_port=${ZENOH_REMOTE_ROUTER_PORT//\"/}
    remote_router_port=${remote_router_port//\'/}
fi

# Parse options dynamically
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            display_usage
            $is_sourced && return 0 || exit 0
            ;;
        -r|--remote)
            remote_router_ip="$2"
            shift 2
            ;;
        -p|--port)
            remote_router_port="$2"
            shift 2
            ;;
        -4|--ipv4)
            ipv4_only=true
            shift 1
            ;;
        *)
            echo "Invalid option: $1" >&2
            display_usage
            $is_sourced && return 1 || exit 1
            ;;
    esac
done

########################################################################

echo "=== WayWiseR Zenoh Router Setup ==="

# Build ZENOH_CONFIG_OVERRIDE
config_override="${ZENOH_CONFIG_OVERRIDE:-}"

# Connect to a remote router if specified
if [[ -n "$remote_router_ip" ]]; then
    if is_valid_ip "$remote_router_ip"; then
        echo "  Connecting to remote Zenoh router: tcp/$remote_router_ip:$remote_router_port"
        connect_override="connect/endpoints=[\"tcp/$remote_router_ip:$remote_router_port\"]"
        if [[ -n "$config_override" ]]; then
            config_override="$config_override;$connect_override"
        else
            config_override="$connect_override"
        fi
    else
        echo "Error: '$remote_router_ip' is not a valid IP address." >&2
        $is_sourced && return 1 || exit 1
    fi
fi

# Handle IPv4-only systems
if $ipv4_only; then
    echo "  IPv4-only mode: listening on tcp/0.0.0.0:7447"
    listen_override='listen/endpoints=["tcp/0.0.0.0:7447"]'
    if [[ -n "$config_override" ]]; then
        config_override="$config_override;$listen_override"
    else
        config_override="$listen_override"
    fi
fi

# Export override if we have any
if [[ -n "$config_override" ]]; then
    export ZENOH_CONFIG_OVERRIDE="$config_override"
fi

# Print configuration
echo "  ZENOH_ROUTER_CONFIG_URI=${ZENOH_ROUTER_CONFIG_URI:-<default>}"
echo "  ZENOH_CONFIG_OVERRIDE=${ZENOH_CONFIG_OVERRIDE:-<none>}"
echo "  RUST_LOG=${RUST_LOG:-<unset>}"
echo ""
echo "Starting Zenoh router (rmw_zenohd)..."
echo "Press Ctrl+C to stop."
echo ""

# Start the Zenoh router
ros2 run rmw_zenoh_cpp rmw_zenohd
