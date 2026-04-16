# WayWiseR Meta Package

The `waywiser` package is a core component that provides global environment configurations and discovery tools for the entire WayWiseR project. It contains:

- **Environment Hooks**: Automatically loads project-wide `.env` settings during workspace sourcing.
- **Launch Files**: Top-level launch files for various use cases (Simulator, Hardware Bringup, Teleop).
- **Discovery Tools**: Scripts and profiles for FastDDS and Zenoh middleware.
- **Shared Utilities**: Common Python and C++ helper functions.

## Middleware Selection

WayWiseR supports two ROS 2 middleware (RMW) implementations. Set `RMW_IMPLEMENTATION` in your `.env` file:

| Middleware | `RMW_IMPLEMENTATION` | Discovery |
|---|---|---|
| **FastDDS** (default) | `rmw_fastrtps_cpp` | Discovery Server or Multicast |
| **Zenoh** | `rmw_zenoh_cpp` | Zenoh Router |

---

## Zenoh Middleware Setup

[rmw_zenoh](https://github.com/ros2/rmw_zenoh/tree/humble) provides a lightweight, efficient alternative to DDS. It uses a **Zenoh router** for node discovery and a **peer session** for each ROS 2 node.

> [!IMPORTANT]
> Install `rmw_zenoh_cpp` before using Zenoh:
> ```bash
> sudo apt update && sudo apt install ros-humble-rmw-zenoh-cpp
> ```

### Quick Start (Single Machine)

1. Set `RMW_IMPLEMENTATION=rmw_zenoh_cpp` in your `.env` file.

2. **Terminal 1** — Start the Zenoh router:

   ```bash
   cd $WAYWISER_WS
   source .venv/bin/activate
   ./install/waywiser/discovery/zenoh_router_setup.bash
   ```

3. **Terminal 2** — Source and run a node:

   ```bash
   cd $WAYWISER_WS
   source .venv/bin/activate
   ros2 run demo_nodes_cpp talker
   ```

4. **Terminal 3** — Source and run another node:

   ```bash
   cd $WAYWISER_WS
   source .venv/bin/activate
   ros2 run demo_nodes_cpp listener
   ```

### Multi-Machine Setup

To connect ROS 2 nodes across multiple machines, the Zenoh routers must be linked.

#### On the server machine

Set in `.env`:
```
RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Start the router (it listens on port 7447 by default):
```bash
./install/waywiser/discovery/zenoh_router_setup.bash
```

#### On the client machine

Set in `.env`:
```
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_REMOTE_ROUTER_IP=<server_ip>
```

**Option A**: Connect the client's Zenoh router to the server's router, then run nodes normally:
```bash
# Start router connected to the remote router
./install/waywiser/discovery/zenoh_router_setup.bash -r <server_ip>
```

**Option B**: Run nodes in client mode (no local router needed) — the session config is set automatically when `ZENOH_REMOTE_ROUTER_IP` is configured in `.env`:
```bash
source install/setup.bash
ros2 run demo_nodes_cpp listener
```

### IPv4-Only Systems

If your system does not support IPv6, start the router with `-4`:
```bash
./install/waywiser/discovery/zenoh_router_setup.bash -4
```

### Advanced Configuration

- **Custom router config**: `export ZENOH_ROUTER_CONFIG_URI=/path/to/config.json5`
- **Custom session config**: `export ZENOH_SESSION_CONFIG_URI=/path/to/config.json5`
- **Override specific fields**: `export ZENOH_CONFIG_OVERRIDE='key/path=value;key2/path2=value2'`
- **Enable multicast (no router needed)**:
  ```bash
  export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
  export ZENOH_CONFIG_OVERRIDE='scouting/multicast/enabled=true'
  ```
- **Zenoh logging**: `export RUST_LOG=zenoh=info`

> [!NOTE]
> For the full list of Zenoh configuration options, see the [rmw_zenoh documentation](https://github.com/ros2/rmw_zenoh/tree/humble#configuration) and [Zenoh default config](https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5).

---

## FastDDS Discovery Server and Client Setup

The `waywiser` package provides a robust setup for FastDDS Discovery Server and Client modes. This is recommended for stable communication in multi-machine environments or networks where standard multicast discovery is unreliable.

> [!IMPORTANT]
> Before proceeding, ensure you have followed the steps in the **[How to install and build](../README.md#how-to-install-and-build-on-ubuntu-2204)** section of the main README.

### On server

- Ensure `ROS_USE_DISCOVERY_SERVER=1` is set in your `.env`.
- If running a remote server, ensure `ROS_REMOTE_DISCOVERY_SERVER_IP` is set to your server's IP.

1. **Open a new terminal** and activate the workspace:

   ```bash
   cd $WAYWISER_WS
   source .venv/bin/activate
   ```

2. **Start the server**:

   ```bash
   # This automatically starts Server 0 (local) and Server 1 (remote) if configured
   ./install/waywiser/discovery/server_setup.bash
   ```

### On client

- Ensure your `.env` matches the server's `ROS_DOMAIN_ID`.
- For remote clients, set `ROS_REMOTE_DISCOVERY_SERVER_IP` to the server's IP and `ROS_REMOTE_DISCOVERY_CLIENT_IP` to your client's IP.

1. **Open a terminal** and activate/configure:

   ```bash
   cd $WAYWISER_WS
   source .venv/bin/activate
   # The client is configured automatically during sourcing if enabled in .env

   # Optional: If you changed .env recently, restart the daemon
   ros2 daemon stop
   ```

2. **Verify connectivity**:

   ```bash
   ros2 run demo_nodes_cpp talker
   ```

3. **In another terminal** (also sourced):

   ```bash
   ros2 run demo_nodes_cpp listener
   ```

### Manual Configuration

If you need to override `.env` settings for a specific terminal, you can still source the script manually with flags:

```bash
# Example: Manually setting a remote client
source install/waywiser/discovery/client_setup.bash -r -s <server_ip> -c <client_ip>
```

> [!NOTE]
> System-level network parameter tunings can address issues faced with large messages or real-world networks. See [the ROS 2 documentation](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html#cross-vendor-tuning) for guidance.
