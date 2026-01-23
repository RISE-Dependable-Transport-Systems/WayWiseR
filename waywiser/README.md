# WayWiseR Meta Package

The `waywiser` package is a core component that provides global environment configurations and discovery tools for the entire WayWiseR project. It contains:

- **Environment Hooks**: Automatically loads project-wide `.env` settings during workspace sourcing.
- **Launch Files**: Top-level launch files for various use cases (Simulator, Hardware Bringup, Teleop).
- **Discovery Tools**: Scripts and FastDDS profiles for stable communication.
- **Shared Utilities**: Common Python and C++ helper functions.

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
