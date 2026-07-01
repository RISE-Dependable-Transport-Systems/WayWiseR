# FastDDS Discovery Server and Client Setup

The `waywiser` package provides a robust setup for FastDDS Discovery Server and Client modes. This is recommended for stable communication in multi-machine environments or networks where standard multicast discovery is unreliable.

> [!IMPORTANT]
> Before proceeding, ensure you have followed the steps in the **[How to install and build](../README.md#how-to-install-and-build-on-ubuntu-2204)** section of the main README.

### On server

- Ensure `FASTDDS_USE_DISCOVERY_SERVER=1` is set in your `.env`.
- If running a remote server, ensure `FASTDDS_REMOTE_DISCOVERY_SERVER_IP` is set to your server's IP.

1. **Open a new terminal** and activate the workspace:

   ```bash
   cd $WAYWISER_WS
   source .venv/bin/activate
   ```

2. **Start the server**:

   ```bash
   # This automatically starts Server 0 (local) and Server 1 (remote) if configured
   ./install/waywiser/fastdds_server_setup.bash
   ```

### On client

- Ensure your `.env` matches the server's `ROS_DOMAIN_ID`.
- For remote clients, set `FASTDDS_REMOTE_DISCOVERY_SERVER_IP` to the server's IP and `FASTDDS_REMOTE_DISCOVERY_CLIENT_IP` to your client's IP.

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
source install/waywiser/fastdds_client_setup.bash -r -s <server_ip> -c <client_ip>
```

> [!NOTE]
> System-level network parameter tunings can address issues faced with large messages or real-world networks. See [the ROS 2 documentation](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html#cross-vendor-tuning) for guidance.
