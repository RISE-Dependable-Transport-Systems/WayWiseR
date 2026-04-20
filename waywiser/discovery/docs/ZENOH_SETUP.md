# Zenoh Middleware Setup

[rmw_zenoh](https://github.com/ros2/rmw_zenoh/tree/humble) provides a lightweight, efficient alternative to DDS. It uses a **Zenoh router** for node discovery and a **peer session** for each ROS 2 node.

> [!IMPORTANT]
> Install `rmw_zenoh_cpp` before using Zenoh:
> ```bash
> sudo apt update && sudo apt install ros-${ROS_DISTRO}-rmw-zenoh-cpp
> ```

### Quick Start (Single Machine)

1. Set `RMW_IMPLEMENTATION=rmw_zenoh_cpp` in your `.env` file.

2. **Terminal 1** — Start the Zenoh router:

   ```bash
   cd $WAYWISER_WS
   source .venv/bin/activate
   ./install/waywiser/zenoh_router_setup.bash
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

- Set in `.env`:
   ```ini
   RMW_IMPLEMENTATION=rmw_zenoh_cpp
   ```

- Start the router (it listens on port 7447 by default):
   ```bash
   ./install/waywiser/zenoh_router_setup.bash
   ```

#### On the client machine

- Ensure your `.env` matches the server's `ROS_DOMAIN_ID`.

- Set in `.env`:
   ```ini
   RMW_IMPLEMENTATION=rmw_zenoh_cpp
   ZENOH_REMOTE_ROUTER_IP=<server_ip>
   ```

- **Option A (Recommended)**: Run a local router connected to the server's router.
  This creates an efficient local "bridge". All your local nodes talk rapidly to your local router, which intelligently multiplexes traffic over the network to the server's router. 
  **(Requires `ZENOH_USE_LOCAL_ROUTER=1` in `.env` to prevent nodes from bypassing the local router)**

  ```bash
  # Start your local router (it reads ZENOH_REMOTE_ROUTER_IP from .env automatically to connect)
  ./install/waywiser/zenoh_router_setup.bash
  ```

- **Option B**: Direct client mode (no local router needed). 
  Your ROS 2 nodes will connect directly across the network to the server's router. This is simpler to run initially, but can use more bandwidth if you have many local nodes competing for network connections. 
  **(Requires `ZENOH_USE_LOCAL_ROUTER=0` in `.env`. Nodes will automatically connect to ZENOH_REMOTE_ROUTER_IP)**

  ```bash
  # Simply run your nodes:
  source install/setup.bash
  ros2 run demo_nodes_cpp listener
  ```

### IPv4-Only Systems

If your system does not support IPv6, start the router with the explicit `--ipv4` flag instead:
```bash
./install/waywiser/zenoh_router_setup.bash --ipv4
```

### Advanced Configuration

- **Custom router config**: Set `ZENOH_ROUTER_CONFIG_URI=/path/to/config.json5` in your `.env`.
- **Custom session config**: Set `ZENOH_SESSION_CONFIG_URI=/path/to/config.json5` in your `.env`.
- **Override specific fields**: Set `ZENOH_CONFIG_OVERRIDE='key/path=value;key2/path2=value2'` in your `.env`.
- **Enable multicast (no router needed)**: Set the following in your `.env`:
  ```ini
  ZENOH_ROUTER_CHECK_ATTEMPTS=-1
  ZENOH_CONFIG_OVERRIDE='scouting/multicast/enabled=true'
  ```
- **Zenoh logging**: `export RUST_LOG=zenoh=info`

> [!NOTE]
> For the full list of Zenoh configuration options, see the [rmw_zenoh documentation](https://github.com/ros2/rmw_zenoh/tree/humble#configuration) and [Zenoh default config](https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5).

---

### Zenoh Encryption & Security

To secure your Zenoh ROS 2 traffic comprehensively over public networks using **TLS** or **QUIC** encryption natively (including Option A and Option B configurations), rigorously consult the dedicated [Zenoh Security Guide](ZENOH_SECURITY.md).
