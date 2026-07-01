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

Start a Zenoh router with:

```bash
ros2 run waywiser zenoh_router
```

Please refer to the [Zenoh Setup Guide](discovery/docs/ZENOH_SETUP.md) for detailed configuration, routing, and security encryption options.

---

## FastDDS Discovery Server and Client Setup

Start a FastDDS discovery server with:

```bash
ros2 run waywiser fastdds_server
```

Please refer to the [FastDDS Setup Guide](discovery/docs/FASTDDS_SETUP.md) for detailed instructions on configuring multi-machine environments in FastDDS.
