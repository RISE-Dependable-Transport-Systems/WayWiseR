# Zenoh Security Configuration

This guide details how to secure your WayWiseR Zenoh ROS 2 traffic using Transport Layer Security (TLS) or QUIC. 

Out of the box, standard Zenoh `tcp/` and `udp/` connections transmit ROS 2 topics unencrypted over your network. By natively forcing the `tls/` or `quic/` protocols and providing explicitly trusted cryptographic certificates, all communication between your nodes and Edge/Cloud routers is securely encrypted and optionally mutually authenticated.

---

## 1. Generating Certificates (OpenSSL)

If you don't already have an official Certificate Authority (CA) issuing keys for your enterprise, use the provided helper script to automatically generate your Server and Client certificates securely:

```bash
cd $WAYWISER_WS
./install/waywiser/generate_tls_cert.bash server
./install/waywiser/generate_tls_cert.bash client1  # Optional: For mTLS
```

The script will securely output a `waywiser_certs/` directory locally. If no Root CA exists, it will automatically instantiate one for you. You now securely possess the fundamental files necessary for active TLS encryption:
- `ca.crt` (Share this securely across all endpoints)
- `server.key` / `server.crt` (Keep securely on your Server)
- `client1.key` / `client1.crt` (Reserved for your Clients if using mTLS)

---

## 2. Configuring the Server (Router)

To enforce encryption, the local Zenoh router instance running on your Server must natively listen via `tls/` or `quic/` rather than standard `tcp/`. 

Inside the server's `.env` configuration file, inject the `ZENOH_CONFIG_OVERRIDE` to override default endpoints and supply the paths to the certificates:

### For Standard TLS:
```ini
# Bind strictly via TLS and load the server certificates
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tls/0.0.0.0:7447"];transport/link/tls/server_private_key="/path/to/server.key";transport/link/tls/server_certificate="/path/to/server.crt"'
```

### For QUIC (UDP TLS):
QUIC natively operates heavily over UDP, which heavily offsets the handshake latencies generally associated with traditional TCP streams, making it structurally advantageous for highly dynamic ROS 2 topics.
```ini
# Bind strictly via QUIC and load the server certificates
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["quic/0.0.0.0:7447"];transport/link/quic/server_private_key="/path/to/server.key";transport/link/quic/server_certificate="/path/to/server.crt"'
```

*Note: You would launch the server router using the standard command: `./install/waywiser/zenoh_router_setup.bash`*

---

## 3. Configuring the Clients (Edge Nodes)

Your client configuration depends on whether you are running a Local Router on the edge device (**Option A**), or if your nodes are directly talking to the server natively (**Option B**).

In both Option architectures, the edge device needs the overarching `ca.crt` file present on its host system so it can mathematically verify the server's encryption signatures.

### For Standard TLS:
```ini
ZENOH_REMOTE_ROUTER_IP="tls/<server_ip>"
ZENOH_CONFIG_OVERRIDE='transport/link/tls/root_ca_certificate="/path/to/ca.crt"'
```

### For QUIC (UDP TLS):
```ini
ZENOH_REMOTE_ROUTER_IP="quic/<server_ip>"
ZENOH_CONFIG_OVERRIDE='transport/link/quic/root_ca_certificate="/path/to/ca.crt"'
```

---

## 4. Mutual TLS (mTLS) Security

By default, the previous steps mathematically prove the Server's identity to the Clients natively guaranteeing that the client nodes are talking to the correct server. However, it does **not** stop unauthorized clients from connecting if they know the IP.

To enforce **Mutual Authentication (mTLS)**, you must generate a `client.crt` and `client.key` (using the same OpenSSL steps). 

Then, on the **Server**, rigorously enforce that the server demands client certificates:
```ini
ZENOH_CONFIG_OVERRIDE='...;transport/link/tls/root_ca_certificate="/path/to/ca.crt"'
```

And on the **Client**, explicitly supply its newly generated identities:
```ini
ZENOH_CONFIG_OVERRIDE='...;transport/link/tls/server_private_key="/path/to/client.key";transport/link/tls/server_certificate="/path/to/client.crt"'
```
> *Zenoh 1.0 explicitly uses the `server_` parameter name for TLS keys on the client endpoint.*
