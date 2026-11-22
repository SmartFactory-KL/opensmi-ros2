# Deployment

opensmi's defaults are meant for development. A server that a robot in a shared network depends on
needs three things set in the deployment, usually in its Plant Integration.

**Users and encryption.** By default every opensmi user has its name as password, the endpoint is
unencrypted (security policy None), and the server listens on every interface. Set your own user
table with strong passwords, turn on encryption, and bind the endpoint to the interface the clients
use. In the opensmi server configuration:

```toml
[ua_server]
endpoint_address = "opc.tcp://192.0.2.10:4841/server"
encryption = true

[[access_control.users]]
name = "orchestrator"
password = "base64:..."
priority = 10
maximum_access_level = 10
allow_multiple = false
```

A user table replaces opensmi's default users, so list every user the site needs. With `encryption =
true` opensmi loads the certificate and key named in `[ua_server]` and creates a self-signed pair if
they are missing. The unencrypted endpoint stays available alongside the encrypted ones (an open
point in opensmi.server), so restrict who can reach the port as well.

**The ROS 2 network.** The ROS 2 graph is not authenticated: every host in the same Domain ID can
publish to the topics the server subscribes to, call the services it uses and see what it publishes.
Run the robot and the server in an isolated network, or use SROS2, and give each robot its own
`domain_id`.

**The robot's own safety.** OpenSMI-ROS2 guarantees nothing safety-relevant. Protective stops, speed
limits and safety-rated sensors stay with the robot controller and the PLC. A robot that takes
velocity commands from a publishing Binding needs its own command timeout, so it stops when no new
command arrives.
