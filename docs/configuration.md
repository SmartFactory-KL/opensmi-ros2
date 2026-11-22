# Configuration

## The machine file

The top-level tables are `[machine]`, `[ros2]`, `[server]` and `[[component]]`, and any other key is
an error. The docstring of
`opensmi.ros2.server.loader` describes the format, and [`examples/machine.toml`](../examples/machine.toml)
uses every Binding kind.

- `[machine]` takes `name` and an `identification` table with `serial_number`, `manufacturer` and
  `product_instance_uri`. Optional keys are `projection`, `machine_class` and `startup_skill_type`,
  and TF Bindings as an array of tables, `[[machine.tf]]`.
- Each `[[component]]` takes `name` and an `identification` table with `serial_number` and
  `manufacturer`. Optional keys are `parent`, `bindings_toml`, `projection`, `component_class` and
  `startup_skill_type`. Components form a flat list: `parent` names a Component that appears
  earlier in the file, and a Component without one belongs to the Machine.
- Bindings are arrays of tables under their Component, such as `[[component.monitoring]]`. Only `tf`
  is also allowed under `[machine]`. [Bindings](bindings.md) lists the eight keys, their fields and
  the rules each one is checked against.
- `machine_class`, `component_class`, `startup_skill_type`, a lifecycle Binding's `driver_type` and
  the callables of a `functions` Projection name your own code by import path (`package.module.Name`
  or `package.module:Name`). Loading the file imports those modules, so treat a machine file as
  trusted code.

The file has no environment-variable substitution and no command-line overrides.

## The `[ros2]` table

| Key | Default | Meaning |
| --- | --- | --- |
| `domain_id` | `0` | The ROS 2 Domain ID of this Machine's Runtime |
| `node_name` | `"opensmi_ros2"` | Name of the one rclpy Node the Runtime owns |
| `namespace` | none | ROS 2 namespace of that node |
| `stop_timeout` | `5.0` | Seconds stopping waits for the executor thread |
| `startup_timeout` | `30.0` | Seconds a required Binding has to appear before its StartupSkill gives up |
| `listing_access_level` | `0` | Access level for the introspection listings |
| `skill_access_level` | `10` | Access level for a Skill, on top of the Lock |
| `introspection_access_level` | `20` | Access level for `BindTopic` and `CallService`; at least `skill_access_level` |
| `debug_goal_outcome_booleans` | `false` | Also expose `succeeded`, `aborted` and `canceled` on action Skills |

A nested `[ros2.transforms]` table sets where TF is read from: `path` (default `"/tf"`),
`static_path` (`"/tf_static"`), `depth` (`100`), `interval` (`1.0` seconds between polls) and
`quiet_after` (`5.0` seconds before a silent frame reads `BadNoCommunication`).

The Runtime passes `domain_id` to rclpy directly, so the `ROS_DOMAIN_ID` environment variable does
not choose the domain; set `domain_id`. The package reads no environment variables of its own.
Variables that ROS 2 itself reads still apply. On WSL, for example, Fast DDS needs
`FASTDDS_BUILTIN_TRANSPORTS=SHM`.

## The opensmi server configuration

The endpoint, the users and the logging belong to opensmi.server, which reads them from its own TOML
file. Point to that file with `[server] config = "server.toml"` in the machine file (relative to the
machine file) or with `--server-config PATH` (relative to the working directory). The command line
wins, and without either opensmi's defaults apply. This file moves the endpoint to another port:

```toml
[ua_server]
endpoint_address = "opc.tcp://0.0.0.0:4850/server"
```

The same file holds the user table (`[access_control]`) and the log levels
(`[logging] minimum_levels`). opensmi.server documents its keys. For a deployment, see
[Deployment](deployment.md).
