# Publishing Bindings

A publishing Binding turns a topic into a Skill. The message fields are its Parameters: a client
writes them in any order, then calls `Start`, and only `Start` publishes. A finite Binding
(`continuous = false`) publishes once and reaches Completed. A continuous one is a stream: it
publishes the current Parameters on every tick of `rate` until it halts.

```toml
[[component.publisher]]
name = "Drive"
path = "/cmd_vel"
message_type = "geometry_msgs/msg/Twist"
continuous = true
rate = 20.0
stale_after = 0.5
```

A stream halts when a client calls `Halt`, and also when nobody is in charge of it any more:

- **The Lock ends.** A client needs the Machine's Lock to call `Start`. When it stops holding the
  Lock, because it gave it back, another client broke it, it expired (by default after ten minutes
  without `RenewLock`), or the client's session closed, every stream it started halts. The error code
  and the BindingEvent code are `LOCK_ENDED`.
- **The setpoint goes stale.** With `stale_after` (seconds), the stream halts when no client has
  written any of its Parameters for that long. Every write starts the interval again, even one that
  writes the same value, so a client holding a steady speed keeps writing it. The code is
  `SETPOINT_STALE`. Without `stale_after` only the Lock and `Halt` stop the stream.

Without more, a halting stream just stops publishing. With `halt_message` it publishes that
message once as it ends, whatever ended it (`Halt`, the Lock, a stale setpoint or failed publishes):

```toml
halt_message = { linear = { x = 0.0 }, angular = { z = 0.0 } }
```

The table names ROS 2 fields (`child_frame_id`, not `ChildFrameId`), nested for a nested message, and
ignores the Binding's selection and `rename`. A field it does not name keeps the default the
message type's generated ROS 2 class gives it. If the halt message cannot be published, the stream
halts anyway and a BindingEvent with `PUBLISH_FAILED` says so.

The robot's own command timeout (for example the velocity timeout of its base controller) stays
required either way: it is what brings the robot to rest when no halt message is declared or it is
lost, and the stream cannot stop the robot if this server itself dies.

`stale_after` must be a positive number of seconds, and `halt_message` a table. Both are allowed
only on a Binding with a rate, and a mistake is reported with the other errors of the machine file.
Whether the halt message fits its message type is checked when the server starts, since the type
is only known then: a field the type does not have, or a value the field cannot hold, stops it with
the field path.
