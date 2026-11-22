# Bindings

A Binding connects one ROS 2 interface to its OPC UA representation on a Component (or, for a TF
Binding, on the Machine). This page is the reference for all eight kinds: what each maps, how a
machine file declares it, every field with its default, what is refused and when, how QoS behaves
and which nodes appear in the address space.

## The eight kinds

| Domain concept | Python class | TOML key | ROS 2 side | OPC UA side |
| --- | --- | --- | --- | --- |
| Monitoring Binding | `MonitoringBinding` | `monitoring` | topic | Monitoring variables |
| Keyed Binding | `KeyedBinding` | `keyed` | topic with a repeated field | one group of variables per key |
| Method Binding | `MethodBinding` | `method` | service | Method |
| Action Binding | `ActionBinding` | `action` | action | finite Skill |
| Publisher Binding | `PublisherBinding` | `publisher` | topic, published | Skill (one-shot or stream) |
| Lifecycle Binding | `LifecycleBinding` | `lifecycle` | lifecycle node or manager | StartupSkill |
| ROS 2 parameter Binding | `ROS2ParameterBinding` | `ros2_parameter` | parameters of one node | Parameters in the ParameterSet |
| TF Binding | `TFBinding` | `tf` | TF tree | [RSL](https://reference.opcfoundation.org/specs/OPC-10000-210) Locations in a SpatialObject |

All classes are importable from `opensmi.ros2.server`. In a machine file each key is an array of
tables below its Component, for example `[[component.monitoring]]`. Only `tf` is also allowed below
`[machine]` (`[[machine.tf]]`). The same keys, without the `component.` prefix, are the tables of a
Binding file that a Component includes with `bindings_toml`. A Binding file may hold any of the eight
tables, and any other table name is refused.

## Declaring Bindings

- In Python, a Component lists Bindings in its `bindings` class attribute. A constructor may assign
  `self.bindings` to compute the list.
- In TOML, a Component declares them in its tables, or in a file named by `bindings_toml`.
- Both sources union, the Python ones first. A name may be declared once across all sources, and a
  second declaration raises `DuplicateBindingNameError`. Neither source can change a Binding the
  other wrote.
- A Machine takes `TFBinding` only. Any other kind on a Machine is refused.

Python declarations are frozen dataclasses with keyword-only fields. A TOML file cannot name a Python
object, so the fields that hold a class or a callable (`skill_type`, `goal_from`, `result_to`,
`method_type`, `request_from`, `response_to`, `message_from`, and a class given as `driver`) are
Python-only. A table that carries one of them, or any other unknown key, is refused. A lifecycle table
may name a driver class with `driver_type`, by import path, which the machine file loader resolves.

TOML arrays become tuples, and a TOML table with integer keys (`states`, `severities`) becomes a
dictionary with integer keys.

## Fields every Binding has

| Field | Type | Default | Meaning |
| --- | --- | --- | --- |
| `name` | `str` | required | Becomes a browse name. It must follow `^[A-Z][A-Za-z0-9]*$`. |
| `required` | `bool` | `False` | The StartupSkill waits for the Binding before `Running`. |
| `startup_timeout` | `float or None` | `None` | Seconds that wait lasts. `None` takes `[ros2] startup_timeout` (30.0). |

`PublisherBinding` is the exception. It has neither `required` nor `startup_timeout`, because the
publisher is this server itself and there is nothing to wait for. To wait for the other end,
declare a required `MonitoringBinding` on the topic that answers.

What "satisfied" means depends on the kind:

| Kind | The StartupSkill waits until |
| --- | --- |
| Monitoring, Keyed | a publisher is on the topic (presence, never data) |
| Method | a service server is discovered |
| Action | an action server is discovered |
| ROS 2 parameter | every declared parameter is confirmed on the node |
| Lifecycle | every observed node is on the graph (it publishes `transition_event`) |
| TF | a publisher is on a transform topic (never a particular frame) |

Checks common to all kinds, made when the Binding is built (a Python constructor call, or the check of
the whole machine file):

- An empty name, or one that breaks the naming rule, raises `BindingNameError`. The message suggests a
  valid name where it can, for example `try 'JointStates'` for `joint_states`. A name is never converted.
- `startup_timeout` without `required = true`, or one that is not positive, raises
  `RequiredDeclarationError`.
- Every value given to `rename` (and `feedback_rename`, `response_rename`, `result_rename`) must follow
  the naming rule as well, and is taken as it is.

All these errors are `DeclarationError` subclasses. In a machine file, every mistake is collected with
its path (`component[1].monitoring[2].message_type`) and raised together as one `FileDeclarationError`
before anything is built.

## When a mistake is found

The documentation of each kind separates three points in time.

- **Load.** The Binding is constructed, from Python or from the machine file. Contradictory or
  malformed declarations stop here, without ROS 2.
- **Init.** The Machine initialises and resolves message, service and action types from the installed
  ROS 2 interface packages. A type whose package is missing, or a field path the type does not have,
  is fatal here.
- **Run time.** What the graph does afterwards (a robot that is off, a publisher with another QoS, a
  service that does not answer) is reported to clients as a status code, a BindingEvent or a state,
  and does not stop the server.

## Field selection and renaming

`include`, `exclude` and `rename` narrow and rename the fields of a message. They exist on every kind
that renders a message, and kinds with several messages have one set per message.

- `include` is a list of field patterns. Empty means every field.
- `exclude` removes fields after `include` has narrowed, for example `["pose/covariance"]`.
- `rename` maps a field path to a new browse name, such as `{ "pose/position/x" = "East" }` in TOML
  inline form. The new name is taken as it is and must follow the naming rule. A path that is not
  rendered is refused at init. It resolves a name collision without a Projection.

Names that come from ROS 2 are converted part by part (`child_frame_id` becomes `ChildFrameId`) and
keep the raw name as DisplayName. See [Browse paths](browse-paths.md).

## QoS

A QoS profile has five policies: `reliability` (`reliable` or `best_effort`), `durability`
(`volatile` or `transient_local`), `deadline` and `lease` (seconds, default infinite, never zero) and
`liveliness` (`automatic` or `manual_by_topic`). The defaults are reliable, volatile, no deadline,
automatic liveliness. Queue depth is a separate keyword (`depth`), because it takes no part in
matching. In TOML a profile is a table:

<!-- toml: fragment -->
```toml
qos = { reliability = "best_effort", durability = "volatile", deadline = 0.5 }
```

Every endpoint's profile has a provenance, and one rule follows from it: the profile of an endpoint
adapts if and only if it was discovered.

| Provenance | Meaning | Adapts |
| --- | --- | --- |
| declared | written on the Binding as `qos` | never, honoured exactly for the life of the endpoint |
| discovered | computed from the topic's live publishers | yes, only ever relaxing |
| defaulted | the middleware default, for an outward endpoint with nothing declared | never |

Discovery picks, per policy, the least demanding value any current publisher offers, which is
compatible with all of them. A topic without a publisher at init is subscribed with the most
permissive request (best effort, volatile) and resolved once more when the first publisher matches,
because a volatile request would receive none of a transient-local publisher's history.

QoS is never fatal and never blocks startup. A declared profile that clashes with a publisher is
logged and reported as a BindingEvent with the code `QOS_INCOMPATIBLE`: a warning when the profile was
discovered (and relaxed), an error when it was declared (and stands, so that publisher is not
received). The provenance of each Binding's endpoint is listed by the introspection Component.

| Kind | QoS behaviour |
| --- | --- |
| Monitoring, Keyed | `qos` declared, or discovered when left out |
| Publisher | `qos` declared, or defaulted (reliable, volatile); never discovered |
| TF | no per-Binding keyword; both subscriptions are discovered, the outward publishers are fixed |
| Method, Action, ROS 2 parameter, Lifecycle | no `qos` keyword |

## MonitoringBinding

Maps a ROS 2 topic to Monitoring variables on the declaring Component, direction ROS 2 to OPC UA. TOML
key `monitoring`. The message is one complex Variable named after the Binding, and each field is a
child Variable.

| Field | Type | Default | Meaning |
| --- | --- | --- | --- |
| `name` | `str` | required | Name of the complex Variable. `Status`, `Health`, `Process` and `Consumption` are reserved. |
| `path` | `str` | required | The ROS 2 topic. |
| `message_type` | `str or None` | `None` | `package/Name` or `package/kind/Name`. `None` takes the type from the graph. |
| `include` | `tuple[str]` | `()` | Field patterns to keep. Empty keeps every field. |
| `exclude` | `tuple[str]` | `()` | Field patterns to remove. |
| `rename` | `dict[str, str]` | `{}` | New browse names by field path. |
| `depth` | `int` | `10` | Messages the subscription buffers before the oldest is dropped. |
| `qos` | `QoSProfile or None` | `None` | A declared profile, or `None` to discover one. |
| `max_rate` | `float or None` | `None` | Most writes per second to the variables. `None` mirrors every message. |
| `group` | `str or None` | `None` | Files the Binding under `Status`, `Health`, `Process` or `Consumption`. |
| `required` | `bool` | `False` | Wait for a publisher on the topic. |
| `startup_timeout` | `float or None` | `None` | Seconds that wait lasts. |

Load:

- The name rules above, and a reserved name (`DeclarationError`). The match is exact, so `StatusCode`
  is allowed.
- A `group` that is none of the four Machinery Monitoring sub-structures is refused.
- `max_rate` must be above 0 and at most 1000.0 (`DeclarationError`).

Init: a `message_type` whose package is not installed is fatal. A `rename` path that is not rendered
is refused.

Run time: a topic without publishers writes nothing. Without `message_type` the Binding is graph-driven: its nodes appear when the topic does, and
it is the only kind that makes the Machine poll the graph. A declared type gives the nodes at boot, so
the address space does not depend on start-up timing.

`max_rate` limits per Binding, so the fields of one message stay one snapshot. The first message after
a quiet spell is written at once, then at most one per `1 / max_rate` (the newest), and a message
still waiting when the topic stops is written at the end of its window. Messages that are not written
are not decoded.

OPC UA nodes: below the Component, `Monitoring/<Name>` is a complex Variable and each field is
`Monitoring/<Name>/<Field>`, nested messages nested again. With `group`, the path is
`Monitoring/<Group>/<Name>/<Field>`, and the group is a FolderType Object shared by every Binding that
names it. For the String topic `/chatter`, the text is `Monitoring/Chatter/Data`.

<!-- toml: component -->
```toml
[[component.monitoring]]
name = "Odom"
path = "/odom"
message_type = "nav_msgs/msg/Odometry"
exclude = ["pose/covariance", "twist/covariance"]
rename = { child_frame_id = "BaseFrame" }
depth = 5
qos = { reliability = "best_effort" }
max_rate = 10.0
group = "Process"
required = true
startup_timeout = 60.0
```

```python
from opensmi.ros2.runtime.qos import QoSProfile, Reliability
from opensmi.ros2.server import MonitoringBinding

binding = MonitoringBinding(
    name="Odom",
    path="/odom",
    message_type="nav_msgs/msg/Odometry",
    exclude=("pose/covariance", "twist/covariance"),
    rename={"child_frame_id": "BaseFrame"},
    depth=5,
    qos=QoSProfile(reliability=Reliability.BEST_EFFORT),
    max_rate=10.0,
    group="Process",
    required=True,
    startup_timeout=60.0,
)
```

## KeyedBinding

Maps a ROS 2 topic whose message carries a repeated message field to one group of variables per
element, direction ROS 2 to OPC UA. TOML key `keyed`. The usual instance is `/diagnostics`, keyed by
`status` on `hardware_id` and `name`. The repeated field and the key fields are declared and the
members are taken from the messages. Only the elements are rendered. For the fields beside the repeated
one, declare a `MonitoringBinding` on the same topic narrowed to them. For TF, which is keyed in the
same sense, use `TFBinding`.

| Field | Type | Default | Meaning |
| --- | --- | --- | --- |
| `name` | `str` | required | The group the members sit in. Reserved names as on `MonitoringBinding`. |
| `path` | `str` | required | The ROS 2 topic. |
| `message_type` | `str` | required | The message type. It is required, since the member shape comes from it. |
| `repeated_field` | `str` | required | The field whose elements become members. |
| `key` | `tuple[str]` | required | The element fields that together name a member, in browse-name order. |
| `state_field` | `str or None` | `None` | One field whose change is an event. |
| `states` | `dict[int, str]` | `{}` | Label of each value of `state_field`. Mints an enumeration. |
| `severities` | `dict[int, Severity]` | `{}` | Severity band of a transition into each state, `0`, `555` or `999`. |
| `include` | `tuple[str]` | `()` | Element field patterns to keep. |
| `exclude` | `tuple[str]` | `()` | Element field patterns to remove. |
| `rename` | `dict[str, str]` | `{}` | New browse names by element field path. |
| `depth` | `int` | `10` | Messages the subscription buffers. |
| `qos` | `QoSProfile or None` | `None` | Declared, or discovered. |
| `group` | `str or None` | `None` | A Machinery Monitoring sub-structure, as on `MonitoringBinding`. |
| `required` | `bool` | `False` | Wait for a publisher on the topic, never for a member. |
| `startup_timeout` | `float or None` | `None` | Seconds that wait lasts. |

Load:

- The name, reserved name, `group` and `rename` checks of `MonitoringBinding`.
- An empty `key` raises `DeclarationError`, because a set whose elements have no names is a list.
- `states` or `severities` without `state_field` raise `DeclarationError`.
- A key in `states` or `severities` that is not an integer is refused in a file.

Init: `repeated_field` must be a repeated message field of `message_type`, every `key` entry a single
value of the element, `state_field` must stay rendered (not excluded or renamed), and an element with a
field that browses as `Key` is refused, since every member carries its raw key in a variable of that
name.

Run time: members are created when their keys first appear and are never removed. The number of
members is capped by the Component's and the Machine's mutation limit, and a Binding that reaches it
reports a BindingEvent. Two keys that map to one browse name report `KEY_COLLISION` and the second gets
no member. With `state_field`, every write that moves it is a BindingEvent `MEMBER_STATE_CHANGED`
at the band `severities` gives (information without it), and a member starts at the first declared
constant so a healthy unit announces nothing when it appears.

OPC UA nodes: `Monitoring/<Name>` is a group. Each member is `Monitoring/<Name>/<Key>`, where
the key fields are joined with `_` and each of `/ . < > : # ! &` becomes `_`. The DisplayName is the
raw key. A member holds the element's variables and a `Key` variable with the raw key. With
`state_field` there is one more variable below the group, `Worst<StateField>` (`WorstLevel`), which
holds the highest state any member holds. With `group`, the path starts `Monitoring/<Group>/`.

<!-- toml: component -->
```toml
[[component.keyed]]
name = "Diagnostics"
path = "/diagnostics"
message_type = "diagnostic_msgs/msg/DiagnosticArray"
repeated_field = "status"
key = ["hardware_id", "name"]
state_field = "level"

[component.keyed.states]
0 = "OK"
1 = "WARN"
2 = "ERROR"
3 = "STALE"

[component.keyed.severities]
0 = 0
1 = 555
2 = 999
3 = 999
```

```python
from opensmi.ros2.server import KeyedBinding, Severity

binding = KeyedBinding(
    name="Diagnostics",
    path="/diagnostics",
    message_type="diagnostic_msgs/msg/DiagnosticArray",
    repeated_field="status",
    key=("hardware_id", "name"),
    state_field="level",
    states={0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"},
    severities={0: Severity.INFO, 1: Severity.WARNING, 2: Severity.ERROR, 3: Severity.STALE},
)
```

## MethodBinding

Maps a ROS 2 service to a Method on the declaring Component, direction OPC UA to ROS 2. TOML key
`method`. The request is the Method's Parameters, written before `Call`, and the response is its
FinalResultData. There is no Monitoring, since a service has no feedback. `Call` takes no input
arguments of its own and returns a status code.

| Field | Type | Default | Meaning |
| --- | --- | --- | --- |
| `name` | `str` | required | The Binding's name, also the Method's name. |
| `path` | `str` | required | The ROS 2 service. |
| `service_type` | `str` | required | `package/srv/Name`. |
| `include` | `tuple[str]` | `()` | Request field patterns to keep. |
| `exclude` | `tuple[str]` | `()` | Request field patterns to remove. |
| `rename` | `dict[str, str]` | `{}` | New browse names for request fields. |
| `response_include` | `tuple[str]` | `()` | Response field patterns to keep. |
| `response_exclude` | `tuple[str]` | `()` | Response field patterns to remove. |
| `response_rename` | `dict[str, str]` | `{}` | New browse names for response fields. |
| `success_field` | `str or None` | `None` | A `bool` response field that carries the service's verdict. |
| `server_timeout` | `float` | `10.0` | Seconds `Call` waits for a service server. |
| `call_timeout` | `float` | `30.0` | Seconds to wait for the answer once the request has gone. |
| `method_type` | `type or None` | `None` | Subclass of `ROS2ServiceMethod`. Python-only. |
| `request_from` | callable or `None` | `None` | Builds the request from the Parameters. Python-only. |
| `response_to` | callable or `None` | `None` | Turns the response into FinalResultData values. Python-only. |
| `required` | `bool` | `False` | Wait for a service server. |
| `startup_timeout` | `float or None` | `None` | Seconds that wait lasts. |

Load: the common checks, and `method_type` must be a subclass of `ROS2ServiceMethod`
(`ImplementationClassError`).

Init: a missing interface package is fatal. A `success_field` that the response lacks or that is not
a `bool` is fatal (`SuccessFieldError`). A selection that leaves the request unbuildable is refused
instead of being filled from a default.

Run time: two failures reach a client differently.

- A transport failure means the call did not happen. No server on the path gives `BadNotConnected`, and
  a server that took the request and went quiet gives `BadTimeout`.
- An application failure (the service ran and answered no) is `Good` with the response in
  FinalResultData. With `success_field`, a `False` there answers `BadRequestNotAllowed`, and the
  response is still written. No field is detected automatically.

`Call` needs the Skill access level, the Machine's Lock and a completed startup. FinalResultData and
the Parameters are never reset, so FinalResultData holds the last response that arrived.

OPC UA nodes: `MethodSet/<Name>` is the Method, with `ParameterSet` (the request fields),
`FinalResultData` (the response fields) and the `Call` method.

<!-- toml: component -->
```toml
[[component.method]]
name = "TriggerRefill"
path = "/refill/trigger"
service_type = "std_srvs/srv/Trigger"
success_field = "success"
server_timeout = 5.0
call_timeout = 20.0
```

```python
from opensmi.ros2.server import MethodBinding

binding = MethodBinding(
    name="TriggerRefill",
    path="/refill/trigger",
    service_type="std_srvs/srv/Trigger",
    success_field="success",
    server_timeout=5.0,
    call_timeout=20.0,
)
```

## ActionBinding

Maps a ROS 2 action to a finite Skill on the declaring Component, direction OPC UA to ROS 2. TOML key
`action`. The goal is the Skill's Parameters, written before `Start`, the feedback is its Monitoring
and the result is its FinalResultData. The outcome drives the state machine and is not data of its
own: `Completed` is success and `Halted` is every other ending. Each of the three messages has its own
include, exclude and rename.

| Field | Type | Default | Meaning |
| --- | --- | --- | --- |
| `name` | `str` | required | The Binding's name, also the Skill's name. |
| `path` | `str` | required | The ROS 2 action. |
| `action_type` | `str` | required | `package/action/Name`. |
| `include` | `tuple[str]` | `()` | Goal field patterns to keep. |
| `exclude` | `tuple[str]` | `()` | Goal field patterns to remove. |
| `rename` | `dict[str, str]` | `{}` | New browse names for goal fields. |
| `feedback_include` | `tuple[str]` | `()` | Feedback field patterns to keep. |
| `feedback_exclude` | `tuple[str]` | `()` | Feedback field patterns to remove. |
| `feedback_rename` | `dict[str, str]` | `{}` | New browse names for feedback fields. |
| `result_include` | `tuple[str]` | `()` | Result field patterns to keep. |
| `result_exclude` | `tuple[str]` | `()` | Result field patterns to remove. |
| `result_rename` | `dict[str, str]` | `{}` | New browse names for result fields. |
| `feedback_depth` | `int` | `10` | Feedback messages the goal handle buffers. |
| `server_timeout` | `float` | `10.0` | Seconds `Start` waits for an action server before halting. |
| `skill_type` | `type or None` | `None` | Subclass of `ROS2ActionSkill`. Python-only. |
| `goal_from` | callable or `None` | `None` | Builds the goal from the Parameters. Python-only. |
| `result_to` | callable or `None` | `None` | Turns the result into FinalResultData values. Python-only. |
| `required` | `bool` | `False` | Wait for the action server to be discovered. |
| `startup_timeout` | `float or None` | `None` | Seconds that wait lasts. |

Load: the common checks, including the three `rename` maps, and `skill_type` must derive from
`ROS2ActionSkill` (`ImplementationClassError`).

Init: a missing interface package is fatal, and a goal selection that leaves the goal unbuildable is
refused instead of being filled from a default.

Run time: `Start` reaches `Running` only once the server has accepted the goal. Without an action
server for `server_timeout` seconds the Skill halts, so it does not sit in `Starting` against a robot
that is off. An `OpenSmiRuntimeError` in a handler halts the Skill and appears in the Component's
Notification with its code as `ErrorCode`. `Halt` cancels the goal. `Reset` clears Monitoring and
FinalResultData and leaves the Parameters alone. `Suspend` does not exist, since actions cannot be
paused. `Start`, `Halt` and `Reset` need the Skill access level and the Lock, and `Start` also needs a
completed startup. With `[ros2] debug_goal_outcome_booleans = true`, FinalResultData also holds
`succeeded`, `aborted` and `canceled`.

OPC UA nodes: `SkillSet/<Name>/SkillExecution` holds `ParameterSet` (goal), `Monitoring` (feedback)
and `FinalResultData` (result), with the Skill's `Start`, `Halt` and `Reset`. Variables are named for
the field path alone.

<!-- toml: component -->
```toml
[[component.action]]
name = "NavigateToPose"
path = "/navigate_to_pose"
action_type = "nav2_msgs/action/NavigateToPose"
feedback_exclude = ["current_pose"]
feedback_depth = 5
server_timeout = 5.0
required = true
startup_timeout = 20.0
```

```python
from opensmi.ros2.server import ActionBinding

binding = ActionBinding(
    name="NavigateToPose",
    path="/navigate_to_pose",
    action_type="nav2_msgs/action/NavigateToPose",
    feedback_exclude=("current_pose",),
    feedback_depth=5,
    server_timeout=5.0,
    required=True,
    startup_timeout=20.0,
)
```

The Python-only functions are pure and take the message class, the Parameters by name and a
Projection context. The default is `opensmi.ros2.server.skill.default_goal_from`.

<!-- python: only -->
```python
from collections.abc import Mapping

from opensmi.ros2.server import ActionBinding
from opensmi.ros2.server.projection import ProjectionContext
from opensmi.ros2.server.skill import default_goal_from


def goal_with_a_log_line(message_type: type, parameters: Mapping[str, object], context: ProjectionContext):
    return default_goal_from(message_type, parameters, context)


binding = ActionBinding(
    name="NavigateToPose",
    path="/navigate_to_pose",
    action_type="nav2_msgs/action/NavigateToPose",
    goal_from=goal_with_a_log_line,
)
```

## PublisherBinding

Maps a ROS 2 topic to a Skill that publishes it, direction OPC UA to ROS 2. TOML key `publisher`. The
message fields are the Skill's Parameters, a client writes them in any order, and only `Start`
publishes. There is no Monitoring, because the echo of a topic this server publishes would only be its
own message. To watch the topic, declare a `MonitoringBinding` on it.

Continuity is declared with `continuous`, never inferred from `rate`. A finite Skill publishes once and
reaches `Completed`, and a second push is `Reset`, write, `Start`. A continuous Skill publishes the
current Parameters on every tick of `rate` until it halts and never reaches `Completed`.

| Field | Type | Default | Meaning |
| --- | --- | --- | --- |
| `name` | `str` | required | The Binding's name, also the Skill's name. |
| `path` | `str` | required | The ROS 2 topic. |
| `message_type` | `str` | required | The message type. It is required, since the Parameters exist before the graph. |
| `continuous` | `bool` | required | Stream (`true`) or publish once (`false`). |
| `include` | `tuple[str]` | `()` | Field patterns to keep. |
| `exclude` | `tuple[str]` | `()` | Field patterns to remove. |
| `rename` | `dict[str, str]` | `{}` | New browse names by field path. |
| `rate` | `float or None` | `None` | Publishes per second. Required when continuous, refused otherwise. |
| `depth` | `int` | `10` | ROS 2 queue depth for subscribers that have not taken messages yet. |
| `qos` | `QoSProfile or None` | `None` | Declared profile, or the default (reliable, volatile). |
| `stale_after` | `float or None` | `None` | Seconds without a client write before the stream halts. Continuous only. |
| `halt_message` | table or `None` | `None` | Message published once when a stream ends. Continuous only. |
| `tolerated_failures` | `int` | `0` | Consecutive publish failures a stream absorbs. Continuous only. |
| `skill_type` | `type or None` | `None` | Subclass of the stock publishing Skill for this continuity. Python-only. |
| `message_from` | callable or `None` | `None` | Builds the message from the Parameters. Python-only. |

There is no `required` and no `startup_timeout` on this kind. A table that carries one is refused.

Load (`DeclarationError` unless noted):

- On a finite Binding, `rate`, `stale_after`, `halt_message` and a non-zero `tolerated_failures` are
  refused.
- A continuous Binding without `rate` is refused, as is a `rate` outside 0.001 to 1000.0 hertz, a
  `stale_after` that is not a positive finite number, and a negative `tolerated_failures`.
- `skill_type` must derive from `ROS2ContinuousPublisherSkill` or `ROS2FinitePublisherSkill` to match
  `continuous` (`ImplementationClassError`).

Init: a missing interface package is fatal. A selection that leaves the message unbuildable is refused.
A message with a field that browses as `Rate` is refused on a continuous Binding, since the Skill has a
Parameter of that name. A `halt_message` is checked against the message type: a field the type does not
have, or a value the field cannot hold, stops it with the field path.

Run time: a stream halts on `Halt`, when the Lock it was started under ends (`LOCK_ENDED`), when no
Parameter was written for `stale_after` seconds (`SETPOINT_STALE`) and when the tolerated failures are
spent (`PUBLISH_FAILED`). Each reports a BindingEvent with that code, and the Skill halts with the same
`ErrorCode`. Every write starts the `stale_after` interval again, even one that writes the same value.
With `halt_message`, the stream publishes that message once as it ends, whatever ended it. Its table
names ROS 2 fields (`child_frame_id`, not `ChildFrameId`), nested for a nested message, and ignores
`rename`. A field it does not name keeps the default of the generated ROS 2 class. The robot's own
command timeout stays necessary, since this server cannot stop a robot if it dies itself.
See [Publishing Bindings](publishing-bindings.md) for the full halting behaviour.

OPC UA nodes: `SkillSet/<Name>/SkillExecution/ParameterSet/<Field>` holds the message fields. A
continuous Skill has one more Parameter, `Rate` (DisplayName `rate`), writable and bounded, with `rate`
as its initial value, which takes effect on the next tick. The Skill has `Start`, `Halt` and `Reset`.
It has no `Suspend`, no Monitoring and no FinalResultData.

<!-- toml: component -->
```toml
[[component.publisher]]
name = "Drive"
path = "/cmd_vel"
message_type = "geometry_msgs/msg/Twist"
continuous = true
rate = 20.0
stale_after = 0.5
tolerated_failures = 2
halt_message = { linear = { x = 0.0 }, angular = { z = 0.0 } }
```

```python
from opensmi.ros2.server import PublisherBinding

binding = PublisherBinding(
    name="Drive",
    path="/cmd_vel",
    message_type="geometry_msgs/msg/Twist",
    continuous=True,
    rate=20.0,
    stale_after=0.5,
    tolerated_failures=2,
    halt_message={"linear": {"x": 0.0}, "angular": {"z": 0.0}},
)
```

A one-shot push, for example a map or a goal topic, is a finite Binding:

<!-- toml: component -->
```toml
[[component.publisher]]
name = "SetInitialPose"
path = "/initialpose"
message_type = "geometry_msgs/msg/PoseWithCovarianceStamped"
continuous = false
exclude = ["pose/covariance"]
```

## LifecycleBinding

Maps a ROS 2 lifecycle node, or a manager of several, to the StartupSkill of the declaring Component,
in both directions. TOML key `lifecycle`. The managed node's state machine and the continuous Skill's
are the same machine:

| Skill state | Lifecycle state | Trigger | Lifecycle transition |
| --- | --- | --- | --- |
| `Halted` | `unconfigured` | `reset` | `configure` |
| `Ready` | `inactive` | `start` | `activate` |
| `Running` | `active` | `halt` | `deactivate`, then `cleanup` |

`halt` is never `shutdown`, because `finalized` is terminal and the Skill could not be reset. There is
no `suspend`. Transitions that somebody else caused move the Skill, downward only. With several
Lifecycle Bindings on one Component, they drive the one StartupSkill, and `Running` means every node is
active.

| Field | Type | Default | Meaning |
| --- | --- | --- | --- |
| `name` | `str` | required | Keeps declarations apart, prefixes the Binding's variables, names the event source. |
| `path` | `str` | required | The node the Binding commands: the managed node itself, or the manager. |
| `nodes` | `tuple[str]` | `()` | The lifecycle nodes observed, when `path` is a manager. |
| `driver` | `str or type` | `"lifecycle"` | Coordinator flavour: `lifecycle` (standard services) or `nav2`. |
| `depth` | `int` | `10` | Queue depth of each node's `transition_event` subscription. |
| `server_timeout` | `float` | `10.0` | Seconds a question or command waits for the node's services. |
| `call_timeout` | `float` | `30.0` | Seconds to wait for an answer once the request has gone. |
| `required` | `bool` | `False` | Wait for the nodes to be on the graph. |
| `startup_timeout` | `float or None` | `None` | Seconds that wait lasts. |

In a machine file, `driver_type = "package.module:Class"` names your own `LifecycleDriver` subclass
instead of `driver`. A Python declaration passes the class as `driver`.

Load:

- A `driver` name that is not registered, or a value that is neither a name nor a class, raises
  `DeclarationError`. A class that is not a `LifecycleDriver` subclass raises `ImplementationClassError`.
- A driver whose `path` is a manager (`nav2`) with an empty `nodes` raises `DeclarationError`.
- The common name and deadline checks.

Run time: a node that is off is a condition and not an error, and the next appearance looks again.
A node is present when it publishes `transition_event`, so a `finalized` node still counts. A refused
transition has no BindingEvent of its own and is reported through opensmi's failure path of the StartupSkill.

A Component with a Lifecycle Binding uses the lifecycle StartupSkill, and `startup_skill_type` in the
machine file must subclass that one.

OPC UA nodes: the Component's StartupSkill is `SkillSet/StartupSkill`. Every observed node has a
Variable with its lifecycle state at
`SkillSet/StartupSkill/SkillExecution/Monitoring/<Name>/<Node>/State`, one grouping Object per segment
of the node path (`/planner_server` is `PlannerServer`) and the DisplayName `state`. It reads
`unknown` until something is heard from the node.

<!-- toml: component -->
```toml
[[component.lifecycle]]
name = "Navigation"
path = "/lifecycle_manager_navigation"
nodes = ["/planner_server", "/controller_server"]
driver = "nav2"
required = true
```

```python
from opensmi.ros2.server import LifecycleBinding

binding = LifecycleBinding(
    name="Navigation",
    path="/lifecycle_manager_navigation",
    nodes=("/planner_server", "/controller_server"),
    driver="nav2",
    required=True,
)
```

The standard flavour needs no `nodes`, because `path` is the managed node:

<!-- toml: component -->
```toml
[[component.lifecycle]]
name = "Camera"
path = "/camera_driver"
```

## ROS2ParameterBinding

Maps the ROS 2 parameters of one node to Parameters on the declaring Component's ParameterSet, in
both directions. TOML key `ros2_parameter`. One declaration per ROS 2 node: a Component that tunes two
nodes declares two. The names, types and read-only marks are declared and never discovered, because an
OPC UA Parameter's access level is fixed when its node is created. Declaring a parameter here is the
only way to make it writable. The introspection Component only lists and describes them.

The type of each parameter is one of `bool`, `integer`, `double`, `string`, `byte_array`,
`bool_array`, `integer_array`, `double_array` and `string_array`. The union of `ParameterValue` never
reaches OPC UA: each Parameter is a scalar or array of its declared type.

| Field | Type | Default | Meaning |
| --- | --- | --- | --- |
| `name` | `str` | required | The group the Parameters sit in. |
| `path` | `str` | required | The ROS 2 node, fully qualified. A missing leading `/` is added. |
| `parameters` | `dict[str, ParameterType]` | required | ROS 2 parameter name to type. Must not be empty. |
| `read_only` | `tuple[str]` | `()` | Names among `parameters` that a client may not write. |
| `retry_interval` | `float` | `5.0` | Seconds before a parameter not yet confirmed is read again. |
| `write_timeout` | `float` | `3.0` | Seconds a client's write waits for the node before `BadTimeout`. |
| `required` | `bool` | `False` | Wait until every declared name is confirmed on the node. |
| `startup_timeout` | `float or None` | `None` | Seconds that wait lasts. |

Load (`ParameterDeclarationError` unless noted):

- An empty `parameters` map is refused.
- A `read_only` name that `parameters` does not declare is refused.
- A `retry_interval` or `write_timeout` that is not positive is refused.
- A parameter name segment that does not convert to a browse name raises `BindingNameError`.
- The common name and deadline checks.

Run time: nothing waits at init. The Parameters exist from boot with typed zeros and the status
`BadNoCommunication`, and the first read happens on a task. A name that is not confirmed is read again
every `retry_interval` seconds. Because a typo never gets confirmed, a `required` Binding stays
unsatisfied until the StartupSkill gives up. A write is checked against the node's descriptor, then
sent as `set_parameters`: `BadNotWritable` for a read-only parameter (also one the node marks
read-only itself), `BadTypeMismatch` for another type, `BadOutOfRange` outside the range (also when the
node's reason is a range), `BadInvalidArgument` for any other refusal, and `BadTimeout` without an
answer. The client gets `Good`, and the value is stored, only when the node took it. A change observed
on `/parameter_events` comes back in, so the value is the node's, whoever moved it.

BindingEvents: `PARAMETER_UNREACHABLE` when the node does not answer, `PARAMETER_ABSENT` for a declared
name the node does not have, and `PARAMETER_TYPE_MISMATCH` for one the node holds at another type. The
last two leave the Parameter at `BadNoCommunication`.

OPC UA nodes: `ParameterSet/<Name>` is a group on the Component. Each dot in a ROS 2 name is one more
grouping Object, and every segment is converted to a browse name, so `FollowPath.max_vel_x` is
`ParameterSet/<Name>/FollowPath/MaxVelX`.

<!-- toml: component -->
```toml
[[component.ros2_parameter]]
name = "Controller"
path = "/controller_server"
read_only = ["use_sim_time"]
write_timeout = 2.0

[component.ros2_parameter.parameters]
"FollowPath.max_vel_x" = "double"
use_sim_time = "bool"
```

```python
from opensmi.ros2.runtime import ParameterType
from opensmi.ros2.server import ROS2ParameterBinding

binding = ROS2ParameterBinding(
    name="Controller",
    path="/controller_server",
    parameters={
        "FollowPath.max_vel_x": ParameterType.DOUBLE,
        "use_sim_time": ParameterType.BOOL,
    },
    read_only=("use_sim_time",),
    write_timeout=2.0,
)
```

### OPC UA Parameter and ROS2ParameterBinding

The word Parameter has two unrelated meanings in this project.

- An **OPC UA Parameter** is a Variable in the `ParameterSet` of a Skill, a Method or a Component, in
  the vocabulary of OPC 40001-1 and the SkillSet model. The goal fields of an `ActionBinding`, the
  request fields of a `MethodBinding` and the message fields of a `PublisherBinding` all become
  Parameters of their Skill or Method. They hold values a client writes before `Start` or `Call`, and
  no ROS 2 parameter is involved.
- A **ROS 2 parameter** is a named, typed setting of a ROS 2 node, read and written through the node's
  parameter services. `ROS2ParameterBinding`, in TOML the key `ros2_parameter`, binds the parameters
  of one node, and each one becomes an OPC UA Parameter in the Component's ParameterSet.

The qualifier is part of the class and the key because a bare `parameter` would be ambiguous. A TOML
key `parameter` is not a Binding kind, and the file is refused.

## TFBinding

Maps the TF tree to Locations (OPC 10000-210, Relative Spatial Location) inside a SpatialObject,
in both directions, per frame. TOML key `tf`, on a Component (`[[component.tf]]`) or on the Machine
(`[[machine.tf]]`). One declaration per object. Its content is one frame name, the head: the declaring
item gains a SpatialObject whose mandatory `PositionFrame` is that frame, and every frame below it in
the tree belongs to the item, down to the next declared head. The Machine's mirror runs the whole
partition.

Where TF is read from and how often are facts about the Machine and are set once, in the
`[ros2.transforms]` table (`path`, `static_path`, `depth`, `interval`, `quiet_after`; see
[Configuration](configuration.md)). The five fields of the same names exist on `TFBinding` only so that
writing one is refused with a message saying where it went.

| Field | Type | Default | Meaning |
| --- | --- | --- | --- |
| `name` | `str` | required | Names the Binding in BindingEvents and the mutation cap. It is no prefix of any browse name. |
| `head` | `str` | required | The one frame this item is. It becomes the `PositionFrame`. |
| `alternatives` | `tuple[tuple[str, str]]` | `()` | Composed poses to publish, each a `(base, frame)` pair. |
| `publishes` | `tuple[PublishedFrame]` | `()` | Frames this item puts on the frame network instead of reading. |
| `exclude` | `tuple[str]` | `()` | `fnmatch` globs over whole frame ids that are not rendered. |
| `required` | `bool` | `False` | Wait for a publisher on a transform topic. |
| `startup_timeout` | `float or None` | `None` | Seconds that wait lasts. |
| `path`, `static_path`, `depth`, `interval`, `quiet_after` | `None` | `None` | Machine-wide, refused here. |

A `PublishedFrame` has these fields:

| Field | Type | Default | Meaning |
| --- | --- | --- | --- |
| `base` | `str` | required | The frame this one's pose is relative to (`header.frame_id`). |
| `frame` | `str` | required | The frame this declaration owns (`child_frame_id`). |
| `static` | `bool` | `False` | Publish on `/tf_static` instead of `/tf`, when the value changes. |
| `position` | `tuple[float, float, float]` | `(0.0, 0.0, 0.0)` | Starting position in metres. |
| `orientation` | `tuple[float, float, float]` | `(0.0, 0.0, 0.0)` | Roll, pitch and yaw in degrees (RSL `A`, `B`, `C`). |

Load (`DeclarationError` unless noted):

- A missing head, a head that `exclude` removes, and any of the five Machine-wide fields given a value.
- An alternative that is not a pair of two different, non-empty frames.
- A published frame with an empty `base` or `frame`, a frame that is its own parent, one that is
  declared twice, or one that `exclude` removes.
- A published frame that is the head raises `FrameDirectionError`: the head is read inward, and a frame
  is inward or outward and never both.
- The common deadline checks.

Run time: a frame removed from the middle of a chain re-bases its children onto the nearest surviving
ancestor. A frame that stays unpublished for longer than `quiet_after` reads `BadNoCommunication`, and
static frames are exempt. A frame whose browse name is taken is reported as `FRAME_COLLISION`, and an
alternative whose base has no node as `ALTERNATIVE_BASE_ABSENT`. The poll renders Locations; each one
counts about ten nodes against the mutation cap, which counts Locations. A failed broadcast reports
`PUBLISH_FAILED`.

TF QoS: both subscriptions (`/tf` and `/tf_static`) are discovered. Outward, `/tf` is offered reliable
and volatile, and `/tf_static` reliable and transient-local, so a late joiner gets the frame. A
declaration that publishes nothing never advertises `/tf`. These profiles are fixed and are not listed
as Binding QoS yet.

Outward frames: a client writes the pose of a published frame and the poll broadcasts it. Moving
frames are re-asserted every poll, and static frames when they differ from what last went out. Only the
pose is writable.

OPC UA nodes: the item gets a `SpatialObject` (a `SpatialObjectList` also appears on the Machine) whose
`PositionFrame` is the head. The other Locations are named for the sanitised `child_frame_id`, with
each of `/ . < > : # ! &` replaced by `_`, and show the raw id as DisplayName. They sit in the folders
of the object:

| Folder | Holds |
| --- | --- |
| `AttachPoints` | the frames in `publishes`, writable, created at init |
| `InternalFrames` | every other frame the item owns |
| `AlternativeFrames` | the composed poses in `alternatives` |

<!-- toml: component -->
```toml
[[component.tf]]
name = "PlatformFrames"
head = "base_link"
alternatives = [["map", "base_link"]]
exclude = ["*_wheel_link"]
publishes = [
    { base = "base_link", frame = "marker", static = true, position = [0.5, 0.0, 1.0], orientation = [0.0, 0.0, 90.0] },
]
```

```python
from opensmi.ros2.server import PublishedFrame, TFBinding

binding = TFBinding(
    name="PlatformFrames",
    head="base_link",
    alternatives=(("map", "base_link"),),
    exclude=("*_wheel_link",),
    publishes=(
        PublishedFrame(
            base="base_link",
            frame="marker",
            static=True,
            position=(0.5, 0.0, 1.0),
            orientation=(0.0, 0.0, 90.0),
        ),
    ),
)
```

The Machine declares its own frame the same way. Its head is the root of the Machine's partition:

<!-- toml: machine -->
```toml
[machine]
name = "Cell"

[machine.identification]
serial_number = "cell-0001"
manufacturer = "Example Manufacturer"
product_instance_uri = "urn:example:cell"

[ros2.transforms]
path = "/robot/tf"
static_path = "/robot/tf_static"
interval = 0.5

[[machine.tf]]
name = "CellFrames"
head = "world"
```

## One file with every kind

[`examples/machine.toml`](../examples/machine.toml) declares one Binding of every kind on a single
Component. Its Python twin is `examples/component.py`.
