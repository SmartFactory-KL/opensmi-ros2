# Client quickstart

This page walks an OPC UA client through a served Machine: log in, browse by path, take the Lock,
start a Skill, read its feedback and result, call a Method, and read the errors the server answers
with. The client is [`examples/client/quickstart.py`](../examples/client/quickstart.py), a plain
asyncua script that imports nothing from OpenSMI-ROS2. It exits with status 1 as soon
as a step does not go as described here, and the tests in
[`tests/test_client_quickstart.py`](../tests/test_client_quickstart.py) run its steps against the same
Machine on every change.

You need the setup of [Getting started](getting-started.md), and the two ROS 2 demo nodes below. Both
ship with the ROS 2 Jazzy desktop installation.

## The Machine

[`examples/client/machine.toml`](../examples/client/machine.toml) declares one Machine, `Demo`, with
one Component, `Workcell`, and four Bindings:

| Binding | ROS 2 side | OPC UA side |
| --- | --- | --- |
| `Fibonacci` | action `/fibonacci` (`action_tutorials_interfaces/action/Fibonacci`) | a finite Skill: the goal `order` is its Parameter, the feedback `partial_sequence` its Monitoring variable, the result `sequence` its FinalResultData |
| `AddTwoInts` | service `/add_two_ints` (`example_interfaces/srv/AddTwoInts`) | a Method: the request fields `a` and `b` are its Parameters, the response `sum` its FinalResultData |
| `Unserved` | service `/no_such_service`, which nobody serves | a Method that cannot work |
| `UnservedFibonacci` | action `/no_such_action`, which nobody serves | a Skill that cannot work |

The last two exist to show what a client sees when the ROS 2 side is missing. Both wait one second for
a server instead of the default ten.

## Run it

Start the two demo nodes, each in a shell with ROS 2 sourced. They have to be on the Domain ID the
machine file sets, which is 42:

```sh
. /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
ros2 run action_tutorials_py fibonacci_action_server
```

```sh
. /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
ros2 run demo_nodes_cpp add_two_ints_server
```

On WSL, set `FASTDDS_BUILTIN_TRANSPORTS=SHM` in every shell, here and below. In a third shell, from the
repository root and with the virtual environment of Getting started active, serve the Machine:

```sh
opensmi-ros2 serve examples/client/machine.toml
```

In a fourth shell, with the same virtual environment, run the client:

```sh
python examples/client/quickstart.py
```

The client connects to `opc.tcp://localhost:4841/server`. If the server listens elsewhere, for
instance because you moved it with a server configuration file (see
[Configuration](configuration.md)), pass `--endpoint opc.tcp://host:port/server`.

This is the output of a real run. The server in that run listened on port 4851, so the two lines that
name the endpoint read 4851, where a default configuration reads 4841. The Fibonacci node sends feedback
about once a second, so the run takes about ten seconds:

```text
connected to opc.tcp://localhost:4851/server as orchestrator
browsed Machines/Demo/Components/Workcell: Skills ['Fibonacci', 'UnservedFibonacci'], Methods ['AddTwoInts', 'Unserved']
without the Lock:
  Reset on Fibonacci: BadRequiresLock
  Call on AddTwoInts: BadRequiresLock
took the Lock as orchestrator
started Fibonacci with Order = 6
  feedback PartialSequence = [0, 1, 1]
  feedback PartialSequence = [0, 1, 1, 2]
  feedback PartialSequence = [0, 1, 1, 2, 3]
  feedback PartialSequence = [0, 1, 1, 2, 3, 5]
  feedback PartialSequence = [0, 1, 1, 2, 3, 5, 8]
  result Sequence = [0, 1, 1, 2, 3, 5, 8]
called AddTwoInts: 2 + 40 = 42
a service nobody serves:
  Call on Unserved: BadNotConnected
an action nobody serves:
  Start was accepted; the Skill went to Halted after the server found no action server
gave the Lock back
connected to opc.tcp://localhost:4851/server as planner
planner holds the Lock but is below the Skill access level:
took the Lock as planner
  Reset on Fibonacci: BadUserAccessDenied
gave the Lock back
every step went as described
```

The exit status is 0 here. A step that goes differently prints `FAILED: ...` to stderr and the status
is 1.

## The steps

**Connect and log in.** opensmi's default users have their name as password. Each has a maximum
access level, and an action needs at least the level of the user's role:

| User | Access level | Can do |
| --- | --- | --- |
| `visitor` | 0 | Browse and read. Several sessions at once are allowed |
| `planner` | 5 | Take the Lock, and nothing that needs level 10 |
| `orchestrator` | 10 | Start, halt and reset Skills, call Methods, write Parameters, all with the Lock |
| `remote` | 15 | As `orchestrator` |
| `operator` | 20 | As `orchestrator`, and the introspection Methods that change something (`BindTopic`, `CallService`) |
| `watchdog` | 20 | As `operator` |
| `developer` | 100 | Everything |

`visitor` is the only user with `allow_multiple`; every other user holds one session. The endpoint is
unencrypted and the password travels as it is, which suits a development machine. For anything else see
[Deployment](deployment.md). The levels a Skill, a Method and the introspection Methods ask for come
from the `[ros2]` table ([Configuration](configuration.md)); the table above shows the defaults.

**Browse by path.** The script follows browse names from the Objects folder: `Machines`, `Demo`,
`Components`, `Workcell`. Below the Component, `SkillSet` holds the Skills and `MethodSet` the Methods.
A Skill's variables are below `SkillExecution`: `ParameterSet`, `Monitoring`, `FinalResultData` and the
`StateMachine` with the Methods `Start`, `Halt` and `Reset`. A Method has `ParameterSet`,
`FinalResultData` and the Method `Call`. The names ROS 2 gives a field are converted
(`partial_sequence` is `PartialSequence`); [Browse paths](browse-paths.md) has the rules. Find nodes by
these names and never store a NodeId.

**Take the Lock.** The Machine has a `Lock` object. Calling its `InitLock` with a short text gives the
session the Lock, and `ExitLock` gives it back. Every Skill, Method and Parameter write needs the Lock in
addition to the access level. Closing the session does not give the Lock back, so the script calls
`ExitLock` in a `finally`. opensmi releases a Lock after ten minutes without a call to `InitLock`,
`BreakLock` or `RenewLock`, and a variable write does not count, so a client that holds the Lock longer
calls `RenewLock` at least every ten minutes.

If a session of the same user closes just before the script runs again, opensmi can still be sweeping
it away, and a Lock taken in that moment is given up with the old session. The sweep runs once a
second. If the next call answers `BadRequiresLock`, run the script again.

**Start a Skill.** A Skill starts `Halted`. The script calls `Reset`, which makes it `Ready`, writes the
goal into `ParameterSet/Order`, and calls `Start`. The state is `Starting` while the server sends the
goal, `Running` once the ROS 2 action server accepted it and `Completed` when the goal succeeded. Any
other ending, such as an aborted, cancelled or rejected goal, leaves the Skill `Halted`. The state is
the variable `StateMachine/CurrentState`. The script polls it and the feedback variable
`Monitoring/PartialSequence` every 0.2 seconds, and reads `FinalResultData/Sequence` at the end. To
run the Skill again it calls `Reset` first, which clears Monitoring and FinalResultData and leaves the
Parameters as they are.

**Call a Method.** The request is written into the Method's `ParameterSet`, and `Call` takes no
arguments of its own. The response is in `FinalResultData` when `Call` returns. A Method has no
Monitoring, because a service has no feedback.

## Error states

A refused call reaches the client as an asyncua `UaStatusCodeError` whose code is the status below. The
quickstart shows the first five rows against a running server. The others follow from the sources.

| What happened | Status code | Cause |
| --- | --- | --- |
| `Reset`, `Start` or `Halt` of a Skill, or `Call` of a Method, without the Lock | `BadRequiresLock` | The session does not hold the Machine's Lock. The Lock is also gone after ten minutes without renewal |
| A write to a Parameter without the Lock | `BadUserAccessDenied` | A variable write has no status code for a missing Lock: the server answers as it does to a user below the variable's level. Take the Lock first |
| `Reset` of a Skill, or `Call` of a Method, by a user below the Skill level | `BadUserAccessDenied` | The access level gate is separate from the Lock. `planner` holds the Lock and is refused. `skill_access_level` sets the level, 10 by default |
| `Start` while the Skill is `Running`, or while it is `Halted` | `BadInvalidState` | opensmi's state machine has no such transition. A Skill is `Halted` at first, so `Reset` comes before `Start` |
| `Call` of a Method whose ROS 2 service has no server | `BadNotConnected` | No service server answered within `server_timeout` seconds (default 10). The request was not sent |
| `Call` of a Method whose server took the request and did not answer | `BadTimeout` | No answer within `call_timeout` seconds (default 30) |
| `Call` of a Method that declares `success_field`, when the service answered `false` there | `BadRequestNotAllowed` | The service ran and refused. The response is in FinalResultData, and the service's own reason text is usually in it. A Method without `success_field` answers `Good` whatever the service says |
| `Call` of a Method that failed inside the server for any other reason | `BadInternalError` | The server's log has the reason |
| `Start` or `Call` in a Component whose StartupSkill is `Halted` or never ran | `BadInvalidState` | opensmi refuses Skills and Methods until the startup completed. A Component has a StartupSkill when it has a required Binding or a lifecycle Binding, and the Machine always has one |
| `Start` of a Skill whose ROS 2 action has no server | `Good`, then the Skill goes to `Halted` | `Start` is accepted and the server then waits `server_timeout` seconds for an action server. The error `ActionServerUnavailableError` is reported through the Component's Notification and the server log |
| `Start` of a Skill whose ROS 2 action server rejects the goal | `Good`, then `Halted` | The error code `GoalRejected` is reported through the Notification |
| `Start` with Parameters the server cannot turn into a goal | `Good`, then `Halted` | The error code `GoalUnbuildable` is reported through the Notification. The goal never reaches ROS 2 |
| A write of a value of the wrong data type to a Parameter | `BadTypeMismatch` | The server checks the variable's declared type. Write with the variant type the node declares; the script reads it first |
| Logging in with a wrong user name or password | `BadUserAccessDenied` | Raised when the session is activated, before any browsing |

A Component's StartupSkill halts when a Binding marked `required` is still absent after its
`startup_timeout`, and the Machine's StartupSkill halts when a Component's did not come up. The
Notification then carries the error codes `REQUIRED_BINDING_ABSENT` and `COMPONENT_NOT_RUNNING`. To
recover, call `Reset` and `Start` on the StartupSkill, with the Lock, once the ROS 2 side is there. A
Binding lost after a successful startup is reported as a warning and never halts anything.

The alerts are OPC UA events on the Notification object of the Machine or the Component. The
quickstart does not subscribe to them. The server log has the same text.
