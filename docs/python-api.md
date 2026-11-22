# Writing a server in Python

The Python API declares the same things as the TOML file. It also covers what a file cannot express,
such as your own Skill classes or logic between declarations. This server has one Component that
reads the `std_msgs/String` topic `/chatter`:

```python
import asyncio

from asyncua import ua
from opensmi.server import BaseComponent, BaseMachine, Server

from opensmi.ros2.runtime import Runtime
from opensmi.ros2.server import MonitoringBinding, ROS2ComponentMixin, ROS2MachineMixin


class Talker(ROS2ComponentMixin, BaseComponent):
    bindings = (MonitoringBinding(name="Chatter", path="/chatter", message_type="std_msgs/msg/String"),)

    async def _write_identification(self) -> None:
        await self.identification.SerialNumber.write("talker-0001")
        await self.identification.Manufacturer.write(ua.LocalizedText("Example", "en-US"))


class Robot(ROS2MachineMixin, BaseMachine):
    async def _write_identification(self) -> None:
        await self.identification.SerialNumber.write("robot-0001")
        await self.identification.ProductInstanceUri.write("urn:example:robot")
        await self.identification.Manufacturer.write(ua.LocalizedText("Example", "en-US"))

    async def _init(self) -> None:
        await super()._init()
        await self.add_component(Talker(name="Talker"))


async def main() -> None:
    async with Server() as server:
        await server.add_machine(Robot(runtime=Runtime(domain_id=0)))
        await server.start(blocking=True)


asyncio.run(main())
```

Run `ros2 run demo_nodes_py talker` next to it, and `Talker/Monitoring/Chatter/Data` changes with
every message. The mixins go before the opensmi base class. Your application creates the Runtime and
hands it to the Machine, which starts it before anything else is built and stops it at the end.

The other Binding classes work the same way: `MethodBinding`, `ActionBinding`, `PublisherBinding`,
`ROS2ParameterBinding`, `LifecycleBinding`, `KeyedBinding` and `TFBinding`, all importable from
`opensmi.ros2.server`. `opensmi.ros2.server.load_machine` loads a machine file from Python.

The reference is in the docstrings, so `help()` and your editor show it:

- `opensmi.ros2`: the mapping, the requirements and a minimal server.
- `opensmi.ros2.server`: the public names and what a client sees.
- `opensmi.ros2.server.bindings`, `.component`, `.machine`: declaring Bindings, Components and
  Machines.
- `opensmi.ros2.server.loader` and `opensmi.ros2.cli`: the machine file and the command line.
- `opensmi.ros2.server.access`: access levels and the Lock.
- `opensmi.ros2.runtime`: the ROS 2 half, which knows nothing about OPC UA.
