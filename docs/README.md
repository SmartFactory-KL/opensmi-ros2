# Documentation

The [README](../README.md) is the short version. These pages hold the detail.

| Page | What it answers |
| --- | --- |
| [Getting started](getting-started.md) | How do I install it, serve a Machine and stop it? |
| [Client quickstart](client-quickstart.md) | How does an OPC UA client log in, take the Lock, start a Skill and read the errors? |
| [Architecture](architecture.md) | How is it built, and which ROS 2 interface becomes which OPC UA object? |
| [Configuration](configuration.md) | What does the machine file take, and how do I set the ROS 2 and server options? |
| [Writing a server in Python](python-api.md) | How do I declare a Machine in code, and where is the API reference? |
| [Bindings](bindings.md) | What does each of the eight Binding kinds map, declare, validate and create? |
| [Browse paths](browse-paths.md) | How do ROS 2 names turn into OPC UA browse names? |
| [Publishing Bindings](publishing-bindings.md) | How does a topic become a Skill, and when does a stream halt? |
| [Deployment](deployment.md) | What must I set before a robot in a shared network depends on it? |
| [Testing](testing.md) | How do I run the tests? |
| [Migration](migration.md) | Which changes break a client, and what do I change? |

Elsewhere in the repository:

- [`examples/`](../examples/README.md): runnable example Machines, two of them complete simulated robots.
- [CONTRIBUTING.md](../CONTRIBUTING.md) covers the setup, the checks CI runs and the commit conventions.
- [catalog/README.md](../catalog/README.md) describes the definition catalog, the vendored ROS 2
  interface definitions behind the shipped type stubs.

The reference for the Python API is in the docstrings, so `help()` and your editor show it. The
list of modules is in [Writing a server in Python](python-api.md).
