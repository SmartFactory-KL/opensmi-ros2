# Testing

Install the development tools, then run the suite in parallel:

```sh
python -m pip install -c constraints.txt -e . --group dev
pytest -n auto
```

Tests that need a live ROS 2 Jazzy graph are marked `jazzy` and deselected by default. Run them with
ROS 2 sourced and the interface packages `example_interfaces`, `control_msgs` and
`controller_manager_msgs` installed:

```sh
pytest -m jazzy
```

Timing measurements are marked `perf` and run with `pytest -m perf -s`. CI also runs ruff,
basedpyright, `reuse lint` and a check of the definition catalog, listed in
[CONTRIBUTING.md](../CONTRIBUTING.md).

The examples that run a simulation (`examples/nav2-rox-argo`, `examples/moveit-hc10-robotiq`) are
built and run by `.github/workflows/examples.yml` and are not part of the suite. Each one runs its
scenario against a simulated robot and writes a report; see the [examples index](../examples/README.md).
