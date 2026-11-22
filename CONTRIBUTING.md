# Contributing

## Setup

Work in a clone, in a virtual environment. For the tests that need ROS 2, create it with
`--system-site-packages` after sourcing ROS 2 Jazzy, as the README shows. Then install the package
with the development tools:

```
python -m pip install --upgrade pip
python -m pip install -c constraints.txt -e . --group dev
```

`--group` needs pip 25.1 or newer, hence the upgrade first.

pyproject.toml holds version ranges, which is what a released package needs. constraints.txt pins
the exact opensmi releases that development and CI install, all from PyPI (today opensmi-server
0.4.1 and opensmi-core 0.8.1). Raise a pin in its own merge request, together with a green run of the
[opensmi main](.github/workflows/opensmi-main.yml) workflow, which runs the tests against the current
opensmi-server and opensmi-core main branches every Monday.

## Checks

CI runs these checks, and so should you before pushing:

```
ruff check .
ruff format --check .
reuse lint
basedpyright
pytest -n auto
```

`ruff format --check` also checks Python code blocks in Markdown files. basedpyright checks `src`,
`tests`, `tools` and `examples`, so a type error in a test fails the pipeline too. The runtime half
(`opensmi.ros2.runtime`) never imports opensmi or asyncua; `tests/test_import_boundary.py` enforces
that.

Tests that need a live ROS 2 Jazzy graph are marked `jazzy` and deselected by default. Source ROS 2
Jazzy, install the interface packages `example_interfaces`, `control_msgs` and
`controller_manager_msgs`, and run `pytest -m jazzy`. On WSL, set `FASTDDS_BUILTIN_TRANSPORTS=SHM`.

## The public surface

The public surface is what a server author touches to declare or start a server. The docstrings of
`opensmi.ros2.server` and `opensmi.ros2.runtime` list it. Every other name stays importable from its
module but may change in any release. The error classes are public through
`opensmi.ros2.server.errors` and `opensmi.ros2.runtime.errors`.

A name joins the surface when a server author needs it to write a declaration, a custom driver or a
custom Projection. A name leaves it by being removed from `__all__`, after a rename or a deprecation
if it was in a release. Make either change in one commit that edits `__all__`, the expected lists in
`tests/test_public_surface.py` and, if a user would notice, the docstring and the README. That test
also checks that every public name imports and is documented where it is defined.

## The definition catalog

`catalog/` holds verbatim ROS 2 interface definitions for packages that cannot be installed, and
`tools/catalog/` generates the stubs in `src/opensmi/ros2/runtime/messages/` from them. CI runs
`python -m tools.catalog_check`. See [catalog/README.md](catalog/README.md).

## Before a tag

The version comes from the git tag through setuptools-scm. Tags are `vX.Y.Z` (pre-releases such as
`v0.1.0rc1`), and a tag cannot be moved. Before tagging, run every check above, basedpyright
included, and the `jazzy` tests (`pytest -m jazzy`).

## Commits

Use [Conventional Commits](https://www.conventionalcommits.org/). The changelog is generated from the
history with [git-cliff](https://git-cliff.org/), so the commit message is the release note.

```
<type>(<optional scope>): <summary in the imperative, lower case, no trailing period>

<optional body: what and why>

<optional footers, e.g. BREAKING CHANGE: ...>
```

Types: `feat`, `fix`, `perf`, `refactor`, `docs`, `test`, `build`, `ci`, `style`, `revert`, `chore`.
Mark a breaking change with `!` after the type or scope, or with a `BREAKING CHANGE:` footer. A change
that breaks a client also gets an entry in [docs/migration.md](docs/migration.md).

Preview the unreleased changelog with `git cliff --unreleased`. When releasing, regenerate it with
`git cliff --tag vX.Y.Z -o CHANGELOG.md`. The configuration is in cliff.toml.
