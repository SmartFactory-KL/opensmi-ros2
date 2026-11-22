# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""The definition catalog: interface definitions for packages that cannot be installed.

The catalog holds verbatim ``.msg``, ``.srv`` and ``.action`` text for the interface packages in
required scope that have **no binary release** -- currently four -- pinned by commit SHA in
``catalog/sources.toml``. It exists because a Component author cannot type against a package
nobody can install, and because a Projection for such a package could otherwise never be tested:
:func:`opensmi.ros2.runtime.messages.build_message_dataclass` takes a description rather than an
``rclpy`` object, so CI fabricates the dataclass from catalog text and runs the Projection against
it.

**It lives here rather than under ``src/`` on purpose.** The catalog is read while authoring and
testing and **never while serving**: a definition it holds can make a message *describable*, never
*receivable*, because ``rclpy`` cannot subscribe to a type whose typesupport is absent. Keeping it
out of the distribution makes that a structural fact instead of a rule someone has to remember --
nothing importable from ``opensmi.ros2`` can reach it. What ships is the *structure*, as the
generated stubs in :mod:`opensmi.ros2.runtime.messages`.

**One normalised definition type, and the catalog is a producer of it rather than a second type.**
:class:`~opensmi.ros2.runtime.messages.MessageDescription` is owned by the runtime half; this
package produces one from definition text exactly as :mod:`opensmi.ros2.runtime._parser` produces
one from an installed interface class, and a Jazzy CI job asserts the two agree for every
installed interface package. If they ever disagree we learn it on ``sensor_msgs``, not on a vendor
family nobody can check.
"""
