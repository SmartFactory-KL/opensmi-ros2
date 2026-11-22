# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

"""Third-party source vendored verbatim, and never edited.

One file lives here: ``rosidl_adapter_parser.py``, which is ``rosidl_adapter/parser.py`` from
``ros2/rosidl`` at the commit ``catalog/sources.toml`` pins. It is vendored rather than depended
on because ``rosidl-adapter`` is not published on PyPI, and it is vendored rather than rewritten
because it is the parser ROS2 itself reads ``.msg`` text with.

**Nothing in this directory is edited, including to add a licence header.** Writing an SPDX header
into a vendored file changes its bytes, and byte-identical re-extraction is what makes the pin
checkable -- the two are mutually exclusive, so the annotation lives in ``REUSE.toml`` instead.
The same reason keeps this directory out of ruff's and basedpyright's reach, configured in
``pyproject.toml``: a lint rule that can only be satisfied by editing the file is a lint rule
pointed at the wrong thing.
"""
