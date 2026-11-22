<!--
SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.

SPDX-License-Identifier: MIT
-->

# Security policy

OpenSMI-ROS2 is pre-1.0. Only the latest release gets security fixes.

Report a vulnerability privately, through
[GitHub's private vulnerability reporting](https://github.com/SmartFactory-KL/opensmi-ros2/security/advisories/new)
or by mail to <benjamin.blumhofer@smartfactory.de>. Do not open a public issue for it. Expect an
answer within a week.

Read [Deployment](docs/deployment.md) before you run a server next to a robot: opensmi's defaults
(unencrypted endpoint, passwords equal to user names, all interfaces) are for development, and the
ROS 2 graph itself is not authenticated. OpenSMI-ROS2 guarantees nothing safety-relevant.
