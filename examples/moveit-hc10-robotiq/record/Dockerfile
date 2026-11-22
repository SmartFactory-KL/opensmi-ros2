# syntax=docker/dockerfile:1
# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT
#
# The example's image plus what recording a video takes: a virtual X server for Gazebo's GUI, ffmpeg
# to capture and cut it, and a font for the captions. Built by record.sh after the example's image.

FROM opensmi-ex-hc10:local

ARG UBUNTU_MIRROR=
RUN if [ -n "$UBUNTU_MIRROR" ]; then \
        sed -i "s#http://archive.ubuntu.com/ubuntu#$UBUNTU_MIRROR#g" /etc/apt/sources.list.d/ubuntu.sources; \
    fi \
    && apt-get update \
    && apt-get install -y --no-install-recommends xvfb ffmpeg fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*
