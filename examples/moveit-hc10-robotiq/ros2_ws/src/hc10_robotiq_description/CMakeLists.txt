# SPDX-FileCopyrightText: 2026 Technologie-Initiative SmartFactory KL e.V.
#
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.8)
project(hc10_robotiq_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch urdf worlds DESTINATION share/${PROJECT_NAME})
ament_package()
