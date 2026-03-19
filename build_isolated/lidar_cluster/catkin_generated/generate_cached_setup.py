# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/melodic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/melodic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/RZS/A06_ws/devel_isolated/lidar_check;/home/RZS/A06_ws/devel_isolated/lego_loam;/home/RZS/A06_ws/devel_isolated/imu_load;/home/RZS/A06_ws/devel_isolated/fusion;/home/RZS/A06_ws/devel_isolated/control;/home/RZS/A06_ws/devel_isolated/boundary_detector;/home/RZS/A06_ws/devel_isolated/fsd_tools;/home/RZS/A06_ws/devel_isolated/estimationState;/home/RZS/A06_ws/devel_isolated/can_interaction;/home/RZS/A06_ws/devel_isolated/fsd_common_msgs;/home/RZS/A06_ws/devel_isolated/darknet_ros;/home/RZS/A06_ws/devel_isolated/darknet_ros_msgs;/home/RZS/A06_ws/devel_isolated/daheng;/home/RZS/A06_ws/devel_isolated/coordSystem;/home/RZS/A06_ws/devel_isolated/cloud_msgs;/opt/ros/melodic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/RZS/A06_ws/devel_isolated/lidar_cluster/env.sh')

output_filename = '/home/RZS/A06_ws/build_isolated/lidar_cluster/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
