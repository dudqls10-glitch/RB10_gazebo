#!/usr/bin/env python3
import subprocess
import time
import os
import sys

WORLD_NAME = "empty"
ROBOT_MODEL = "rb10"
ROBOT_LINK = "link6"

TOF_MODEL = "tof_sensor"
TOF_LINK = "tof_link"

TOF_SDF = os.path.expanduser(
    "~/rb10_gazebo_ws/src/rbpodo_ros2/rbpodo_description/robots/sensors/tof.sdf"
)

def run(cmd: str):
    print(f"[CMD] {cmd}")
    ret = subprocess.run(cmd, shell=True)
    if ret.returncode != 0:
        print(f"[ERROR] command failed")
        sys.exit(1)

def spawn_tof():
    cmd = f"""
ign service -s /world/{WORLD_NAME}/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 2000 \
  --req "
sdf_filename: '{TOF_SDF}'
name: '{TOF_MODEL}'
"
"""
    run(cmd)

def attach_tof():
    cmd = f"""
ign service -s /world/{WORLD_NAME}/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 2000 \
  --req "
joint {{
  name: 'tof_fixed_joint'
  parent: '{ROBOT_MODEL}::{ROBOT_LINK}'
  child: '{TOF_MODEL}::{TOF_LINK}'
  type: FIXED
  pose {{
    position {{ x: 0.03 y: 0 z: 0 }}
  }}
}}
"
"""
    run(cmd)

if __name__ == "__main__":
    print("[INFO] Waiting for Gazebo to be ready...")
    time.sleep(3.0)

    print("[INFO] Spawning ToF sensor...")
    spawn_tof()

    time.sleep(1.0)

    print("[INFO] Attaching ToF sensor to robot...")
    attach_tof()

    print("[INFO] ToF sensor successfully attached.")

