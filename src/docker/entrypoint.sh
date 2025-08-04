#!/bin/bash

# Source ROS and the Barney workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ ! -f /Barney/install/setup.bash ]
then
  colcon build
fi
source /Barney/install/setup.bash

# Add dependencies to path
PDDLSTREAM_PATH=/Barney/src/dependencies/pddlstream
if [ -d "$PDDLSTREAM_PATH" ]
then
    export PYTHONPATH=$PDDLSTREAM_PATH:$PYTHONPATH
fi

# Execute the command passed into this entrypoint
exec "$@"
