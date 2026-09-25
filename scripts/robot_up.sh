#!/bin/bash
# Start/stop the base stack on the robot in the background: robot_up.sh start|stop [launch args]
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash
case "$1" in
  stop)
    [ -f ~/ugv_base.pid ] && kill -INT -- -"$(cat ~/ugv_base.pid)" 2>/dev/null
    sleep 3; rm -f ~/ugv_base.pid; echo stopped ;;
  start)
    shift
    setsid ros2 launch ugv_bringup base.launch.py "$@" > ~/ugv_base.log 2>&1 < /dev/null &
    echo $! > ~/ugv_base.pid; echo "started pid $(cat ~/ugv_base.pid)" ;;
esac
