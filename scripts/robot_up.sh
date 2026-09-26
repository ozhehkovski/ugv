#!/bin/bash
# Control the robot stack (systemd user service `ugv`, autostarts at boot).
#   robot_up.sh install   copy the unit, enable autostart, start now
#   robot_up.sh start|stop|restart|status
#   robot_up.sh log       follow ~/ugv_base.log
set -euo pipefail
UNIT_SRC="$(dirname "$0")/systemd/ugv.service"
case "${1:-status}" in
  install)
    mkdir -p ~/.config/systemd/user
    cp "$UNIT_SRC" ~/.config/systemd/user/ugv.service
    loginctl enable-linger "$USER"
    systemctl --user daemon-reload
    systemctl --user enable --now ugv.service
    systemctl --user --no-pager status ugv.service | head -5 ;;
  start|stop|restart)
    systemctl --user "$1" ugv.service ;;
  status)
    systemctl --user --no-pager status ugv.service | head -8 ;;
  log)
    tail -f ~/ugv_base.log ;;
  *)
    echo "usage: $0 install|start|stop|restart|status|log" >&2; exit 2 ;;
esac
