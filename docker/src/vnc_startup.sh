#!/bin/bash
set -e

# Shutdown signal
cleanup () {
    kill -s SIGTERM $!
    exit 0
}
trap cleanup SIGINT SIGTERM

## VNC password
mkdir -p "$HOME/.vnc"
echo "$VNC_PW" | vncpasswd -f >> $HOME/.vnc/passwd

# noVNC
$NO_VNC_HOME/utils/novnc_proxy --vnc localhost:$VNC_PORT --listen $NO_VNC_PORT &
PID_SUB=$!

# VNC
vncserver -kill $DISPLAY &> $STARTUPDIR/vnc_startup.log \
    || rm -rfv /tmp/.X*-lock /tmp/.X11-unix &> $STARTUPDIR/vnc_startup.log \
    || echo "No locks present"
vncserver $DISPLAY -depth $VNC_COL_DEPTH -geometry $VNC_RESOLUTION PasswordFile=$HOME/.vnc/passwd &

# XFCE
$HOME/wm_startup.sh &
wait $PID_SUB
