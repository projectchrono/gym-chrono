#!/bin/bash
set -e

xset -dpms &
xset s noblank &
xset s off &

/usr/bin/startxfce4 --replace &
