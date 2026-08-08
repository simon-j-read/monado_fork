#!/bin/bash
# Script to configure diy vr display while using X11 on CachyOS
echo ---------------------------------------------
echo     X11 DIY VR Display Configuration
echo ---------------------------------------------

echo Assumptions:
echo 1. Output is for 2880x1440 60Hz output

echo Steps:
echo 1. Create new mode
echo 2. Add mode to VR Display output
echo 3. Set mode to VR display output
echo 4. Set VR display output to non-desktop to give Monado rights to display

echo ---------------------------------------------
echo Monitors connected:
xrandr --listmonitors
echo Choose a monitor for VR configuration
read -p "Name:" name

# Output from "cvt -r 2880 1440 60" -r for reduced blinking
xrandr --newmode "2880VR"  270.00  2880 2928 2960 3040  1440 1443 1453 1481 +hsync -vsync
xrandr --addmode $name        "2880VR"      # add new mode to the VR display output usable options
xrandr --output  $name --mode "2880VR"      # Set the mode to new mode
xrandr --output  $name --set  "non-desktop" 1  # Make display output non-desktop so Monado can be given "rights to use it."
