#!/bin/bash

# -------------------------------------------------------------------
# Baxter Startup Script
# This script sets up the Baxter workspace, enables the robot, untucks
# arms, and runs save_ears.sh to disable sonar sensors.
# -------------------------------------------------------------------

# Move to Baxter workspace
cd ~/baxter_ws || { echo "Failed to cd to ~/baxter_ws"; exit 1; }

# Source Baxter environment
./baxter.sh

# Reset robot if E-stop was pushed
rosrun baxter_tools enable_robot.py -r

# Enable the robot
rosrun baxter_tools enable_robot.py -e

# Untuck arms (ensure there is space)
rosrun baxter_tools tuck_arms.py -u

# Move back to workspace folder (optional, safe practice)
cd ~/baxter_ws || { echo "Failed to cd to ~/baxter_ws"; exit 1; }

# Disable sonar sensors
./save_ears.sh

echo "Baxter startup sequence complete!"
