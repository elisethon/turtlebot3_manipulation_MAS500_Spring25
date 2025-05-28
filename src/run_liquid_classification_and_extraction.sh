#!/bin/bash

# === Run RViz locally on dev PC ===
echo "Starting RViz locally on dev PC..."
gnome-terminal -- bash -c "
  source /opt/ros/humble/setup.bash &&
  source ~/turtlebot3_ws/install/setup.bash &&
  export ROS_DOMAIN_ID=30 &&
  export TURTLEBOT3_MODEL=waffle &&
  ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py;
  exec bash"
sleep 2






# Define SSH connection (change to your bot's IP and user)
TURTLEBOT_USER="tilde"
TURTLEBOT_IP="10.245.30.146"
SSH_CMD="ssh ${TURTLEBOT_USER}@${TURTLEBOT_IP}"

# Function to open a remote gnome-terminal via SSH with full environment setup
run_remote_command() {
  local CMD="$1"
  gnome-terminal -- bash -c "${SSH_CMD} '
    source /opt/ros/humble/setup.bash &&
    source ~/turtlebot3_ws/install/setup.bash &&
    export ROS_DOMAIN_ID=30 &&
    export PATH=\$HOME/.local/bin:\$PATH &&
    export LDS_MODEL=LDS-01 &&
    export TURTLEBOT3_MODEL=waffle &&
    export OPENCR_PORT=/dev/ttyACM0 &&
    export OPENCR_MODEL=turtlebot3_manipulation &&
    cd ~/turtlebot3_ws &&
    ${CMD};
    exec bash'"
}

# Start the bringup launch in a new terminal
echo "Starting bringup in a new terminal..."
run_remote_command "ros2 launch turtlebot3_manipulation_bringup hardware.launch.py"
sleep 8
echo "Bringup in a new terminal is started"

# Start the analog sensor publisher
#echo "Starting analog sensor publisher in a new terminal..."
#run_remote_command "ros2 run analog_sensor adc_publisher"
#sleep 8
#echo "Analog sensor is publishing in a new terminal"

# Start the joint state reader
echo "Starting joint state reader in a new terminal..."
run_remote_command "ros2 run turtlebot3_manipulator_mas500 ReadJointState"
sleep 8
echo "Reading joint state in a new terminal"

# Start manipulator movement
echo "Starting manipulator movement in a new terminal..."
run_remote_command "ros2 run turtlebot3_manipulator_mas500 sensor_Test_v3"
sleep 65
echo "Testing liquid in a new terminal"

# Start the Qtip test if joint_position.txt exists and is not empty
echo "Checking if joint_position.txt exists and is not empty..."
gnome-terminal -- bash -c "${SSH_CMD} '
  source /opt/ros/humble/setup.bash &&
  source ~/turtlebot3_ws/install/setup.bash &&
  export ROS_DOMAIN_ID=30 &&
  export PATH=\$HOME/.local/bin:\$PATH &&
  export LDS_MODEL=LDS-01 &&
  export TURTLEBOT3_MODEL=waffle &&
  export OPENCR_PORT=/dev/ttyACM0 &&
  export OPENCR_MODEL=turtlebot3_manipulation &&
  cd ~/turtlebot3_ws &&
  if [ -s ./captured_joint_positions/joint_position.txt ]; then
    ros2 run turtlebot3_manipulator_mas500 Qtip_Test_P3_v5;
  else
    echo \"\"
    echo \"joint_position.txt is empty, skipping Qtip test\";
  fi;
  exec bash'"

echo "Everything has run successfully"




# === Wait for user to type a shutdown keyword ===
echo ""
echo "👉 Type 'kill_all_terminals' and press Enter to close all opened gnome-terminals"
read -r USER_INPUT

if [[ "$USER_INPUT" == "kill_all_terminals" ]]; then
    echo "🔻 Killing all gnome-terminal windows..."
    pkill -f gnome-terminal
else
    echo "⚠️ Unknown input. Terminals not closed."
fi

