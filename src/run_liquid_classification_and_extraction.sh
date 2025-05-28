#!/bin/bash

# THIS SHELL ASSUMES THAT THIS REPOSITORY IS CLONED AND SET UP ON BOTH DEV PC AND ON THE TURTLEBOT
# If this is not correct, change the ws and folder structure for it to be able to run the scripts 
# Also assumes TurtleBot3 Waffle Pi with LDS-01, change if not correct


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


# === Define SSH conn params ===

TURTLEBOT_USER="tilde" # CHANGE IF DIFFERENT USER ON TURTLEBOT
TURTLEBOT_IP="10.245.30.146" # CHANGE FOR IP ON TURTLEBOT
SSH_CMD="ssh ${TURTLEBOT_USER}@${TURTLEBOT_IP}"


# === Use SSH conn ===
# Sets params for TurtleBot3 Waffle Pi with LDS-01, change if not correct

run_remote_command() {
  local CMD="$1"
  gnome-terminal -- bash -c "${SSH_CMD} '
    source /opt/ros/humble/setup.bash &&
    source ~/turtlebot3_manipulation_MAS500_Spring25/install/setup.bash &&
    export ROS_DOMAIN_ID=30 &&
    export PATH=\$HOME/.local/bin:\$PATH &&
    export LDS_MODEL=LDS-01 &&
    export TURTLEBOT3_MODEL=waffle &&
    export OPENCR_PORT=/dev/ttyACM0 &&
    export OPENCR_MODEL=turtlebot3_manipulation &&
    cd ~/turtlebot3_manipulation_MAS500_Spring25 &&
    ${CMD};
    exec bash'"
}


# Start the bringup in a new terminal

echo "Starting bringup in a new terminal..."
run_remote_command "ros2 launch turtlebot3_manipulation_bringup hardware.launch.py"
sleep 8
echo "Bringup in a new terminal is started"


# Start the analog sensor publisher

echo "Starting analog sensor publisher in a new terminal..."
run_remote_command "ros2 run analog_sensor adc_publisher"
sleep 8
echo "Analog sensor is publishing in a new terminal"


# Start the joint state reader

echo "Starting joint state reader in a new terminal..."
run_remote_command "ros2 run turtlebot3_manipulator_mas500 ReadJointState"
sleep 8
echo "Reading joint state in a new terminal"


# Start manipulator movement

echo "Starting manipulator movement in a new terminal..."
run_remote_command "ros2 run turtlebot3_manipulator_mas500 Sensor_liquid_clasification"
sleep 65
echo "Testing liquid in a new terminal"


# Start the q-tip test if joint_position.txt exists and is not empty
# Sets params for TurtleBot3 Waffle Pi with LDS-01, change if not correct

echo "Checking if joint_position.txt is empty..."
gnome-terminal -- bash -c "${SSH_CMD} '
  source /opt/ros/humble/setup.bash &&
  source ~/turtlebot3_manipulation_MAS500_Spring25/install/setup.bash &&
  export ROS_DOMAIN_ID=30 &&
  export PATH=\$HOME/.local/bin:\$PATH &&
  export LDS_MODEL=LDS-01 &&
  export TURTLEBOT3_MODEL=waffle &&
  export OPENCR_PORT=/dev/ttyACM0 &&
  export OPENCR_MODEL=turtlebot3_manipulation &&
  cd ~/turtlebot3_manipulation_MAS500_Spring25 &&
  if [ -s ./captured_joint_positions/joint_position.txt ]; then
    ros2 run turtlebot3_manipulator_mas500 Qtip_liquid_sample;
  else
    echo \"\"
    echo \"joint_position.txt is empty, skipping q-tip test\";
  fi;
  exec bash'"

echo "Everything has run successfully"




# === Wait for user to type a shutdown keyword ===
echo ""
echo "Type 'kill_all_terminals' and press Enter to close all opened gnome-terminals"
read -r USER_INPUT

if [[ "$USER_INPUT" == "kill_all_terminals" ]]; then
    echo "Killing all gnome-terminal windows..."
    pkill -f gnome-terminal
else
    echo "Unknown input. Terminals not closed."
fi

