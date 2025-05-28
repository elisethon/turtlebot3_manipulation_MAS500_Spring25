This repository is a part of the documentation for MAS500 spring 2025 for group 3. 
The repository is forked from the repository combining the TurtleBot3 Waffle Pi robot platform with the OpenManipulator-X manipulator. 
This was done to share the changes made during development for MAS500 for the use of the robot platform and manipulator for autonomous deployment and liquid classification. 


From the original repository, some changes have been made, mainly those in the table below. 


| **Name of Script**                   | **Description of Changes Made**                                           | **Location in GitHub Repository**                   |
| ------------------------------------ | ------------------------------------------------------------------------- | --------------------------------------------------- |
| `turtlebot3_manipulation.urdf.xacro` | Changed the position and orientation of the manipulator on the TurtleBot3 | `src/turtlebot3_manipulation_description/urdf/`     |
| `open_manipulator_x.urdf.xacro`      | Added more pre-defined positions for the manipulator                      | `src/turtlebot3_manipulation_description/urdf/`     |
| `turtlebot3_manipulation.srdf`       | Changed the limits of the opening of the gripper                          | `src/turtlebot3_manipulation_moveit_config/config/` |


In addition to this, some new files have been made and added. This includes the package *read_analog_sensor* with the execution script *adc_publisher.py*. An overview can be seen in the table below. 


| **Name of script**                 | **Description of function**                                                            | **Location in this GitHub repository**    |
| ---------------------------------- | -------------------------------------------------------------------------------------- | ----------------------------------------- |
| `Qtip_liquid_sample.cpp`           | Manipulator grabs the q-tip from test tube and takes a sample of liquid                | `src/turtlebot3_manipulator_mass500/scr/` |
| `ReadJointState.cpp`               | Saves the joint state if the detected liquid is outside the ranges for classification  | `src/turtlebot3_manipulator_mass500/scr/` |
| `Sensor_detect_liquid.cpp`         | Manipulator takes the sensor and several tests to locate the liquid relative to itself | `src/turtlebot3_manipulator_mass500/scr/` |
| `Sensor_liquid_classification.cpp` | Manipulator takes the sensor to classify the liquid at a predefined location           | `src/turtlebot3_manipulator_mass500/scr/` |
| `adc_publisher.py`                 | Publishes analog sensor signals                                                        | `src/analog_sensor/analog_sensor/`        |


