# General Information

Several tables in this *README* file have been copied from our Master's thesis titled
**"Framework for Modular Robotic System for Remote Liquid Classification and Sample Retrieval"**
to provide insight and clarity into the functions of this repository.

This repository is part of the documentation for MAS500 Spring 2025, developed by Group 3.
It is forked from a repository that combines the TurtleBot3 Waffle Pi robot platform with the OpenManipulator-X manipulator.
This was done to share the changes made during development for MAS500, specifically for autonomous deployment and liquid classification tasks.

For this project, **Ubuntu 22.04** and **ROS2 Humble** were used.

Additional documentation can be found in the [Sphinx documentation](https://master-documentation-s25-gr3-405ee9.gitlab.io/index.html).

---

# Changes and Additions to Hardware

![Image of Robot Platform Assembly](TBOgBill_Front.jpg)

For the thesis, several components were changed or added. An overview of the main changes is shown below:

| **Part**                       | **Attached with**                  | **Attached to**          |  **Weight \[g]**   |
| ------------------------------ | ---------------------------------- | ------------------------ | -------------------| 
| Manipulator stand (level four) | Six screws, three support rods     | Top level (level three)  | 49                 |
| Bracket                        | Two screws, two bolts              | Top level (level three)  | 23                 |
| Sample rack                    |                                    | Bracket                  | 59                 |
| Sensor holder                  |                                    | Collection kit           | 14                 |
| Sensor                         | Two screws, two bolts, three wires | Sensor holder            | 4                  |
| Test tube                      |                                    | Collection kit           | 7                  |
| Q-tip with attachments         | Q-tip lid                          | Test tube                | 2                  |
| Analog to digital converter    | Two screws, five wires             | Bottom level (level two) | 4                  |

---

# Changes and Additions to Repository

From the original repository, several changes have been made—primarily those shown in the table below:

| **Name of script**                   | **Description of changes made**                                           | **Location in GitHub repository**                   |
| ------------------------------------ | ------------------------------------------------------------------------- | --------------------------------------------------- |
| `turtlebot3_manipulation.urdf.xacro` | Changed the position and orientation of the manipulator on the TurtleBot3 | `src/turtlebot3_manipulation_description/urdf/`     |
| `open_manipulator_x.urdf.xacro`      | Added more predefined positions for the manipulator                       | `src/turtlebot3_manipulation_description/urdf/`     |
| `turtlebot3_manipulation.srdf`       | Changed the limits of the gripper opening                                 | `src/turtlebot3_manipulation_moveit_config/config/` |

In addition to this, several new files were created and added. These include the package *read\_analog\_sensor*, which contains the execution script *adc\_publisher.py*.
An overview of the main scripts is given below:

| **Name of script**                 | **Description of function**                                                            | **Location in this GitHub repository**    |
| ---------------------------------- | -------------------------------------------------------------------------------------- | ----------------------------------------- |
| `Qtip_liquid_sample.cpp`           | Manipulator grabs the Q-tip from the test tube and takes a sample of liquid            | `src/turtlebot3_manipulator_mass500/scr/` |
| `ReadJointState.cpp`               | Saves the joint state if the detected liquid is outside classification ranges          | `src/turtlebot3_manipulator_mass500/scr/` |
| `Sensor_detect_liquid.cpp`         | Manipulator uses the sensor and multiple tests to locate the liquid relative to itself | `src/turtlebot3_manipulator_mass500/scr/` |
| `Sensor_liquid_classification.cpp` | Manipulator uses the sensor to classify the liquid at a predefined location            | `src/turtlebot3_manipulator_mass500/scr/` |
| `adc_publisher.py`                 | Publishes analog sensor signals                                                        | `src/analog_sensor/analog_sensor/`        |

The added package *read\_analog\_sensor* reads the analog sensor signal and publishes it.
It is configured for the **Adafruit Industries LLC 4965** analog sensor and the **12-bit ADC** from Adafruit.

---

# How to Use

*If you want to use this repository for the TurtleBot3 Waffle Pi in combination with the OpenManipulator-X*:

Follow the [official TurtleBot3 quick start guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/),
but **clone this repository** instead of the one described in the section on
[manipulator and robot platform integration](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation).

Make sure to also clone the other required repositories and **download all necessary packages** as described in the guide.

