// MADE AND CHANGED FOR MAS500 SPRING 2025 

// THIS SCRIPT DOES (coordinates): 
// - Pick up q-tip
// - Take liquid sampe, using q-tip, at location of liquid --> using joint states from .txt file 
// - Returns the q-tip 
// - Moves to home position

// THIS WAS MADE USING THE EXAMPLE SCRIPT FROM THE TURTLEBOT WS,
// HOWEVER; MOST OF THIS IS MADE USING GENERAL MOVEIT SYNTAX

// THE ORIGINAL COPYRIGHT:
/*******************************************************************************
 * Copyright 2024 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/
/* Authors: Wonho Yoon, Sungho Woo */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>


// --- Transform old positions to new with quaterniond --- //

class StaticPoseTransformer {
public:
    StaticPoseTransformer() {
        
        double angle_rad = -M_PI / 2.0; // Rotation: 90 degrees around Z-axis clock-wise
        Eigen::AngleAxisd rotation_z(angle_rad, Eigen::Vector3d::UnitZ()); 
        rotation_quat_ = Eigen::Quaterniond(rotation_z);
        translation_vec_ = Eigen::Vector3d(0.0, 0.035, 0.154); // Translasjon from O1 to O2
    }
    
    // --- Old position in x,y,z --- //
    geometry_msgs::msg::Pose transformToGlobal(const geometry_msgs::msg::Pose& local_pose) {
        Eigen::Vector3d position_local(
            local_pose.position.x,
            local_pose.position.y,
            local_pose.position.z
        );
	// --- Old Quaterniond in x, y, z, w --- //
        Eigen::Quaterniond orientation_local(
            local_pose.orientation.w,
            local_pose.orientation.x,
            local_pose.orientation.y,
            local_pose.orientation.z
        );

        Eigen::Vector3d position_global = rotation_quat_ * position_local + translation_vec_; // Vector from O1 to P in global frame
        Eigen::Quaterniond orientation_global = rotation_quat_ * orientation_local; // Rotation of orientation to global frame (from local O2)

	// -- New pose msgs --- //
        geometry_msgs::msg::Pose global_pose;
        global_pose.position.x = position_global.x();
        global_pose.position.y = position_global.y();
        global_pose.position.z = position_global.z();

        global_pose.orientation.w = orientation_global.w();
        global_pose.orientation.x = orientation_global.x();
        global_pose.orientation.y = orientation_global.y();
        global_pose.orientation.z = orientation_global.z();

        return global_pose;
    }

private:
    Eigen::Quaterniond rotation_quat_;
    Eigen::Vector3d translation_vec_;
};




// --- Read from the txt file with the joint states for the manipulator --- //

std::vector<double> readJointPositions(const std::string& filename) {
    std::ifstream infile(filename);
    std::vector<double> positions;
    std::string line;

    if (infile && std::getline(infile, line)) {
        std::istringstream iss(line);
        double val;
        while (iss >> val) {
            positions.push_back(val);
        }
    } else {
        std::cerr << "Failed to open or read file: " << filename << std::endl;
    }

    return positions;
}



int main(int argc, char * argv[])
{
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);

  // --- Create a shared pointer for the node and enable automatic parameter declaration --- //
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // --- Create a logger for logging messages --- //
  auto const logger = rclcpp::get_logger("hello_moveit");

  // --- Create the MoveIt MoveGroup Interface for the "arm" planning group --- //
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");



// --- Move to temporary home position --- // 

  auto const temp_home = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = -0.009;  // Orientation (quaternion x)
    msg.orientation.y = 0.466;  // Orientation (quaternion y)
    msg.orientation.z = 0.017;  // Orientation (quaternion z)
    msg.orientation.w = 0.885;  // Orientation (quaternion w)
    msg.position.x = 0.147;   // Position in x
    msg.position.y = 0.005;  // Position in y
    msg.position.z = 0.100;   // Position in z
    return msg;
  }();
    
  
  // --- Transform and set as target --- //
  StaticPoseTransformer transformer;
  geometry_msgs::msg::Pose temp_homeG = transformer.transformToGlobal(temp_home);  
  
  move_group_interface.setPoseTarget(temp_homeG);
  move_group_interface.setPlanningTime(10.0);  // Planning time set to 10s

  // --- Set tolerances for goal position and orientation --- //
  move_group_interface.setGoalPositionTolerance(0.02);
  move_group_interface.setGoalOrientationTolerance(0.02);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_tempHome, plan_tempHome] = [&move_group_interface, &logger]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute the planned motion --- //
  if(success_tempHome) {
    move_group_interface.execute(plan_tempHome);
    RCLCPP_INFO(logger, "Arm reach the temp home successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(10));  // Wait for 10 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the temp home!");  // Log an error if planning fails
  }



// --- Open gripper -- //
  
   // --- Create the MoveIt MoveGroup Interface for the "gripper" planning group --- //
  auto gripper_interface = MoveGroupInterface(node, "gripper");

    gripper_interface.setNamedTarget("open");
    if (gripper_interface.move()) {
      RCLCPP_INFO(logger, "Gripper opened successfully");  // Log success
      std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds
    } else {
      RCLCPP_ERROR(logger, "Failed to open the gripper");  // Log an error if it fails
    }



// --- Move above the q-tip --- //

  auto const temp_qtip = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = -0.243;  // Orientation (quaternion x)
    msg.orientation.y = 0.670;  // Orientation (quaternion y)
    msg.orientation.z = 0.239;  // Orientation (quaternion z)
    msg.orientation.w = 0.660;  // Orientation (quaternion w)
    msg.position.x = 0.136;   // Position in x
    msg.position.y = 0.103;  // Position in y
    msg.position.z = 0.094;   // Position in z
    return msg;
  }();

  // --- Set the target pose for the arm --- //
  geometry_msgs::msg::Pose temp_qtipG = transformer.transformToGlobal(temp_qtip);
  
  move_group_interface.setPoseTarget(temp_qtipG);
  move_group_interface.setPlanningTime(10.0);  // Planning time set to 10s

  // --- Set tolerances for goal position and orientation --- //
  move_group_interface.setGoalPositionTolerance(0.05);
  move_group_interface.setGoalOrientationTolerance(0.05);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_tempQtip, plan_tempQtip] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute the planned motion --- //
  if(success_tempQtip) {
    move_group_interface.execute(plan_tempQtip);
    RCLCPP_INFO(logger, "Arm reach the temerary qtip");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(10));  // Wait for 10 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the temerary qtip!");  // Log an error if planning fails
  }
  
  
  
 // --- Moving down to the q-tip --- //

  auto const qtip = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = -0.240;  // Orientation (quaternion x)
    msg.orientation.y = 0.657;  // Orientation (quaternion y)
    msg.orientation.z = 0.245;  // Orientation (quaternion z)
    msg.orientation.w = 0.671;  // Orientation (quaternion w)
    msg.position.x = 0.136;   // Position in x
    msg.position.y = 0.105;  // Position in y
    msg.position.z = 0.013;   // Position in z
    return msg;
  }();

  // --- Set the target pose for the arm --- //
  geometry_msgs::msg::Pose qtipG = transformer.transformToGlobal(qtip);
  
  move_group_interface.setPoseTarget(qtipG);
  move_group_interface.setPlanningTime(10.0);  // Planning time set to 10s

  // --- Set tolerances for goal position and orientation --- //
  move_group_interface.setGoalPositionTolerance(0.02);
  move_group_interface.setGoalOrientationTolerance(0.02);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_qtip, plan_qtip] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute the planned motion --- //
  if(success_qtip) {
    move_group_interface.execute(plan_qtip);
    RCLCPP_INFO(logger, "Arm reached qtip successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(10));  // Wait for 10 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the reached qtip !");  // Log an error if planning fails
  }
  
  
 
 // --- Grab q-tip with gripper --- // 

  // --- Set the "close" position for the gripper and move it --- //
  gripper_interface.setNamedTarget("hold_qtip");
  if (gripper_interface.move()) {
    RCLCPP_INFO(logger, "Gripper hold qtip successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds
  } else {
    RCLCPP_ERROR(logger, "Failed to hold qtip");  // Log an error if it fails
  }

 
 
// --- Lift q-tip out of test tube --- // 

  // --- Set the target pose for the arm --- //
  move_group_interface.setPoseTarget(temp_qtipG);
  move_group_interface.setPlanningTime(10.0);  // Planning time set to 10s
  
  // --- Set tolerances for goal position and orientation --- //
  move_group_interface.setGoalPositionTolerance(0.05);
  move_group_interface.setGoalOrientationTolerance(0.05);

  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_tempQtip_2, plan_tempQtip_2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute the planned motion --- //
  if(success_tempQtip_2) {
    move_group_interface.execute(plan_tempQtip_2);
    RCLCPP_INFO(logger, "Arm lifted qtip from collection kit");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(10));  // Wait for 10 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the lift qtip!");  // Log an error if planning fails
  }

	

// --- Move above the liquid --- // 

  auto const temp_liquid = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = -0.467;  // Orientation (quaternion x)
    msg.orientation.y = 0.501;  // Orientation (quaternion y)
    msg.orientation.z = 0.497;  // Orientation (quaternion z)
    msg.orientation.w = 0.533;  // Orientation (quaternion w)
    msg.position.x = 0.024;   // Position in x
    msg.position.y = 0.174;  // Position in y
    msg.position.z = 0.052;   // Position in z
    return msg;
  }();

  // --- Set the target pose for the arm --- //
  geometry_msgs::msg::Pose temp_liquidG = transformer.transformToGlobal(temp_liquid);
  
  move_group_interface.setPoseTarget(temp_liquidG);
  move_group_interface.setPlanningTime(10.0);  // Planning time set to 10s

  // --- Set tolerances for goal position and orientation --- //
  move_group_interface.setGoalPositionTolerance(0.02);
  move_group_interface.setGoalOrientationTolerance(0.02);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_tempLiquid, plan_tempLiquid] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute the planned motion --- //
  if(success_tempLiquid) {
    move_group_interface.execute(plan_tempLiquid);
    RCLCPP_INFO(logger, "Arm is above liquid successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(10));  // Wait for 10 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the above liquid!");  // Log an error if planning fails
  }



// --- Take sample of liquid --- // 

// --- Read the joint state from the saved txt file --- //
  std::vector<double> saved_joint_position = readJointPositions("captured_joint_positions/joint_position.txt");
  
  std::map<std::string, double> liquid;
  // --- Joint values --- //
  liquid["joint1"] = saved_joint_position[3];  // joint 1 rad
  liquid["joint2"] = saved_joint_position[1];  // joint 2 rad
  liquid["joint3"] = saved_joint_position[7];  // joint 3 rad
  liquid["joint4"] = saved_joint_position[4];  // joint 4 rad

  // --- Set the target pose for the arm --- //
  move_group_interface.setJointValueTarget(liquid);
  

  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_liquid, plan_liquid] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute the planned motion --- //
  if(success_liquid) {
    move_group_interface.execute(plan_liquid);
    RCLCPP_INFO(logger, "Are reach the liquid successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(3));  // Wait for 3 seconds to make sure som liquid was absorbed
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the liquid!");  // Log an error if planning fails
  }



// -- Move above the liquid again --- // 

  // --- Set the target pose for the arm --- //
  move_group_interface.setPoseTarget(temp_liquidG);
  move_group_interface.setPlanningTime(10.0);  // Planning time set to 10s
  
  // --- Set tolerances for goal position and orientation --- //
  move_group_interface.setGoalPositionTolerance(0.02);
  move_group_interface.setGoalOrientationTolerance(0.02);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_tempLiquid_2, plan_tempLiquid_2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute the planned motion --- //
  if(success_tempLiquid_2) {
    move_group_interface.execute(plan_tempLiquid_2);
    RCLCPP_INFO(logger, "Arm reach the temp liquid successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(10));  // Wait for 10 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the temp liquid!");  // Log an error if planning fails
  }



  // --- Move above the test tube --- //

  // --- Set the target pose for the arm --- //
  move_group_interface.setPoseTarget(temp_qtipG);
  move_group_interface.setPlanningTime(10.0);  // Planning time set to 10s
  
  // --- Set tolerances for goal position and orientation --- //
  move_group_interface.setGoalPositionTolerance(0.05);
  move_group_interface.setGoalOrientationTolerance(0.05);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_tempQtip_3, plan_tempQtip_3] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute the planned motion --- //
  if(success_tempQtip_3) {
    move_group_interface.execute(plan_tempQtip_3);
    RCLCPP_INFO(logger, "Arm reach above collection kit successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(10));  // Wait for 10 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the above collection!");  // Log an error if planning fails
  }



// --- Put q-tip back into test tube --- //

  auto const qtip_back = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = -0.243;  // Orientation (quaternion x)
    msg.orientation.y = 0.674;  // Orientation (quaternion y)
    msg.orientation.z = 0.237;  // Orientation (quaternion z)
    msg.orientation.w = 0.656;  // Orientation (quaternion w)
    msg.position.x = 0.135;   // Position in x
    msg.position.y = 0.102;  // Position in y
    msg.position.z = 0.061;   // Position in z
    return msg;
  }();

  // --- Set the target pose for the arm --- //
  geometry_msgs::msg::Pose qtip_backG = transformer.transformToGlobal(qtip_back);
  
  move_group_interface.setPoseTarget(qtip_backG);
  move_group_interface.setPlanningTime(10.0);  // Planning time set to 10s
	
  // --- Set tolerances for goal position and orientation --- //
  move_group_interface.setGoalPositionTolerance(0.02);
  move_group_interface.setGoalOrientationTolerance(0.02);

  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_qtipBack, plan_qtipBack] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute the planned motion --- //
  if(success_qtipBack) {
    move_group_interface.execute(plan_qtipBack);
    RCLCPP_INFO(logger, "Arm putting qtip into tube successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(10));  // Wait for 10 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the qtip into tube!");  // Log an error if planning fails
  }
  
  
  
// --- Open gripper to let go of q-tip --- // 

    gripper_interface.setNamedTarget("open");
    if (gripper_interface.move()) {
      RCLCPP_INFO(logger, "Gripper opened successfully");  // Log success
      std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds
    } else {
      RCLCPP_ERROR(logger, "Failed to open the gripper");  // Log an error if it fails
    }
  


// --- Move to temporary home position --- //

  // --- Set the target pose for the arm --- //
  move_group_interface.setPoseTarget(temp_homeG);
  move_group_interface.setPlanningTime(10.0);  // Planning time set to 10s
  
  // --- Set tolerances for goal position and orientation --- //
  move_group_interface.setGoalPositionTolerance(0.02);
  move_group_interface.setGoalOrientationTolerance(0.02);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_tempHome_2, plan_tempHome_2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute the planned motion --- //
  if(success_tempHome_2) {
    move_group_interface.execute(plan_tempHome_2);
    RCLCPP_INFO(logger, "Arm reach the temp home successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(10));  // Wait for 10 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the temp home!");  // Log an error if planning fails
  }


	
// --- Closing gripper to prepare to move to home position --- //

  // --- Set the "close" position for the gripper and move it --- //
  gripper_interface.setNamedTarget("close");
  if (gripper_interface.move()) {
    RCLCPP_INFO(logger, "Gripper closed successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds
  } else {
    RCLCPP_ERROR(logger, "Failed to close the gripper");  // Log an error if it fails
  }

  

// --- Move to home position --- // 

  auto const home = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.015;  // Orientation (quaternion x)
    msg.orientation.y = 0.015;  // Orientation (quaternion y)
    msg.orientation.z = -0.706;  // Orientation (quaternion z)
    msg.orientation.w = 0.708;  // Orientation (quaternion w)
    msg.position.x = 0.000;   // Position in x
    msg.position.y = -0.128;  // Position in y
    msg.position.z = 0.306;   // Position in z
    return msg;
  }();

  // --- Set the target pose for the arm --- //
  move_group_interface.setPoseTarget(home);
  move_group_interface.setPlanningTime(10.0);  // Planning time set to 10s

  // --- Set tolerances for goal position and orientation --- //
  move_group_interface.setGoalPositionTolerance(0.02);
  move_group_interface.setGoalOrientationTolerance(0.02);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_home, plan_home] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute the planned motion --- //
  if(success_home) {
    move_group_interface.execute(plan_home);
    RCLCPP_INFO(logger, "Arm reach the home pos successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(10));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the home!");  // Log an error if planning fails
  }


	
// --- Shut down the ROS2 node --- //
  rclcpp::shutdown();
  return 0;
}
