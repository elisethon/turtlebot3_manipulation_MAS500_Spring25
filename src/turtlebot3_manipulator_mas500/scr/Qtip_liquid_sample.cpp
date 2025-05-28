// MADE AND CHANGED FOR MAS500 SPRING 2025 

// THIS SCRIPT DOES: 
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
  
  std::map<std::string, double> temp_home;
  //  --- Joint values --- //
  temp_home["joint1"] = 0.038;  // joint 1 rad
  temp_home["joint2"] = -0.660;  // joint 2 rad
  temp_home["joint3"] = 0.566;  // joint 3 rad
  temp_home["joint4"] = 1.062;  // joint 4 rad

    
  // --- Set the target pose for the arm --- //
  move_group_interface.setJointValueTarget(temp_home);


  // --- Plan the motion for the arm to reach the target pose -- //
  auto const [success_tempHome, plan_tempHome] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

    
  // -- If planning succeeds, execute it --- //
  if(success_tempHome) {
    move_group_interface.execute(plan_tempHome);
    RCLCPP_INFO(logger, "Arm reach the temp home successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
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
  
  std::map<std::string, double> temp_qtip;
  //  --- Joint values --- //
  temp_qtip["joint1"] = 0.668;  // joint 1 rad
  temp_qtip["joint2"] = 0.129;  // joint 2 rad
  temp_qtip["joint3"] = -0.285;  // joint 3 rad
  temp_qtip["joint4"] = 1.742;  // joint 4 rad

  // --- Set the target pose for the arm --- //
  move_group_interface.setJointValueTarget(temp_qtip);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_tempQtip, plan_tempQtip] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute it --- //
  if(success_tempQtip) {
    move_group_interface.execute(plan_tempQtip);
    RCLCPP_INFO(logger, "Arm reach the temerary qtip");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the temerary qtip!");  // Log an error if planning fails
  }
  
  
  
  // --- Moving down to the q-tip --- //  
  
  std::map<std::string, double> qtip;
  // ---Joint values --- //
  qtip["joint1"] = 0.700;  // joint 1 rad
  qtip["joint2"] = 0.210;  // joint 2 rad
  qtip["joint3"] = 0.272;  // joint 3 rad
  qtip["joint4"] = 1.068;  // joint 4 rad

  // --- Set the target pose for the arm --- //
  move_group_interface.setJointValueTarget(qtip);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_qtip, plan_qtip] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute it --- //
  if(success_qtip) {
    move_group_interface.execute(plan_qtip);
    RCLCPP_INFO(logger, "Arm reached qtip successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
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
  move_group_interface.setJointValueTarget(temp_qtip);

  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_tempQtip_2, plan_tempQtip_2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute it --- //
  if(success_tempQtip_2) {
    move_group_interface.execute(plan_tempQtip_2);
    RCLCPP_INFO(logger, "Arm lifted qtip from collection kit");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the lift qtip!");  // Log an error if planning fails
  }

    
 
// --- Move above the liquid --- // 
    
  std::map<std::string, double> temp_liquid;
  // --- Joint values --- //
  temp_liquid["joint1"] = 1.255;  // joint 1 rad
  temp_liquid["joint2"] = -0.480;  // joint 2 rad
  temp_liquid["joint3"] = 0.316;  // joint 3 rad
  temp_liquid["joint4"] = 1.190;  // joint 4 rad

  // --- Set the target pose for the arm --- //
  move_group_interface.setJointValueTarget(temp_liquid);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_tempLiquid, plan_tempLiquid] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute it --- //
  if(success_tempLiquid) {
    move_group_interface.execute(plan_tempLiquid);
    RCLCPP_INFO(logger, "Arm is above liquid successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
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

  // --- If planning succeeds, execute it --- //
  if(success_liquid) {
    move_group_interface.execute(plan_liquid);
    RCLCPP_INFO(logger, "Arm reach the liquid successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(3));  // Wait for 3 seconds to make sure som liquid was absorbed
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the liquid!");  // Log an error if planning fails
  }



// -- Move above the liquid again --- // 

  // --- Set the target pose for the arm --- //
  move_group_interface.setJointValueTarget(temp_liquid);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_tempLiquid_2, plan_tempLiquid_2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute the planned motion --- //
  if(success_tempLiquid_2) {
    move_group_interface.execute(plan_tempLiquid_2);
    RCLCPP_INFO(logger, "Arm reach the temp liquid successfully EEEEEEEEEEEEEEEEEEE");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the temp liquid!");  // Log an error if planning fails
  }



  // --- Move above the test tube --- //

  // --- Set the target pose for the arm --- //
  move_group_interface.setJointValueTarget(temp_qtip);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_tempQtip_3, plan_tempQtip_3] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute it --- //
  if(success_tempQtip_3) {
    move_group_interface.execute(plan_tempQtip_3);
    RCLCPP_INFO(logger, "Arm reach above collection kit successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the above collection!");  // Log an error if planning fails
  }



// --- Put q-tip back into test tube --- //

  std::map<std::string, double> qtip_back;
  // --- Joint values --- //
  qtip_back["joint1"] = 0.668;  // joint 1 rad
  qtip_back["joint2"] = 0.130;  // joint 2 rad
  qtip_back["joint3"] = -0.021;  // joint 3 rad
  qtip_back["joint4"] = 1.450;  // joint 4 rad

  // --- Set the target pose for the arm --- //
  move_group_interface.setJointValueTarget(qtip_back);

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
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
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
  move_group_interface.setJointValueTarget(temp_home);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_tempHome_2, plan_tempHome_2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute it --- //
  if(success_tempHome_2) {
    move_group_interface.execute(plan_tempHome_2);
    RCLCPP_INFO(logger, "Arm reach the temp home successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
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
  
  std::map<std::string, double> home;
  // --- Joint values --- //
  home["joint1"] = 0.002;  // joint 1 rad
  home["joint2"] = -1.046;  // joint 2 rad
  home["joint3"] = 1.077;  // joint 3 rad
  home["joint4"] = 0.011;  // joint 4 rad

  // --- Set the target pose for the arm --- //
  move_group_interface.setJointValueTarget(home);


  // --- Plan the motion for the arm to reach the target pose --- //
  auto const [success_home, plan_home] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // --- If planning succeeds, execute it --- //
  if(success_home) {
    move_group_interface.execute(plan_home);
    RCLCPP_INFO(logger, "Arm reach the home pos successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the home!");  // Log an error if planning fails
  }



  // --- Shut down the ROS2 node --- //
    
  rclcpp::shutdown();
  return 0;
}
