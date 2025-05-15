
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


int main(int argc, char * argv[])
{
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);

  // Create a shared pointer for the node and enable automatic parameter declaration
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a logger for logging messages
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface for the "arm" planning group
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

//--------------------------------------------------------------------------

// MOVING TO TEMPERARY POS HOME 
  
  std::map<std::string, double> temp_home;

  // Joint values
  temp_home["joint1"] = 1.088;  // joint 1 rad
  temp_home["joint2"] = -0.397;  // joint 2 rad
  temp_home["joint3"] = 0.284;  // joint 3 rad
  temp_home["joint4"] = 1.532;  // joint 4 rad

  // Set the target pose for the arm
  move_group_interface.setJointValueTarget(temp_home);

  // Plan the motion for the arm to reach the target pose
  auto const [success_tempHome, plan_tempHome] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning succeeds, execute the planned motion
  if(success_tempHome) {
    move_group_interface.execute(plan_tempHome);
    RCLCPP_INFO(logger, "Arm reach the temp home successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the temp home!");  // Log an error if planning fails
  }








// OPEN GRIPPER
  
   //Create the MoveIt MoveGroup Interface for the "gripper" planning group
  auto gripper_interface = MoveGroupInterface(node, "gripper");

    gripper_interface.setNamedTarget("open");
    if (gripper_interface.move()) {
      RCLCPP_INFO(logger, "Gripper opened successfully");  // Log success
      std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds
    } else {
      RCLCPP_ERROR(logger, "Failed to open the gripper");  // Log an error if it fails
    }








// ABOVE SENSOR	
  
  std::map<std::string, double> temp_sensor;

  // Joint values
  temp_sensor["joint1"] = 0.993;  // joint 1 rad
  temp_sensor["joint2"] = -0.150;  // joint 2 rad
  temp_sensor["joint3"] = 0.668;  // joint 3 rad
  temp_sensor["joint4"] = 0.980;  // joint 4 rad (0.956)

  // Set the target pose for the arm
  move_group_interface.setJointValueTarget(temp_sensor);

  // Plan the motion for the arm to reach the target pose
  auto const [success_tempSensor, plan_tempSensor] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning succeeds, execute the planned motion
  if(success_tempSensor) {
    move_group_interface.execute(plan_tempSensor);
    RCLCPP_INFO(logger, "Arm reach the temerary sensor");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the temerary sensor!");  // Log an error if planning fails
  }
  
  










  
  
  
  
  // DOWN TO SENSOR
  
  std::map<std::string, double> sensor;

  // Joint values
  sensor["joint1"] = 0.960;  // joint 1 rad
  sensor["joint2"] = 0.167;  // joint 2 rad
  sensor["joint3"] = 0.576;  // joint 3 rad
  sensor["joint4"] = 0.832;  // joint 4 rad

  // Set the target pose for the arm
  move_group_interface.setJointValueTarget(sensor);

  // Plan the motion for the arm to reach the target pose
  auto const [success_sensor, plan_sensor] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning succeeds, execute the planned motion
  if(success_sensor) {
    move_group_interface.execute(plan_sensor);
    RCLCPP_INFO(logger, "Arm reached sensor successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the reached sensor !");  // Log an error if planning fails
  }
  
  
 
 
 
 
 
 
 // GRAB SENSOR WITH GRIPPER

  // Set the "close" position for the gripper and move it
  gripper_interface.setNamedTarget("hold_sensor");
  if (gripper_interface.move()) {
    RCLCPP_INFO(logger, "Gripper hold sensor successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds
  } else {
    RCLCPP_ERROR(logger, "Failed to hold sensor");  // Log an error if it fails
  }

 
 
 
 
 
 
 
// LIFT SENSOR
  
  std::map<std::string, double> temp_sensorLift;

  // Joint values
  temp_sensorLift["joint1"] = 1.088;  // joint 1 rad
  temp_sensorLift["joint2"] = -0.397;  // joint 2 rad
  temp_sensorLift["joint3"] = 0.284;  // joint 3 rad
  temp_sensorLift["joint4"] = 1.532;  // joint 4 rad

  // Set the target pose for the arm
  move_group_interface.setJointValueTarget(temp_sensorLift);

  // Plan the motion for the arm to reach the target pose
  auto const [success_tempLift, plan_tempLift] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning succeeds, execute the planned motion
  if(success_tempLift) {
    move_group_interface.execute(plan_tempLift);
    RCLCPP_INFO(logger, "Arm lifted sensor");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed to lift sensor!");  // Log an error if planning fails
  }
 
 
 
 







// TEMPERARY POS LIQUID 

  std::map<std::string, double> temp_liquid;

  // Joint values
  temp_liquid["joint1"] = 1.837;  // joint 1 rad
  temp_liquid["joint2"] = 0.434;  // joint 2 rad
  temp_liquid["joint3"] = -0.006;  // joint 3 rad
  temp_liquid["joint4"] = 1.065;  // joint 4 rad

  // Set the target pose for the arm
  move_group_interface.setJointValueTarget(temp_liquid);

  // Plan the motion for the arm to reach the target pose
  auto const [success_tempLiquid, plan_tempLiquid] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning succeeds, execute the planned motion
  if(success_tempLiquid) {
    move_group_interface.execute(plan_tempLiquid);
    RCLCPP_INFO(logger, "Arm is above liquid successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the above liquid!");  // Log an error if planning fails
  }








// TAKE TEST OF LIQUID, BEGINNING
  
  std::map<std::string, double> liquid1;

  // Joint values
  liquid1["joint1"] = 1.840;   // joint 1 rad  
  liquid1["joint2"] = 0.723;  // joint 2 rad   
  liquid1["joint3"] = -0.146;  // joint 3 rad   
  liquid1["joint4"] = 1.007;  // joint 4 rad   

  // Set the target pose for the arm
  move_group_interface.setJointValueTarget(liquid1);

  // Plan the motion for the arm to reach the target pose
  auto const [success_liquid1, plan_liquid1] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning succeeds, execute the planned motion
  if(success_liquid1) {
    move_group_interface.execute(plan_liquid1);
    RCLCPP_INFO(logger, "Are reach the liquid1 successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the liquid1!");  // Log an error if planning fails
  }







// TAKE TEST OF LIQUID, END
  
  std::map<std::string, double> liquid2;

  // Joint values
  liquid2["joint1"] = 1.624;  // joint 1 rad   1.624
  liquid2["joint2"] = 0.520;  // joint 2 rad   0.420
  liquid2["joint3"] = 0.300;  // joint 3 rad   0.662
  liquid2["joint4"] = 1.007;  // joint 4 rad   0.724

  // Set the target pose for the arm
  move_group_interface.setJointValueTarget(liquid2);

  // Plan the motion for the arm to reach the target pose
  auto const [success_liquid2, plan_liquid2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning succeeds, execute the planned motion
  if(success_liquid2) {
    move_group_interface.execute(plan_liquid2);
    RCLCPP_INFO(logger, "Are reach the liquid2 successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the liquid2!");  // Log an error if planning fails
  }









// ABOVE LIQUID

  // Set the target pose for the arm
  move_group_interface.setJointValueTarget(temp_liquid);
  //move_group_interface.setPlanningTime(10.0); // 5s to plan the path

  // Plan the motion for the arm to reach the target pose
  auto const [success_tempLiquid_2, plan_tempLiquid_2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning succeeds, execute the planned motion
  if(success_tempLiquid_2) {
    move_group_interface.execute(plan_tempLiquid_2);
    RCLCPP_INFO(logger, "Arm reach the temp liquid successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the temp liquid!");  // Log an error if planning fails
  }








  // ABOVE COLLECTION KIT

  // Set the target pose for the arm
  move_group_interface.setJointValueTarget(temp_sensorLift);
  //move_group_interface.setPlanningTime(10.0); // 5s to plan the path

  // Plan the motion for the arm to reach the target pose
  auto const [success_tempLift_2, plan_tempLift_2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning succeeds, execute the planned motion
  if(success_tempLift_2) {
    move_group_interface.execute(plan_tempLift_2);
    RCLCPP_INFO(logger, "Arm reach above collection kit successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the above collection!");  // Log an error if planning fails
  }







  // ABOVE COLLECTION KIT (FOR PRESITION)

  // Set the target pose for the arm
  move_group_interface.setJointValueTarget(temp_sensor);
  //move_group_interface.setPlanningTime(10.0); // 5s to plan the path

  // Plan the motion for the arm to reach the target pose
  auto const [success_tempSensor_3, plan_tempSensor_3] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning succeeds, execute the planned motion
  if(success_tempSensor_3) {
    move_group_interface.execute(plan_tempSensor_3);
    RCLCPP_INFO(logger, "Arm reach above collection kit successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the above collection!");  // Log an error if planning fails
  }






// PUT SENSOR BACK IN PLACE

  move_group_interface.setJointValueTarget(sensor);

  // Plan the motion for the arm to reach the target pose
  auto const [success_sensor_2, plan_sensor_2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning succeeds, execute the planned motion
  if(success_sensor_2) {
    move_group_interface.execute(plan_sensor_2);
    RCLCPP_INFO(logger, "Arm putting sensor back successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(2));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the sensor back!");  // Log an error if planning fails
  }
  
  
  
  
  
  
  
// OPEN GRIPPER

    gripper_interface.setNamedTarget("open");
    if (gripper_interface.move()) {
      RCLCPP_INFO(logger, "Gripper opened successfully");  // Log success
      std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds
    } else {
      RCLCPP_ERROR(logger, "Failed to open the gripper");  // Log an error if it fails
    }
  
  
  
 





// TEMPERARY HOME POS

  // Set the target pose for the arm
  move_group_interface.setJointValueTarget(temp_home);

  // Plan the motion for the arm to reach the target pose
  auto const [success_tempHome_2, plan_tempHome_2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning succeeds, execute the planned motion
  if(success_tempHome_2) {
    move_group_interface.execute(plan_tempHome_2);
    RCLCPP_INFO(logger, "Arm reach the temp home successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the temp home!");  // Log an error if planning fails
  }




  


// CLOSING GRIPPER

  // Set the "close" position for the gripper and move it
  gripper_interface.setNamedTarget("close");
  if (gripper_interface.move()) {
    RCLCPP_INFO(logger, "Gripper closed successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds
  } else {
    RCLCPP_ERROR(logger, "Failed to close the gripper");  // Log an error if it fails
  }

  




// HOME POSITION 
  
  std::map<std::string, double> home;

  // Joint values
  home["joint1"] = 0.002;  // joint 1 rad
  home["joint2"] = -1.046;  // joint 2 rad
  home["joint3"] = 1.077;  // joint 3 rad
  home["joint4"] = 0.011;  // joint 4 rad

  // Set the target pose for the arm
  move_group_interface.setJointValueTarget(home);

  // Plan the motion for the arm to reach the target pose
  auto const [success_home, plan_home] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning succeeds, execute the planned motion
  if(success_home) {
    move_group_interface.execute(plan_home);
    RCLCPP_INFO(logger, "Arm reach the home pos successfully");  // Log success
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait for 1 seconds after execution
  } else {
    RCLCPP_ERROR(logger, "Planning failed for the home!");  // Log an error if planning fails
  }







  // Shut down the ROS2 node
  rclcpp::shutdown();
  return 0;
}
