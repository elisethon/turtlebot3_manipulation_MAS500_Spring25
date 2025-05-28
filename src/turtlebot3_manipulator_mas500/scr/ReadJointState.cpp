// THIS SCRIPT DOES: 
// - Reads sensor values and evaluates them
// - If sensor values is in a certain range, the joint states are captured and parsed into a .txt file 

// The ranges for the values indicating the liquids were found through testing with the sensor and liquids 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <fstream>
#include <chrono>
#include <memory>

class JointStateReader : public rclcpp::Node {
public:
    JointStateReader()
    : Node("joint_state_reader"), save_position_(false), saved_joint_positions_(), sensor_value_(0.0f), has_saved_(false) {
    
    	clearJointFile(); // Clear old data at startup
    	
    	// --- Subscribe to joint state tipic --- //
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&JointStateReader::jointStateCallback, this, std::placeholders::_1));

	// --- Subscribe to sensor tipic --- //
        sensor_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/read_analog_sensor", 10, std::bind(&JointStateReader::sensorCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "JointStateReader is active...");
    }

private:

    // --- Sensor callback --- //
    void sensorCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        sensor_value_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Sensor value: %.2f", sensor_value_);

	// --- Based on the sensor value, determine the substance --- //
	if (sensor_value_ < 1.35f) {
            RCLCPP_INFO(this->get_logger(), "Nothing detected");  				// To low to evaluate (nothing detected)
        } else if (sensor_value_ < 1.65f) {
            RCLCPP_INFO(this->get_logger(), "Coolant is detected"); 				// Liquid is coolant
        } else if (sensor_value_ < 1.75f) {
            RCLCPP_INFO(this->get_logger(), "Unsure of liquid. Saving joint positions..."); 	// Unclear what the liquid is
            save_position_ = true; 								// Save joint position where the liquid was detected
        } else {
            RCLCPP_INFO(this->get_logger(), "Water is detected"); 				// Liquid is water
        }
        
        
    }

    // --- Joint sate callback --- //
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (save_position_ && !has_saved_) {
            saved_joint_positions_ = msg->position; // copy current joint state
            save_position_ = false;
            has_saved_ = true; // prevent duble save

	    // saves the joint state to a txt file
            std::ofstream file("captured_joint_positions/joint_position.txt");
            for (double pos : saved_joint_positions_) {
                file << pos << " ";
            }
            file << std::endl;
            file.close();

            RCLCPP_INFO(this->get_logger(), "Joint positions saved!");

	    // --- Print joints if data has at least 8 joints --- // 
		// note: not all data extracted is usefull, only some are fetched 
            if (saved_joint_positions_.size() >= 8) {
                double joint1 = saved_joint_positions_[3]; // joint 1
                double joint2 = saved_joint_positions_[1]; // joint 2
                double joint3 = saved_joint_positions_[7]; // joint 3
                double joint4 = saved_joint_positions_[4]; // joint 4

		// --- Print the joint state in terminal --- //
                RCLCPP_INFO(this->get_logger(),
                            "Saved Joint Values: Joint1: %f, Joint2: %f, Joint3: %f, Joint4: %f",
                            joint1, joint2, joint3, joint4);
            }
        }
    }
    
    
// --- Clear the .txt file in the begining to make sure no "leftovers" --- //
void clearJointFile() {
    std::ofstream file("captured_joint_positions/joint_position.txt", std::ios::trunc);  
    if (file.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Cleared joint_position.txt at startup.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to clear joint_position.txt");
    }
}


    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sensor_sub_;

    bool save_position_;
    std::vector<double> saved_joint_positions_;
    float sensor_value_;
    bool has_saved_; 

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); // initilize ROS 
    rclcpp::spin(std::make_shared<JointStateReader>()); // start node
    rclcpp::shutdown();
    
    
    return 0;
}

