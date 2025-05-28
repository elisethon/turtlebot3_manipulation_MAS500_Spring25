# This script is made to be able to read the analog signal from the sensor using:
# 12 bit ADC from Adafruit 
# Adafruit Industries LLC 4965 analog sensor 

# ROS2 is used to publish the readings 

import time
import rclpy  # ROS 2 Python client library
from rclpy.node import Node
from std_msgs.msg import Float32  # Message type
import Adafruit_ADS1x15  # ADC Library

class AnalogSensorPublisher(Node):
    def __init__(self):
        # Initialized node
        super().__init__("analog_sensor_publisher")  

        # Create publisher
        self.publisher_ = self.create_publisher(Float32, "read_analog_sensor", 10) # Float32

        # Initialize ADC (ADS1015)
        self.adc = Adafruit_ADS1x15.ADS1015()
        self.channel = 0  # Sensor connected to A0
        self.gain = 2  # for ±2.048V FSR, set to 1 if ±4.096V FSR
        
        self.first_publish = True  
        self.timer = self.create_timer(1.0, self.publish_sensor_data)

    # CONVERT VALUE TO VOLT
    def convert_to_arduino_scale(self, adc_value):
        """Will convert analog signal to voltage"""
        return (adc_value)*(2.048/2047)

    # READ VALUE
    def publish_sensor_data(self):
        raw_value = self.adc.read_adc(self.channel, gain=self.gain)

        # Convert value
        corrected_value = self.convert_to_arduino_scale(raw_value)

        # Publish
        msg = Float32()
        msg.data = corrected_value
        self.publisher_.publish(msg)

        # Reccomend using 'ros2 topic echo /read_analog_sensor' to see values, but if desired to log, uncomment next line: 
        # self.get_logger().info(f"Published: {corrected_value:.2f}")
        
        if self.first_publish:
            self.get_logger().info("Publishing analog sensor signal")
            self.first_publish = False  

def main(args=None):
    rclpy.init(args=args)
    node = AnalogSensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

