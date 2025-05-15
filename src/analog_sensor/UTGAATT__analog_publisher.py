import time
import rclpy  # ROS 2 Python client library
from rclpy.node import Node
from std_msgs.msg import Float32  # Message type
import Adafruit_ADS1x15  # ADC Library

class AnalogSensorPublisher(Node):
    def __init__(self):
        # Ensure Node is properly initialized with a name
        super().__init__("analog_sensor_publisher")  

        # Create a ROS 2 publisher
        self.publisher_ = self.create_publisher(Float32, "analog_sensor", 10)
        
        
        # Timer to publish data every second
        self.timer = self.create_timer(1.0, self.publish_sensor_data)


        # Initialize ADC (ADS1015)
        self.adc = Adafruit_ADS1x15.ADS1015()
        self.channel = 0  # Sensor connected to A0
        self.gain = 1  # ±4.096V FSR

    
    def convert_to_arduino_scale(self, adc_value):
        """Convert ADS1015 (12-bit, 3.3V) readings to Arduino scale (10-bit, 5V)."""
        return (adc_value / 4) * (5 / 3.3)

    def analog_sensor_callback(self):
        # Read ADC value
        raw_value = self.adc.read_adc(self.channel, gain=self.gain)

        # Convert to Arduino-equivalent value
        corrected_value = self.convert_to_arduino_scale(raw_value)

        # Publish message
        msg = Float32()
        msg.data = corrected_value
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {corrected_value:.2f}")

def main(args=None):
    rclpy.init(args=args)
    
    # Ensure Node is properly initialized
    analog_sensor_publisher = AnalogSensorPublisher()

    try:
        rclpy.spin(analog_sensor_publisher)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


