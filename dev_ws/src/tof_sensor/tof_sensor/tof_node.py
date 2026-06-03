import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import board
import adafruit_vl53l1x

class VL53L1XNode(Node):
    def __init__(self):
        super().__init__('tof_sensor_node')
        
        # We publish to the standard ROS 2 '/range' topic
        self.publisher_ = self.create_publisher(Range, '/range', 10)
        
        # 1. Initialize the I2C bus and the sensor
        self.get_logger().info("Connecting to VL53L1X...")
        i2c = board.I2C()
        self.vl53 = adafruit_vl53l1x.VL53L1X(i2c)
        
        # 2. Configure the sensor (Mode 1 = Short, Mode 2 = Long Range)
        self.vl53.distance_mode = 1 
        self.vl53.timing_budget = 50 # ms per reading
        self.vl53.start_ranging()
        
        self.get_logger().info("Sensor connected! Publishing data at 10Hz...")
        
        # 3. Create a timer to read the sensor 10 times a second
        self.timer = self.create_timer(0.1, self.publish_distance)

    def publish_distance(self):
        # Check if the hardware actually has a new measurement ready
        if self.vl53.data_ready:
            distance_cm = self.vl53.distance
            self.vl53.clear_interrupt()
            
            # Sometimes the sensor returns None if it doesn't see anything
            if distance_cm is not None:
                # Build the standard ROS Range message
                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "tof_link" # For RViz
                
                msg.radiation_type = Range.INFRARED
                msg.field_of_view = 0.47 # Roughly 27 degrees
                msg.min_range = 0.04     # 4 cm minimum
                msg.max_range = 4.0      # 4 meters maximum
                
                # Convert centimeters to meters for standard ROS units
                msg.range = distance_cm / 100.0 
                
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VL53L1XNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.vl53.stop_ranging() # Cleanly turn off the laser
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()