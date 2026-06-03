import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import time

class LoadCellNode(Node):
    def __init__(self):
        super().__init__('load_cell_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        # Insert the calibration factor you found earlier
        self.declare_parameter('calibration_factor', 4507) 

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.calibration_factor = self.get_parameter('calibration_factor').value

        try:
            # Connect to Arduino via USB
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f"Connected to Arduino on {port}")
            
            self.get_logger().info("Taring scale... Please make sure it is empty.")
            time.sleep(2.0) # Wait for connection to stabilize
            
            # Clear any old data in the buffer and calculate tare offset
            self.ser.reset_input_buffer()
            tare_readings = []
            
            for _ in range(10):
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    tare_readings.append(float(line))
            
            if tare_readings:
                self.tare_offset = sum(tare_readings) / len(tare_readings)
                self.get_logger().info(f"Tare complete. Zero offset: {self.tare_offset}")
            else:
                self.tare_offset = 0.0
                self.get_logger().warn("Tare failed! No data from Arduino.")

        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            raise

        self.publisher_ = self.create_publisher(Float64, 'weight', 10)
        
        # Check for new serial data every 0.01 seconds
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        # Only read if there is data waiting in the USB buffer
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    raw_value = float(line)
                    
                    # Apply tare and calibration
                    weight = (raw_value - self.tare_offset) / self.calibration_factor
                    
                    # Hard limit filter: Ignore obvious hardware spikes over 50kg
                    #if abs(weight) > 50.0:
                    #    return
                    
                    # Publish the message with exactly 3 decimals
                    msg = Float64()
                    msg.data = round(weight, 3)
                    self.publisher_.publish(msg)
                    
            except ValueError:
                # Ignore incomplete strings (common during startup)
                pass
            except Exception as e:
                self.get_logger().error(f"Error reading serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LoadCellNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt: Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()