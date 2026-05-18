#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
import serial
import csv
import time
import threading
import struct
import can
from datetime import datetime

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
LOG_RATE_HZ = 20.0
TORQUE_ALPHA = 0.15
CALIBRATION_FACTOR = 4507.0

# --- CAN BUS CONFIGURATION ---
CAN_IFACE = 'can0'
BITRATE = 250_000
CMD_GET_ENCODER_EST = 0x09
CMD_GET_TORQUES = 0x1C

# Map ODrive Node IDs to Joint Names
NODE_TO_JOINT = {
    0: 'joint1',
    1: 'joint2',
    2: 'joint3'
}
JOINT_NAMES = list(NODE_TO_JOINT.values())


class JacobianLoggerNode(Node):
    def __init__(self):
        super().__init__('jacobian_logger_node')

        # --- State Variables ---
        self.current_force_n = 0.0
        self.filtered_torques = {name: 0.0 for name in JOINT_NAMES}
        self.current_positions = {name: 0.0 for name in JOINT_NAMES}

        # --- Initialize Load Cell ---
        self.ser, self.tare_offset = self.init_load_cell(SERIAL_PORT, BAUD_RATE)
        if not self.ser:
            self.get_logger().error("Failed to connect to load cell. Exiting.")
            sys.exit(1)

        # Start serial reader in a background thread
        self.serial_thread = threading.Thread(target=self.read_load_cell_thread, daemon=True)
        self.serial_thread.start()

        # --- Initialize CAN Bus ---
        try:
            self.bus = can.interface.Bus(interface='socketcan', channel=CAN_IFACE, bitrate=BITRATE)
            self.get_logger().info(f"Connected to CAN bus on {CAN_IFACE}")
        except Exception as e:
            self.get_logger().error(f"ERROR opening CAN: {e}")
            sys.exit(1)

        # Start CAN reader in a background thread
        self.can_thread = threading.Thread(target=self.read_can_thread, daemon=True)
        self.can_thread.start()

        # --- Initialize CSV log file ---
        self.start_time = time.time()
        self.filename = f"jacobian_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(self.filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            headers = ['Time_s']
            for name in JOINT_NAMES:
                headers.extend([f'{name}_Pos', f'{name}_Torque'])
            headers.append('LoadCell_Force_N')
            writer.writerow(headers)

        # Timer for synchronous data logging
        self.log_timer = self.create_timer(1.0 / LOG_RATE_HZ, self.log_data_callback)
        self.get_logger().info(f"Initialized. Logging data to {self.filename} at {LOG_RATE_HZ} Hz.")

    def init_load_cell(self, port, baud):
        self.get_logger().info(f"Connecting to Arduino on {port}...")
        try:
            ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info("Taring scale. Please ensure it is unloaded.")
            time.sleep(2.0)

            ser.reset_input_buffer()
            tare_readings = []

            for _ in range(10):
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    try:
                        tare_readings.append(float(line))
                    except ValueError:
                        pass

            if tare_readings:
                tare_offset = sum(tare_readings) / len(tare_readings)
                self.get_logger().info(f"Tare complete. Raw zero offset: {tare_offset:.2f}")
            else:
                tare_offset = 0.0
                self.get_logger().warning("WARNING: Tare failed. No valid data received from Arduino.")

            time.sleep(1.0)
            return ser, tare_offset

        except Exception as e:
            self.get_logger().error(f"Serial connection error: {e}")
            return None, 0.0

    def read_load_cell_thread(self):
        """Background thread for reading raw serial data continuously."""
        while rclpy.ok() and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        raw_force = float(line)
                        self.current_force_n = (raw_force - self.tare_offset) / CALIBRATION_FACTOR
            except Exception:
                pass

    def read_can_thread(self):
        """Background thread for reading ODrive CAN bus directly."""
        while rclpy.ok():
            try:
                m = self.bus.recv(timeout=0.1)
                if m:
                    node = m.arbitration_id >> 5
                    cmd = m.arbitration_id & 0x1F

                    # Ensure the message is from one of our expected joints
                    if node in NODE_TO_JOINT:
                        joint_name = NODE_TO_JOINT[node]

                        # Decode Encoder Estimates (Position)
                        if cmd == CMD_GET_ENCODER_EST:
                            pos, vel = struct.unpack('<ff', m.data[:8])
                            self.current_positions[joint_name] = pos
                            
                        # Decode Torque Estimates
                        elif cmd == CMD_GET_TORQUES:
                            _, raw_torque = struct.unpack('<ff', m.data[:8])
                            self.filtered_torques[joint_name] = (TORQUE_ALPHA * raw_torque) + (
                                    (1.0 - TORQUE_ALPHA) * self.filtered_torques[joint_name])
            except Exception as e:
                pass # Ignore occasional CAN read timeouts

    def log_data_callback(self):
        """Log the most recent synchronized sample of all data to CSV."""
        current_time = time.time() - self.start_time

        row = [round(current_time, 4)]
        for name in JOINT_NAMES:
            row.extend([round(self.current_positions[name], 4), round(self.filtered_torques[name], 4)])
        row.append(round(self.current_force_n, 3))

        with open(self.filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

        self.get_logger().info(
            f"Fz: {self.current_force_n:.2f} N | T2: {self.filtered_torques['joint2']:.2f} Nm | T3: {self.filtered_torques['joint3']:.2f} Nm")


def main(args=None):
    rclpy.init(args=args)
    node = JacobianLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupt received. Terminating logging.")
    finally:
        if hasattr(node, 'ser') and node.ser:
            node.ser.close()
        if hasattr(node, 'bus') and node.bus:
            node.bus.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()