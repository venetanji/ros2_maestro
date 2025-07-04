import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import os
from .maestro import MaestroController

class MaestroMotorNode(Node):
    def __init__(self):
        super().__init__('maestro_motor_node')
        port = self.declare_parameter('port', 'COM6').get_parameter_value().string_value
        baudrate = self.declare_parameter('baudrate', 9600).get_parameter_value().integer_value
        yaml_file = self.declare_parameter('yaml_file', os.path.join(os.path.dirname(__file__), '../config/motor_ranges.yaml')).get_parameter_value().string_value
        self.get_logger().info(f"Connecting to Maestro on {port} baud {baudrate} with config {yaml_file}")
        try:
            self.maestro = MaestroController(port=port, baudrate=baudrate, yaml_file=yaml_file)
            self.active_channels = sorted(self.maestro.channels.keys())
            self.simulation = False
        except Exception as e:
            self.get_logger().warn(f"Could not connect to Maestro: {e}. Running in simulation mode.")
            self.active_channels = list(range(6))
            self.simulation = True
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/face_motors',
            self.listener_callback,
            10)
        self.motor_positions = np.zeros(len(self.active_channels))

    def listener_callback(self, msg):
        arr = np.array(msg.data)
        if len(arr) != len(self.active_channels):
            self.get_logger().warn(f"Received {len(arr)} positions, expected {len(self.active_channels)}. Truncating or padding.")
            arr = arr[:len(self.active_channels)]
            if len(arr) < len(self.active_channels):
                arr = np.pad(arr, (0, len(self.active_channels)-len(arr)), 'constant')
        self.motor_positions = np.clip(arr, -1.0, 1.0)
        
        if not self.simulation:
            # Use the more efficient set_multiple_servos method if channels are contiguous
            # First convert normalized values to quarter-microsecond values
            qus_positions = {}
            for i, channel in enumerate(self.active_channels):
                value = float(self.motor_positions[i])
                cfg = self.maestro.channels.get(channel)
                if cfg:
                    value = max(-1.0, min(1.0, value))
                    if value <= 0:
                        qus = int(cfg['neutral_position'] + (cfg['neutral_position'] - cfg['min_position']) * value)
                    else:
                        qus = int(cfg['neutral_position'] + (cfg['max_position'] - cfg['neutral_position']) * value)
                    qus_positions[channel] = qus
            
            # If we have positions to set, use set_multiple_servos
            if qus_positions:
                self.maestro.set_multiple_servos(qus_positions)
        else:
            self.get_logger().info(f"[SIM] Motor positions: {self.motor_positions}")

def main(args=None):
    rclpy.init(args=args)
    node = MaestroMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
