import serial
import yaml

class MaestroController:
    def __init__(self, port="COM6", baudrate=9600, yaml_file=None):
        self.port = port
        self.baudrate = baudrate
        self.channels = {}
        self.ser = None
        if yaml_file:
            self.load_motor_ranges_from_yaml(yaml_file)
        self.connect()

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
        except Exception as e:
            print(f"[MaestroController] Could not open serial port: {e}")
            self.ser = None
            
    def load_motor_ranges_from_yaml(self, yaml_file):
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        
        # Load the new format with channel_X keys
        self.channels = {}
        for key, value in data.items():
            if key.startswith('channel_'):
                # Convert channel_0 to 0 for internal use
                channel_num = int(key.split('_')[1])
                self.channels[channel_num] = value
        
        return list(self.channels.keys())

    def set_servo_normalized(self, channel, value):
        if self.ser is None:
            print(f"[MaestroController] Not connected. Simulating set_servo_normalized({channel}, {value})")
            return
        
        cfg = self.channels.get(channel)
        if cfg is None:
            print(f"[MaestroController] Channel {channel} not configured.")
            return
        
        value = max(-1.0, min(1.0, value))
        
        # Use position values from new format
        if value <= 0:
            qus = int(cfg['neutral_position'] + (cfg['neutral_position'] - cfg['min_position']) * value)
        else:
            qus = int(cfg['neutral_position'] + (cfg['max_position'] - cfg['neutral_position']) * value)
                
        self.set_servo_qus(channel, qus)

    def set_servo_qus(self, channel, qus):
        if self.ser is None:
            print(f"[MaestroController] Not connected. Simulating set_servo_qus({channel}, {qus})")
            return
        # Pololu Maestro Set Target command: 0x84, channel, target low bits, target high bits
        target = int(qus)
        lsb = target & 0x7F
        msb = (target >> 7) & 0x7F
        cmd = bytes([0x84, int(channel), lsb, msb])
        self.ser.write(cmd)

    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None
