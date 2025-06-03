import serial
import yaml
import time

class MaestroController:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200, yaml_file=None):
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
        time.sleep(0.005)  # Allow time for the command to be processed
    
    def set_multiple_servos(self, positions_dict):
        """
        Set multiple servo positions at once using the compact protocol.
        This is more efficient than setting them one at a time.
        
        Args:
            positions_dict: Dictionary mapping channel numbers to target positions (in quarter microseconds)
                            Example: {0: 6000, 1: 7000, 2: 5000}
        """
        if self.ser is None:
            print(f"[MaestroController] Not connected. Simulating set_multiple_servos({positions_dict})")
            return
            
        if not positions_dict:
            return
            
        # Sort channels to ensure they're processed in order
        channels = sorted(positions_dict.keys())
        
        # Find any missing channels and fill them with neutral positions
        min_channel = min(channels)
        max_channel = max(channels)
        
        # Find all gaps in the channel sequence and fill them with neutral positions
        complete_positions = positions_dict.copy()
        for channel_num in range(min_channel, max_channel + 1):
            if channel_num not in channels:
                # If a channel is missing, check if we have configuration for it
                cfg = self.channels.get(channel_num)
                if cfg and 'neutral_position' in cfg:
                    # Add neutral position for missing channel
                    neutral_qus = cfg['neutral_position']
                    complete_positions[channel_num] = neutral_qus
                    print(f"[MaestroController] Filling in missing channel {channel_num} with neutral position {neutral_qus}")
        
        # Now we have a complete set of positions, but they might still be non-contiguous
        # if we don't have configs for some channels
        channels = sorted(complete_positions.keys())
        
        # Break it into contiguous chunks and send each chunk separately
        chunks = []
        current_chunk = [channels[0]]
        
        for i in range(1, len(channels)):
            if channels[i] == channels[i-1] + 1:
                current_chunk.append(channels[i])
            else:
                chunks.append(current_chunk)
                current_chunk = [channels[i]]
        
        if current_chunk:
            chunks.append(current_chunk)
        
        # Send each contiguous chunk of channels
        for chunk in chunks:
            if len(chunk) == 1:
                # Just a single channel, use individual command
                channel = chunk[0]
                self.set_servo_qus(channel, complete_positions[channel])
            else:
                # Build the command for this contiguous chunk
                # Compact protocol: 0x9F, number of targets, first channel number, first target low bits, 
                # first target high bits, second target low bits, second target high bits, …
                cmd = bytearray([0x9F, len(chunk), chunk[0]])
                
                for channel in chunk:
                    target = int(complete_positions[channel])
                    lsb = target & 0x7F
                    msb = (target >> 7) & 0x7F
                    cmd.extend([lsb, msb])
                
                self.ser.write(cmd)
                time.sleep(0.01)  # Small delay between commands

    def close(self):
        if self.ser:
            self.ser.close()
            self.ser = None
