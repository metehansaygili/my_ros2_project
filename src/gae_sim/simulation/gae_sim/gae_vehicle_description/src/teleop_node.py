import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String, Int32
from sensor_msgs.msg import Joy
from gae_msgs.msg import GaeControlCmd
from pynput import keyboard

import threading, os

LATERAL_CONTROLLER_TOPIC = '/controller/steering'
SIGNAL_TOPIC = '/planner/signal'
VEHICLE_COMMAND_TOPIC = '/vehicle/control'
EMERGENCY_STOP_TOPIC = '/vehicle/emergency_stop'
STOP_TOPIC = '/planner/stop'
CONTROLLER = '/controller/vehicle_control'

def clear_terminal():
    os.system("clear")

class VehicleCommandNode(Node):
    def __init__(self):
        super().__init__('vehicle_command_node')

        self.lock = threading.Lock()

        self.lateral_controller_sub = self.create_subscription(Int32, LATERAL_CONTROLLER_TOPIC, self.lateral_controller_callback, 10)
        self.signal_sub = self.create_subscription(Float32, SIGNAL_TOPIC, self.signal_callback, 10)
        self.emergency_stop_sub = self.create_subscription(Bool, EMERGENCY_STOP_TOPIC, self.emergency_stop_callback, 10)
        self.convert_sub = self.create_subscription(GaeControlCmd, CONTROLLER, self.controller_sub, 10)

        self.timer = self.create_timer(0.1, self.control_function)
        self.pressed_keys = set()
        
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        self.vehicle_command_pub = self.create_publisher(GaeControlCmd, VEHICLE_COMMAND_TOPIC , 10)

        self.throttle = 0
        self.steering = 0
        self.driving_mode = 0
        self.gear = 0
        self.autonomous_mode = 0
        self.brake = 0  
        self.mechanical_brake = 0
        self.terminator = 0

        self.autonomous_mode_throttle = 45
        self.frame = 0

        self.signal = 0

        self.prev_mode = 0                
        self.prev_mechanical_brake = 0    
        self.prev_autonomous = 0          
        self.prev_gear = 0                
        self.prev_start_button = 0
        self.last_command = GaeControlCmd()

    def on_press(self, key):
        try:
            self.pressed_keys.add(key.char)
        except AttributeError:
            if key == keyboard.Key.esc:
                self.pressed_keys.add('esc')

    def controller_sub(self, msg):
        if self.autonomous_mode:
            self.last_command = msg
            print("Received command from controller node.")
            self.vehicle_command_pub.publish(msg)

    def on_release(self, key):
        try:
            self.pressed_keys.discard(key.char)
        except AttributeError:
            if key == keyboard.Key.esc:
                self.pressed_keys.discard('esc')

    def control_function(self):
        with self.lock:
            if 'w' in self.pressed_keys:
                self.throttle = 200
                self.gear = 1
            else:
                self.throttle = 0
            
            if 'a' in self.pressed_keys:
                self.steering = 3600
            elif 'd' in self.pressed_keys:
                self.steering = 0
            else:
                self.steering = 1800
            
            if 'z' in self.pressed_keys:
                self.mechanical_brake = 1
            else:
                self.mechanical_brake = 0
            
            if 's' in self.pressed_keys:
                self.throttle = 100
                self.gear = 2
            else:
                self.brake = 0

            if 'f' in self.pressed_keys:
                self.brake = 200
            else:
                self.brake = 0
            
            if 'x' in self.pressed_keys:
                self.autonomous_mode = self.autonomous_mode ^ 1

            if not self.autonomous_mode:
                clear_terminal()
                print("[JOYNODE_PUB]", "REMOTE CONTROL MODE")
                if not self.terminator:
                    vehicle_command = GaeControlCmd()
                    vehicle_command.throttle = self.throttle
                    vehicle_command.steering = self.steering
                    vehicle_command.brake = self.brake
                    vehicle_command.mechanical_brake = self.mechanical_brake
                    vehicle_command.gear = self.gear
                    vehicle_command.mode_auto = self.autonomous_mode
                    vehicle_command.signal = 0
                    if vehicle_command.mechanical_brake == 1:
                        vehicle_command.brake = 1000
                    print('Throttle: ', vehicle_command.throttle)
                    print('Steering: ', vehicle_command.steering)
                    print('Brake: ', vehicle_command.brake)
                    if vehicle_command.gear == 0:
                        print('Gear: Neutral')
                    elif vehicle_command.gear == 1:
                        print('Gear: Drive')
                    else:
                        print('Gear: Reverse')
                    if vehicle_command.mechanical_brake == 0:
                        print('Mechanical Brake: Off')
                    else:
                        print('Mechanical Brake: On')
                    if vehicle_command.mode_auto == 0:
                        print('Autonomous Mode: Off')
                    else:
                        print('Autonomous Mode: On')
                    if vehicle_command.signal == 0:
                        print('Signal: Off', )
                    elif vehicle_command.signal == 1:
                        print('Signal: Left', )
                    elif vehicle_command.signal == 2:
                        print('Signal: Right', )
                    else:
                        print('Signal: Stop')
                    self.vehicle_command_pub.publish(vehicle_command)
    
    def lateral_controller_callback(self, steering):
        with self.lock:
            if(self.autonomous_mode):
                clear_terminal()
                print("[JOYNODE_PUB]", "AUTONOMOUS MODE")
                vehicle_command = GaeControlCmd()
                vehicle_command.throttle = self.autonomous_mode_throttle
                vehicle_command.steering = int(steering.data)
                vehicle_command.brake = 0
                vehicle_command.mechanical_brake = 0
                vehicle_command.gear = 1
                vehicle_command.mode_auto = 1
                vehicle_command.signal = self.signal

                print('Throttle: ', vehicle_command.throttle)
                print('Steering: ', vehicle_command.steering)
                print('Brake: ', vehicle_command.brake)
                if vehicle_command.gear == 0:
                    print('Gear: Neutral')
                elif vehicle_command.gear == 1:
                    print('Gear: Drive')
                else:
                    print('Gear: Reverse')
                if vehicle_command.mechanical_brake == 0:
                    print('Mechanical Brake: Off')
                else:
                    print('Mechanical Brake: On')
                if vehicle_command.mode_auto == 0:
                    print('Autonomous Mode: Off')
                else:
                    print('Autonomous Mode: On')
                if vehicle_command.signal == 0:
                    print('Signal: Off', )
                elif vehicle_command.signal == 1:
                    print('Signal: Left', )
                elif vehicle_command.signal == 2:
                    print('Signal: Right', )
                else:
                    print('Signal: Stop')

                self.vehicle_command_pub.publish(vehicle_command)

    def signal_callback(self, signal):
        with self.lock:
            if(not self.autonomous_mode):
                return
            
            self.signal = signal.data

    def emergency_stop_callback(self, msg):
        with self.lock:
            if msg.data:
                vehicle_command = GaeControlCmd()
                vehicle_command.throttle = 0
                vehicle_command.steering = 0
                vehicle_command.brake = 1000
                vehicle_command.mechanical_brake = 0 # NOT SURE FOR USING THE MECHANICAL BRAKE
                vehicle_command.gear = 0
                vehicle_command.mode_auto = 0
                vehicle_command.signal = 0

                self.vehicle_command_pub.publish(vehicle_command)

"""
GaeControlCmd:

uint16 throttle         # Range: 0-100
uint16 steering         # Range: 0-3600
uint16 brake            # Range: 0-10000
uint8 mechanical_brake  # Range: 0-1 (0: off, 1: on)
uint8 gear              # Range: 0-2 (0: neutral, 1: drive, 2: reverse)
uint8 mode_auto         # Range: 0-1 (0: manual, 1: auto)
uint8 signal            # Range: 0-3 (0: off, 1: left, 2: right 3: stop)
"""

def main(args=None):
    rclpy.init(args=args)

    vehicle_command_node = VehicleCommandNode()

    rclpy.spin(vehicle_command_node)

    vehicle_command_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()