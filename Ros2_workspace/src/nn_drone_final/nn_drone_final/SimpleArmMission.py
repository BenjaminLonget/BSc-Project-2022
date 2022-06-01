import rclpy
from rclpy.node import Node

import sys
sys.path.append("/usr/local/share/pynq-venv/lib/python3.8/site-packages")

from px4_msgs.msg import *
from std_msgs.msg import *

        
class SimpleArmMission(Node):
    def __init__(self):
        super().__init__('simple_arm_mission')

        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand
            , "/fmu/vehicle_command/in"
            , 10
        )

        self.classification_subscriber = self.create_subscription(
            String,
            'classification_topic',
            self.listener_callback,
            10
        )
        
        self.timesync_sub_ = self.create_subscription(
            Timesync
            , "/fmu/timesync/out"
            , self.process_timesync
            , 10
        )
        
        self.vehicle_status_subscriber_ = self.create_subscription(
            VehicleStatus
            , "/fmu/vehicle_status/out"
            , self.process_vehicle_status
            , 10
        )
        
        self.timestamp_ = 0
        self.armed =True
        
        

    def process_vehicle_status(self, status):
        if(status.arming_state == 2):
            self.armed = True
            print('Status: Armed')
        elif(status.arming_state == 1):
            self.armed = False
            print('Status: Disarmed')
            
    def listener_callback(self, msg):
        print('Armed status:  ', bool(self.armed))
        if msg.data.find("monitor") == 0 and not self.armed:
            self.arm()
        elif msg.data.find("electric fan") == 0 and self.armed:
            self.disarm()

        
    def arm(self):
        #self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        self.get_logger().info("Arm command send")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

        self.get_logger().info("Disarm command send")


    def publish_vehicle_command(self, command, param1 = 0.0, param2 = 0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher_.publish(msg)

    print('SimpleArmMission initialized')    

    def process_timesync(self, msg):
        self.timestamp_ = msg.timestamp
    	
    

def main(args=None):
    rclpy.init(args=args)

    simple_arm_mission = SimpleArmMission()
    
    rclpy.spin(simple_arm_mission)

    simple_mission.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()


