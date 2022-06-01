import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import sys
sys.path.append("/usr/local/share/pynq-venv/lib/python3.8/site-packages")

from px4_msgs.msg import *
from std_msgs.msg import *


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        #self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        #self.subscription = self.create_subscription(
        #    String,
        #    'classification_topic',
        #    self.listener_callback,
        #    10)
            
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand
            , "/fmu/vehicle_command/in"
            , 10
        )

        self.timesync_sub_ = self.create_subscription(
            Timesync
            , "/fmu/timesync/out"
            , self.process_timesync
            , 10
        )

        self.timestamp_ = 0
        
 
    def timer_callback(self):
        if(self.i % 2 == 0):
            self.arm()
        else:
            self.disarm()
    
        self.i += 1
        
        #self.subscription  # prevent unused variable warning
        print('Minimal subscriber node initialized')
        
    
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
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
    	
    
    #def listener_callback(self, msg):
    #    self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('Stopping minimal subscriber node')
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
