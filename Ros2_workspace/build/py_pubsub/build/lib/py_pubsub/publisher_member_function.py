# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#enable the software accecible GPIO to show a positive classification
#! echo 378 > /sys/class/gpio/export
#! echo out > /sys/class/gpio/gpio378/direction
#print('378',  file=open('/sys/class/gpio/export', 'w'))
#print('out',  file=open('/sys/class/gpio/gpio378/direction', 'w'))
    
#method to turn on/off the gpio pin
#def blink_MIO(on=False):
#    if(on==True):
#    	print('1',  file=open('/sys/class/gpio/gpio378/value', 'w'))
#        ! echo 1 > /sys/class/gpio/gpio378/value
#    else:
#    	print('0',  file=open('/sys/class/gpio/gpio378/value', 'w'))
#        ! echo 0 > /sys/class/gpio/gpio378/value


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def timer_callback(self):
        self.i += 1
        if self.i <= 101:
        	msg = String()
        	msg.data = 'Requesting reconfiguration {}'.format(self.i)
        	self.publisher_.publish(msg)
        	print(msg.data)




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
