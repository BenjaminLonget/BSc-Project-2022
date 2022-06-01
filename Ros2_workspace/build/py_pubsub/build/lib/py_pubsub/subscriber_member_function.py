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
import sys
sys.path.append("/usr/local/share/pynq-venv/lib/python3.8/site-packages")
import os, warnings
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pynq import PL
from pynq import Overlay
from pynq import GPIO
import time

overlay=Overlay('/home/xilinx/pynq/overlays/GPIO_Wstatic/full.bit')

class MinimalSubscriber(Node):

    def __init__(self):

        super().__init__('minimal_subscriber')
        self.i = 0
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.time_log = []
        self.avg = 0

    def listener_callback(self, msg):
        self.i += 1
        if self.i <= 100:
        	if(self.i%2 == 0):
        		print('{} Downloading pr_1.bit.'.format(self.i))
        		start = time.time()
        		overlay.pr_download('pr_hier', '/home/xilinx/pynq/overlays/GPIO_Wstatic/pr_1.bit')
        		end = time.time()
        		self.time_log.append((end-start)*1000)
        		self.avg += (end-start)*1000
        		print('pr_1.bit took {} ms'.format((end-start)*1000))
        	else:
        		print('{} Downloading pr_2.bit'.format(self.i))
        		start = time.time()
        		overlay.pr_download('pr_hier', '/home/xilinx/pynq/overlays/GPIO_Wstatic/pr_2.bit')
        		end = time.time()
        		self.time_log.append((end-start)*1000)
        		self.avg += (end-start)*1000
        		print('pr_2.bit took {} ms'.format((end-start)*1000))
        else:
        	print('\nAverage: {} ms'.format(self.avg/100))
        	


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
