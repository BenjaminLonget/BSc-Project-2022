import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

import sys
sys.path.append("/usr/local/share/pynq-venv/lib/python3.8/site-packages")

from pynq_dpu import DpuOverlay
overlay = DpuOverlay("dpu.bit")
import os
import time
import numpy as np
import cv2


overlay.load_model("/home/xilinx/jupyter_notebooks/pynq-dpu/dpu_tf_inceptionv1.xmodel")
#overlay.load_model("/home/xilinx/DPU-PYNQ/pynq_dpu/notebooks/dpu_tf_inceptionv1.xmodel.link")  #path til model måske?


#functions to calculate softmax and output the estimated classification
def calculate_softmax(data):
    result = np.exp(data)
    return result

def predict_label(softmax):
    with open("/home/xilinx/DPU-PYNQ/pynq_dpu/notebooks/img/words.txt", "r") as f:
        lines = f.readlines()
    return lines[np.argmax(softmax)-1]



#instanciate the dpu and define input/output dimensions
dpu = overlay.runner
inputTensors = dpu.get_input_tensors()
outputTensors = dpu.get_output_tensors()
shapeIn = tuple(inputTensors[0].dims)
shapeOut = tuple(outputTensors[0].dims)
outputSize = int(outputTensors[0].get_data_size() / shapeIn[0])
softmax = np.empty(outputSize) 
output_data = [np.empty(shapeOut, dtype=np.float32, order="C")]
input_data = [np.empty(shapeIn, dtype=np.float32, order="C")]
image = input_data[0]

#enable the software accecible GPIO to show a positive classification
#! echo 378 > /sys/class/gpio/export
#! echo out > /sys/class/gpio/gpio378/direction

#method to turn on/off the gpio pin
#def blink_MIO(on=False):
#    if(on==True):
#        ! echo 1 > /sys/class/gpio/gpio378/value
#    else:
#        ! echo 0 > /sys/class/gpio/gpio378/value




class NnNode(Node):

	def __init__(self):
		super().__init__('NN_node')
		self.publisher_ = self.create_publisher(
			String, 
			'classification_topic', 
			10)
		
		self.bridge = CvBridge()
        	#create subscriber
		self.subscription = self.create_subscription(
			Image,
			'frame_topic',
			self.listener_callback,
			10)
		self.subscription  # prevent unused variable warning
        
       	#initial message to start the cam/nn node loop
		start_msg = String()
		start_msg.data = 'start'
		#self.get_logger().info('Sending startup message')
		self.publisher_.publish(start_msg)
		print('NN Node initialized')
        
	def listener_callback(self, data):
		time1 = time.time()
		msg = String()
		#print('Image recieved, running NN')
		self.get_logger().info('Image recieved')
		
		preprocessed = self.bridge.imgmsg_to_cv2(data)
		image[0,...] = preprocessed.reshape(shapeIn[1:])
		job_id = dpu.execute_async(input_data, output_data)
		dpu.wait(job_id)
		temp = [j.reshape(1, outputSize) for j in output_data]
		softmax = calculate_softmax(temp[0][0])

		msg.data = "{}".format(predict_label(softmax))
		#msg.data = 'Hello World: %d' % self.i
		self.publisher_.publish(msg)
		time2 = time.time()
		self.time_total += time2-time1
		
		self.get_logger().info('Classification: "%s"' % msg.data)
		
		print('Time for classification: {}'.format(time2-time1))
		print('FPS: {}'.format(self.i/self.time_total)



def main(args=None):
    rclpy.init(args=args)

    nn_node = NnNode()

    rclpy.spin(nn_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('Stopping node')
    del overlay
    del dpu
    nn_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
