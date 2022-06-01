import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

import sys
sys.path.append("/usr/local/share/pynq-venv/lib/python3.8/site-packages")

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import cv2
import time

#functions to normalize the image, this way, any size image is accepted
_R_MEAN = 123.68
_G_MEAN = 116.78
_B_MEAN = 103.94

MEANS = [_B_MEAN,_G_MEAN,_R_MEAN]

def resize_shortest_edge(image, size):
    H, W = image.shape[:2]
    if H >= W:
        nW = size
        nH = int(float(H)/W * size)
    else:
        nH = size
        nW = int(float(W)/H * size)
    return cv2.resize(image,(nW,nH))

def mean_image_subtraction(image, means):
    B, G, R = cv2.split(image)
    B = B - means[0]
    G = G - means[1]
    R = R - means[2]
    image = cv2.merge([R, G, B])
    return image

def BGR2RGB(image):
    B, G, R = cv2.split(image)
    image = cv2.merge([R, G, B])
    return image

def central_crop(image, crop_height, crop_width):
    image_height = image.shape[0]
    image_width = image.shape[1]
    offset_height = (image_height - crop_height) // 2
    offset_width = (image_width - crop_width) // 2
    return image[offset_height:offset_height + crop_height, offset_width:
                 offset_width + crop_width, :]

def normalize(image):
    image=image/256.0
    image=image-0.5
    image=image*2
    return image

def preprocess_fn(image, crop_height = 224, crop_width = 224):
    image = BGR2RGB(image)
    image = resize_shortest_edge(image, 256)
    image = central_crop(image, crop_height, crop_width)
    image = normalize(image)
    return image
    
#def get_cam():
#	img = cv2.VideoCapture(0)
#	if img.isOpened():
#		_,frame = img.read()
#		img.release() #releasing camera immediately after capturing picture
#	if _ and frame is not None:
#		#writing the image to a file because its easier
#		cv2.imwrite('img_cam.jpg', frame)

	
	
class CameraNode(Node):
	def __init__(self):
		super().__init__('cam_node')
		#create publisher
		self.publisher_ = self.create_publisher(
			Image, 
			'frame_topic', 
			10)
			
		self.video = cv2.VideoCapture(0)
		self.br = CvBridge()	#bridge between ros and cv2

		#create subscriber
		self.subscription = self.create_subscription(
			String,
			'classification_topic',
			self.listener_callback,
            		10)
		self.subscription  # prevent unused variable warning
		print('Camera node initialized')
	

	
	

	def listener_callback(self, msg):
		self.get_logger().info('I heard: "%s"' % msg.data)
		print('Acquiring frame')
		#self.get_logger().info('Acquiring frame')
		#get_cam()
		time1 = time.time()
		_, frame = self.video.read()
		frame = cv2.resize(frame, (224, 224))
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		frame = normalize(frame)
#		preprocessed = preprocess_fn(frame)
#		preprocessed = preprocess_fn(cv2.imread('img_cam.jpg'))
		image_msg = self.br.cv2_to_imgmsg(frame)
		time2 = time.time()
		print('Time for preprocess: ')
		print(time2 - time1)
		self.get_logger().info('Sending frame')
		self.publisher_.publish(image_msg)
        


def main(args=None):
    rclpy.init(args=args)

    camera_node = CameraNode()

    rclpy.spin(camera_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('Stopping node')
    camera_node.video.release()
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

