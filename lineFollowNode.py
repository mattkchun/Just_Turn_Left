#/usr/bin/env python

#import libraries and color segmentation code
import rospy
import cv2
import time
import numpy as np
from zed import Zed_converter
from cv_bridge import CvBridge, CvBridgeError
from rby_segmentation import cd_color_segmentation
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image

#initalize global variables
DRIVE_TOPIC = "/drive"
IMAGE_TOPIC = "/zed/zed_node/color_seg_output"

AUTONOMOUS_MODE = True

class driveStop(object):
	"""class that will help the robot drive and stop at certain conditions
	"""
	def __init__(self):
		"""initalize the node"""
		rospy.init_node("driveStop")
		self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size = 1)
                self.image_pub = rospy.Publisher(IMAGE_TOPIC, Image, queue_size = 2)
		rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)

		""" initialize the box dimensions"""
		self.flag_box = ((0,0),(0,0))
		self.flag_center = (0,0)
		self.flag_size = 0
		self.image_dimen = (0,0)

                """driving stuff"""
		self.cmd = AckermannDriveStamped()
		self.cmd.drive.speed = 0
		self.cmd.drive.steering_angle = 0
	
		"""get the camera data from the class Zed_converter in Zed"""
		self.camera_data = Zed_converter(False, save_image = False)
                self.imagePush = None

                self.bridge = CvBridge()
		self.min_value=0

	def size_calc(self):
		""" calculate the x and y size of the box in pixels"""
		pix_width = self.flag_box[1][0] - self.flag_box[0][0]
		pix_height = self.flag_box[1][1] - self.flag_box[0][1]	

		self.flag_size = pix_width*pix_height
                image_props=self.camera_data.cv_image.shape
                self.image_dimen = (image_props[1],image_props[0])
                box_x_mid=self.flag_box[0][0]+float(self.flag_box[1][0]-self.flag_box[0][0])/2
                box_y_mid=self.flag_box[0][1]+float(self.flag_box[1][1]-self.flag_box[1][0])/2
                self.flag_center= (box_x_mid, box_y_mid)

	def driveStop_car_callback(self,data):
		"""laser scan callback function"""
		#checks if the image is valid first
		while self.camera_data.cv_image is None:
			time.sleep(0.5)
			print("sleeping")

		#applies the current filter to the image and stores the image in imagePush
		self.flag_box, self.imagePush = cd_color_segmentation(self.camera_data.cv_image)

		#finds the size of the box
		self.size_calc()
		
                #outputs the image to the IMAGE_TOPIC
                try:
                        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.imagePush, "bgr8"))
                except CvBridgeError as e:
                        print("Error bridging Zed image", e)
		
                if AUTONOMOUS_MODE:
			self.drive()
		else:
			pass
	
	def drive(self):
		#infinite loop
                if self.image_dimen[0]==0 and self.image_dimen[1]==0:
                        self.cmd.drive.speed=0
                        self.cmd.drive.steering_angle=0
                        print("stopping")
                else:
                        print("driving")
                        self.cmd.drive.speed=1
                        if not (self.flag_center[0]-self.image_dimen[0])==0: 
                                self.cmd.drive.steering_angle=-1*self.flag_center[0]/(self.image_dimen[0]/2)
                                ##turning
                        else:
                                self.cmd.drive.steering_angle=0
                

def main():
	global AUTONOMOUS_MODE
	try:
		ic = driveStop()
		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			if AUTONOMOUS_MODE:
				ic.pub.publish(ic.cmd)
			
	except rospy.ROSInterruptException:
		exit()


if __name__ == "__main__":
	main()
