#!/usr/bin/env python

import rospy
import time
import math
from android_class import Android
from longitudinal_control.msg import IMU_Pose
from longitudinal_control.msg import Encoder
from geometry_msgs.msg import Point

position = 0.0


def get_velocity(msg):
	global position
	position = round(msg.pos,3)

def calc_position(angle):
	global position
	x = position*math.cos(angle)
	y = position*math.sin(angle)
	return x, y


def publisher():
	pub_android_data = rospy.Publisher('android_data', IMU_Pose, queue_size = 1)
	pub_cartesian_pose = rospy.Publisher('xy_pose', Point, queue_size = 1)
	rospy.init_node('Android', anonymous=True)
	rate = rospy.Rate(10)
	pose = Point()
	msg = IMU_Pose()
	android = Android()
	while not rospy.is_shutdown():
		if android.getAndroidData():
			msg.angles.roll, msg.angles.pitch, msg.angles.yaw = android.getAngles()	
			print round(msg.angles.yaw,3)
			pose.x, pose.y = calc_position(round(msg.angles.yaw,3))
			msg.GPS_Data.latitude, msg.GPS_Data.longitude, msg.vel_GPS = android.getGPSData()
			msg.accel.x, msg.accel.y, msg.accel.z = android.getAccelData()
			msg.position_GPS.x, msg.position_GPS.y = android.getGPSPos()
			pub_cartesian_pose.publish(pose)
			pub_android_data.publish(msg)
		rate.sleep()


if __name__ == '__main__':
	try:
		rospy.Subscriber('encoder_data', Encoder, get_velocity, queue_size=1)
		publisher()
	except rospy.ROSInterruptException:
		pass



