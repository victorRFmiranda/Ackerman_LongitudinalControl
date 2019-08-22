#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64, String
from longitudinal_control.msg import IMU_Pose
from longitudinal_control.msg import Encoder



encoder_data = Encoder()

#####Constantes ROS#############
rospy.init_node('data_set', anonymous=True)


## Cria um arquivo com os dados
nome_arquivo = ''.join(("/home/coro/catkin_ws/src/longitudinal_control/steering_calibration/",time.strftime("%d_%b_%H_%M_%S", time.localtime()),'.txt'))
arquivo = open(nome_arquivo, 'w+')
arquivo.write("%1-Time (s)\t2-Velocity (m/s)\t3-Yaw (rad)\t4-ax\t4-ay\t4-az\n")


def get_encoder(msg):
	global encoder_data
	encoder_data = msg

def get_sensor(msg):
	global encoder_data
	global start
	yaw = round(msg.angles.yaw,3)
	ax = round(msg.accel.x,3)
	ay = round(msg.accel.y,3)
	az = round(msg.accel.z,3)
	string = str(round(rospy.get_rostime().to_sec()-start,2)) + "\t" + str(encoder_data.velocity) + "\t" + str(yaw) + "\t"
	string += str(ax) + "\t" + str(ay) + "\t" + str(az) + "\n"
	arquivo.write(string)
	arquivo.flush()
	
if __name__ == '__main__':
	try:
		start = rospy.get_rostime().to_sec()
		rospy.Subscriber('encoder_data', Encoder, get_encoder, queue_size=1)
		rospy.Subscriber('android_data', IMU_Pose, get_sensor, queue_size=1)
		rospy.spin()
	except rospy.ROSInterruptException:
		arquivo.close()
		pass
