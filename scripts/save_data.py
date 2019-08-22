#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64, String
from longitudinal_control.msg import Trajectory
from longitudinal_control.msg import Encoder



encoder_data = Encoder()
torque = 0.0;

#####Constantes ROS#############
rospy.init_node('data_set', anonymous=True)
rate = rospy.Rate(4)


## Cria um arquivo com os dados
nome_arquivo = ''.join(("/home/coro/catkin_ws/src/longitudinal_control/log_experiments/",time.strftime("%d_%b_%Y_%H_%M_%S", time.localtime()),'.txt'))
arquivo = open(nome_arquivo, 'w+')
#arquivo.write("%1-Time(seg)\t2-Pos_Encoder\t3-x_ref\t4-PWM\t5-Torque\t6-Velo_Encoder(m/s)\n")
arquivo.write("%1-Time(seg)\t2-Pos_Encoder\t3-x_ref\t4-Velo_Encoder(m/s)\t5-Torque(N.m)\n")


def get_encoder(msg):
	global encoder_data
	encoder_data = msg

def get_torque(msg):
	global torque
	torque = msg.data

def get_trajectory(msg):
	global encoder_data
	global start
	global torque
	ref = round(msg.traj,3)
	string = str(round(rospy.get_rostime().to_sec()-start,2)) + "\t" + str(encoder_data.pos) + "\t" + str(ref) + "\t"
	string += "\t" + str(encoder_data.velocity) + "\t" + str(torque) + "\n"
	arquivo.write(string)
	arquivo.flush()


#def save_file(data):
#	global velocity
#	global arquivo
#	msg = data.data
#	msg += "\t" + str(velocity) + "\n"
#	arquivo.write(msg)
#	arquivo.flush() 
	
	
start = rospy.get_rostime().to_sec()
if __name__ == '__main__':
	try:
		rospy.Subscriber('encoder_data', Encoder, get_encoder, queue_size=1)
		rospy.Subscriber('torque', Float64, get_torque, queue_size=1)
		rospy.Subscriber('trajectory_point', Trajectory, get_trajectory, queue_size=1)
#		rospy.Subscriber('data', String, save_file, queue_size=1)
		rospy.spin()
	except rospy.ROSInterruptException:
		arquivo.close()
		pass
