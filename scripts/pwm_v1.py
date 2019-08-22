#!/usr/bin/env python

import rospy
import time
import Adafruit_PCA9685
from longitudinal_control.msg import IMU_Pose

SERVANT = Adafruit_PCA9685.PCA9685()

i = 280
start = 0.0

# 280 max direita
# 400 frente
# 550 esquerda
SERVANT.set_pwm_freq(60)
SERVANT.set_pwm(0, 0, 0)
velo = 280
SERVANT.set_pwm(3, 0, velo)


## Cria um arquivo com os dados
nome_arquivo = ''.join(("/home/coro/catkin_ws/src/longitudinal_control/steering_calibration/",time.strftime("%d_%b_%H_%M_%S", time.localtime()),'.txt'))
arquivo = open(nome_arquivo, 'w+')
arquivo.write("%1-PWM\t2-Yaw (rad)\n")


def routine(msg):
	global i
	global start
	if (i > 550):
		rospy.signal_shutdown("Finish")
	yaw = round(msg.angles.yaw,3)
	SERVANT.set_pwm(3, 0, i)
	string = str(i) + "\t" + str(yaw) + "\n"
	arquivo.write(string)
	arquivo.flush()
	print i
	i = i+1


##############################################
############ MAIN 
##############################################
if __name__ == '__main__':
	try:	
		rospy.init_node('steering_calibration', anonymous=True)
		start = rospy.get_rostime().to_sec()
		rospy.Subscriber('android_data', IMU_Pose, routine, queue_size=1)
		rospy.spin()
		SERVANT.set_pwm(3, 0, 550)
	except rospy.ROSInterruptException:
		SERVANT.set_pwm(3, 0, 280)
		pass
