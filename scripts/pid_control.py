#!/usr/bin/env python

import time
import threading, os
from pid_class import PID
import numpy as np
import rospy
import Adafruit_PCA9685
from std_msgs.msg import Float64, String
from longitudinal_control.msg import Trajectory
from longitudinal_control.msg import Encoder
import matplotlib.pyplot as plt


######## Constantes ###########
KP = 14.4 
KI = 14   #-> Ganhos 10 Hz 
KD = 5.1 
#KP = 15.8 
#KI = 14   #-> Ganhos 5 Hz 
#KD = 5.1 
TS = 0.16        ## Periodo de amostragem dos dados
ETOL = 0.01     ## Tolerancia do erro do controlador
CONTROLADOR = PID(KP,KI,KD,TS,ETOL)     #chamada da classe do controlador
ATUADOR = Adafruit_PCA9685.PCA9685()    #chamada da classe da placa de comunicao com atuador
ATUADOR.set_pwm_freq(60)                #Frequencia do atuador
PWM_MAX = 480   #Valor maximo de PWM aceito pelo atuador
PWM_MIN = 295   #Valor minimo de PWM aceito pelo atuador
HIST_RE = 348   #Valor minimo da zona de histerese do atuador
HIST_FR = 382   #Valor maximo da zona de histerese do atuador

#####Constantes ROS#############
rospy.init_node('controller', anonymous=True)
pub_torque = rospy.Publisher('torque', Float64, queue_size = 1)

##### Variaveis ###########
ref = 0.0              # m
torque = 0.0           # N.m
torque_anterior = 0.0  # N.m
pwm = 0
start = 0.0


######## Relacao PWM x Torque ###############
### Sentido Re
def torquetopwmRe(T):
    valor = 353.6855 - 10.8643*abs(T)
    if (valor <= PWM_MIN):
        valor = PWM_MIN
    elif(valor >= HIST_RE):
        valor = 348

    return int(valor)
## Sentido Frente
def torquetopwm(T):
    valor =  379.0193 + 4.2135*abs(T)
    if (valor >= PWM_MAX):
        valor = PWM_MAX
    elif(valor <= HIST_FR):
        valor = 379
    return int(valor)

########### Calcula sinal de controle e envia comando ao ESC ##########
def set_PID(msg):
	global ref
	global torque_anterior
	global start
	
	position = round(msg.pos,3)
	torque = CONTROLADOR.update(ref, position)
	if (torque<0):
		# Checa a necessidade de mudar a marcha para sentido Re
		if(torque_anterior < 0):
			torque_anterior = torque
			pwm = torquetopwmRe(torque)
		else:
			torque_anterior = torque
			pwm = torquetopwmRe(torque)
			ATUADOR.set_pwm(0, 0, 350)
			time.sleep(0.08)
			ATUADOR.set_pwm(0, 0, 377)
			time.sleep(0.08)
			ATUADOR.set_pwm(0, 0, 350)
	else:
		torque_anterior = torque
		pwm = torquetopwm(torque)
	# Envia os comandos para o atuador     
	ATUADOR.set_pwm(0, 0, pwm)
	pub_torque.publish(torque)
	#print round(rospy.get_rostime().to_sec()-start,4)
	#data = str(round(rospy.get_rostime().to_sec()-start,4)) + "\t" + str(position) + "\t" + str(round(ref,3)) + "\t"
	#data += str(pwm) + "\t" + str(torque) + "\t" + str(CONTROLADOR.getError())
	#print data	


########### Calcula sinal de controle e envia comando ao ESC ##########
def trajectory(msg):
	global ref	
	ref = round(msg.traj,3)


##############################################
############ MAIN 
##############################################

######## Ler velocidade e posicao da porta serial arduino ########
if __name__ == '__main__':
	try:
		#while not rospy.is_shutdown():
		start = rospy.get_rostime().to_sec()
		rospy.Subscriber('trajectory_point', Trajectory, trajectory, queue_size=1)
		rospy.Subscriber('encoder_data', Encoder, set_PID, queue_size=1)
		rospy.spin()		
		pwm = 0
		ATUADOR.set_pwm(0, 0, pwm)
	except rospy.ROSInterruptException:
		pwm = 0
		ATUADOR.set_pwm(0, 0, pwm)
		pass
	
	

