#!/usr/bin/env python

import Adafruit_PCA9685
import time
import rospy
from std_msgs.msg import Float32, String


######## Constantes ###########
ATUADOR = Adafruit_PCA9685.PCA9685()    #chamada da classe da placa de comunicao com atuador
ATUADOR.set_pwm_freq(60)                #Frequencia do atuador
PWM_MAX = 480   #Valor maximo de PWM aceito pelo atuador
PWM_MIN = 295   #Valor minimo de PWM aceito pelo atuador
HIST_RE = 348   #Valor minimo da zona de histerese do atuador
HIST_FR = 382   #Valor maximo da zona de histerese do atuador
TS = 0.16        ## Periodo de amostragem dos dados

#####Constantes ROS#############
rospy.init_node('atuador', anonymous=True)
rate = rospy.Rate(10)
pub_data = rospy.Publisher('data', String, queue_size=1)

##### Variaveis ###########
torque = 0           # N.m
torque_anterior = 0  # N.m
pwm = 0

######## Relacao PWM x Torque ###############
### Sentido Re
def torquetopwmRe(T):
    valor = 353.6855 - 10.8643*abs(T)
    if (valor <= PWM_MIN):
        valor = PWM_MIN
    elif(valor >= HIST_RE):
        valor = 352
    return int(valor)
## Sentido Frente
def torquetopwm(T):
    valor =  379.0193 + 4.2135*abs(T)
    if (valor >= PWM_MAX):
        valor = PWM_MAX
    elif(valor <= HIST_FR):
        valor = 379
    return int(valor)


########## Enviar comandos para o ESC ################
def comando_esc(msg):
	global torque_anterior
	global pwm
	global start
	torque = msg.data
	### Checa o sentido do torque
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
	#data = str(round(rospy.get_rostime().to_sec()-start,2)) + "\t" + str(posicao) + "\t" + str(round(ref,3)) + "\t"
	#data += str(pwm) + "\t" + str(torque) 
	#pub_data.publish(data)
	#rate.sleep()
		

##############################################
############ MAIN 
##############################################
start = rospy.get_rostime().to_sec()
if __name__ == '__main__':
	try:
		rospy.Subscriber('torque', Float32, comando_esc, , queue_size=1)
		rospy.spin()
		pwm = 0
		ATUADOR.set_pwm(0, 0, pwm)
	except rospy.ROSInterruptException:
		pwm = 0
		ATUADOR.set_pwm(0, 0, pwm)		
		pass
