#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String
from backstepping_class import CONTROL
from longitudinal_control.msg import Trajectory
from longitudinal_control.msg import Encoder
import Adafruit_PCA9685
import time

######## Constantes ###########
#ANDROID = Android()     #Comunicacao com o dispositivo android
K1 = 2
K2 = 2
K3 = 2
Ts = 0.16       ## Periodo de amostragem dos dados
#Etol = 0.07     ## Tolerancia do erro do controlador
CONTROLADOR = CONTROL(K1,K2,K3,Ts)
ATUADOR = Adafruit_PCA9685.PCA9685()
ATUADOR.set_pwm_freq(60)
PWM_MAX = 480   #Valor maximo de PWM aceito pelo atuador
PWM_MIN = 290   #Valor minimo de PWM aceito pelo atuador
HIST_RE = 348   #Valor minimo da zona de histerese do atuador
HIST_FR = 385  #Valor maximo da zona de histerese do atuador

#####Constantes ROS#############
rospy.init_node('controller', anonymous=True)
pub_torque = rospy.Publisher('torque', Float64, queue_size = 1)

##### Variaveis ###########
pos_encoder = 0.0      # m
velo_encoder = 0.0     # m/s
torque = 0.0           # N.m
acel = 0.0             # m/s2
velo_anterior = 0.0
torque_anterior = 0.0
pwm = 0
ref = 0.0         	# m
ref_anterior = 0.0	# m


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



def set_Control(msg):
	global ref
	global ref_anterior
	global torque_anterior
	global velo_anterior
	global acel
	global Ts
	position = round(msg.pos,3)
	velo_encoder = round(msg.velocity, 3)
	#### Derivar a velocidade para teste em laboratorio, depois usar acelerometro ######
	acel = ((velo_encoder-velo_anterior)/Ts)
	velo_anterior = velo_encoder
	torque = CONTROLADOR.Update(ref, ref_anterior, position, float(velo_encoder),torque_anterior,acel)
	ref_anterior = ref
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
	pub_torque.publish(torque)




########### Calcula sinal de controle e envia comando ao ESC ##########
def trajectory(msg):
	global ref	
	ref = round(msg.traj,3)



############ MAIN ############################

######## Ler velocidade e posicao da porta serial arduino ########
if __name__ == '__main__':
	try:
		#while not rospy.is_shutdown():
		start = rospy.get_rostime().to_sec()
		rospy.Subscriber('trajectory_point', Trajectory, trajectory, queue_size=1)
		rospy.Subscriber('encoder_data', Encoder, set_Control, queue_size=1)
		rospy.spin()		
		pwm = 0
		ATUADOR.set_pwm(0, 0, pwm)
	except rospy.ROSInterruptException:
		pwm = 0
		ATUADOR.set_pwm(0, 0, pwm)
		pass
