from __future__ import division
import numpy as np
import math
from longitudinal_control.msg import Trajectory

##### Classe Controlador ######

####### Calcula a derivada ##########
def derivate(self,x0,x1):
	return ((x1-x0)/self.Ts)
class CONTROL:
	def __init__(self, k1, k2, k3, Tz):
		self.K1 = k1
		self.K2 = k2
		self.K3 = k3
		self.Ts = Tz
		
		self.setpoint = 0.0		 # setpoint
		self.d_setpoint = 0.0    # derivada primeira do setpoint
		self.d2_setpoint = 0.0   # derivada segunda do setpoint
		self.d3_setpoint = 0.0   # derivada terceira do setpoint

		self.alpha2_ant = 0.0
		
		self.error = 0.0
		
		#### Parametros
		self.m = 5.5						# massa (kg)
		self.g=9.81							# acel gravidade (m/s^2)
		self.inclination = 0.0				# inclinacao da pista em graus
		rho=1.1839							#
		Cd=1.05								#
		Af=0.15								#
		self.Fa = (1/2*self.m)*rho*Cd*Af	#
		self.r=0.08							#
		self.n=0.95							#
		self.tal=0.1						#
		### Parametros atrito
		B = 10
		C = 1.9
		D = 1
		E = 0.97
		k = 0.02
		self.H = D*math.sin(C*math.atan((B*k-E*(B*k-math.atan(B*k)))))  # coeficiente atrito

	
####### Atualiza a variavel de controle ########
	def Update(self, ref_atual, ref_anterior , distance, velocity, torque, aceleration):
		#### Recebe o valor de referencia para deslocamento e calcula suas derivadas
		self.setpoint = ref_atual
		dsp = self.d_setpoint    # salva o valor anterior da derivada da referencia
		self.d_setpoint = derivate(self, ref_anterior, ref_atual)  # calcula a nova derivada
		d2sp = self.d2_setpoint  # salva o valor anterior da derivada segunda da referencia
		self.d2_setpoint = derivate(self, dsp, self.d_setpoint)	 # calcaulda a nova derivada segunda
		self.d3_setpoint = derivate(self, d2sp, self.d2_setpoint)	 # calcaulda a nova derivada terceira
		
		# Calculando valores auxiliares ao sinal de controle
		self.error = self.setpoint - distance
		signal = (1.0+math.exp(-velocity))/(1+math.exp(-velocity))
		# controle virtual 1
		alpha1 = self.d_setpoint + self.K1*self.error
		z1=velocity-alpha1
		# controle virtual 2
		alpha2 = (self.m*self.r/self.n)*(self.Fa*abs(velocity)*velocity+self.g*math.sin(self.inclination)+self.d2_setpoint-self.K1*z1+self.H*self.g*math.cos(self.inclination)*signal-self.error*self.K1**2+self.error-self.K2*z1)
		z2=torque-alpha2
		# Derivado do controle virtual 2
		d_alpha2 = derivate(self,self.alpha2_ant,alpha2)	
		self.alpha2_ant = alpha2	
		# Lei de controle auxiliar
		un = d_alpha2 - (self.n/(self.m*self.r))*z1 - self.K3*z2		
		# Calcula o sinal de controle
		u_control = self.tal*un+torque
		return u_control
