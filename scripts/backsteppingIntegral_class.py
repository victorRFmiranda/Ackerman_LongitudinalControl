from __future__ import division
import numpy as np
import math
from longitudinal_control.msg import Trajectory

##### Classe Controlador ######

####### Calcula a derivada ##########
def derivate(self,x0,x1):
	return ((x1-x0)/self.Ts)
class CONTROL:
	def __init__(self, k, c1, c2, c3, Tz, etol):
		self.K = k
		self.C1 = c1
		self.C2 = c2
		self.C3 = c3
		self.Ts = Tz
		
		self.setpoint = 0.0		 # setpoint
		self.d_setpoint = 0.0    # derivada primeira do setpoint
		self.d2_setpoint = 0.0   # derivada segunda do setpoint
		self.d3_setpoint = 0.0   # derivada terceira do setpoint
		
		self.error = 0.0
		self.int_erro = 0.0		 # Integral do erro
		self.errorant = 0.0 	 # Error anterior
		self.errortolerance = etol
		
		self.ref_anterior = 0.0
		self.alpha2_ant = 0.0
		
		self.u_filtrado_ant = 0.0
		
		#### Parametros
		self.m = 5.5						# massa (kg)
		self.g=9.81							# acel gravidade (m/s^2)
		self.inclination = 0.0				# inclinacao da pista em graus
		rho=1.1839							#
		Cd=1.05								#
		Af=0.096								#
		self.Fa = (1/2*self.m)*rho*Cd*Af	#
		self.r=0.08							#
		self.n=0.95							#
		self.tal=0.1						#
		### Parametros atrito
		#B = 10
		#C = 1.9
		#D = 1
		#E = 0.97
		#k = 0.02
		#self.H = D*math.sin(C*math.atan((B*k-E*(B*k-math.atan(B*k)))))  # coeficiente atrito
		self.H = 1.2
	
####### Atualiza a variavel de controle ########
	def Update(self, ref_atual, ref_anterior , distance, velocity, torque, aceleration):
		#### Recebe o valor de referencia para deslocamento e calcula suas derivadas
		self.setpoint = ref_atual
		dsp = self.d_setpoint    # salva o valor anterior da derivada da referencia
		self.d_setpoint = derivate(self, ref_anterior, ref_atual)  # calcula a nova derivada
		d2sp = self.d2_setpoint  # salva o valor anterior da derivada segunda da referencia
		self.d2_setpoint = derivate(self, dsp, self.d_setpoint)	 # calcaulda a nova derivada segunda
		self.d3_setpoint = derivate(self, d2sp, self.d2_setpoint)	 # calcaulda a nova derivada terceira
		
		### Erro
		self.error = self.setpoint - distance
		### Analisando tolerancia do erro
		if (abs(self.error) < self.errortolerance):
			self.error = 0
        ### Integral erro
		self.int_erro = self.int_erro + ((self.errorant + self.error)/2)*self.Ts
		self.errorant = self.error
		
		# Calculando valores auxiliares ao sinal de controle
		signal = (1.0+math.exp(-velocity))/(1+math.exp(-velocity))
		z1 = self.error
		alpha1 = self.K*self.int_erro + self.d_setpoint + self.C1*z1		# controle virtual 1
		z2 = velocity - alpha1
		# Controle virtual 2		
		alpha2 = (self.m*self.r/self.n)*(self.Fa*abs(velocity)*velocity + self.g*math.sin(self.inclination) + self.H*self.g*math.cos(self.inclination)*signal + self.K*z1 + self.d2_setpoint - self.C1*z2 - self.C1*self.K*self.int_erro - self.C1**2*z1 + z1 - self.C2*z2)
		z3 = torque - alpha2
		# Derivado do controle virtual 2
		d_alpha2 = derivate(self,self.alpha2_ant,alpha2)	
		self.alpha2_ant = alpha2	
		# Lei de controle auxiliar
		un = d_alpha2 - (self.n/(self.m*self.r))*z2 - self.C3*z3		
		# Calcula o sinal de controle
		u_control = self.tal*un+torque
		
		beta = 0.65
		u_filtrado = beta*self.u_filtrado_ant + (1-beta)*u_control
		self.u_filtrado_ant = u_filtrado
		
		return u_filtrado
		#return u_control
