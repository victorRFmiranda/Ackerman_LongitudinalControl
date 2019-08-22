from __future__ import division
from scipy import signal

##### Classe PID ######
class PID:
    def __init__(self, P, I, D, Tz, etol):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Ts = Tz
        self.Integrator_min = -2.0 
        self.Integrator_max = 2.0
        self.Derivator = 0.0
        self.Integrator = 0.0

	self.alpha = 0.55
	self.control = 0.0

        self.set_point = 0.0
        self.error = 0.0
        self.errorant = 0.0
        self.errortolerance = etol

#### Atualizar o valor de controle com base no sinal de entrada ####
    def update(self,reference,feedback):
        self.error = reference - feedback
        if (abs(self.error) < self.errortolerance):
            self.error = 0
        
        ### Proporcional ###
        self.proportional = self.Kp * self.error
        ### Derivativo #####
        self.derivative = self.Kd * (self.error - self.Derivator)/self.Ts
        self.Derivator = self.error  #erro anterior
        ### Integral #######
        self.Integrator = self.Integrator + ((self.errorant + self.error)/2)*self.Ts
        #### saturacao integrador
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.errorant = self.error
        self.integral = self.Integrator * self.Ki

        PID = self.proportional + self.integral + self.derivative
	#Control_signal = self.proportional + self.integral + self.derivative	
	
	# Filter	
	self.control = (1-self.alpha)*self.control + self.alpha*PID
		

        return self.control
    
####### Retorna o Erro ###########
    def getError(self):
        return self.error
####### Retornar o Integrador ####
    def getIntegrator(self):
        return self.Integrator
####### Retornar o Derivador #####
    def getDerivator(self):
        return self.Derivator
        
