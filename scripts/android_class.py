#!/usr/bin/env python
# -*- coding: utf-8 -*-
#############################################################################
# Driver para ler dados do celular Android
# Nome: Armando Alves Neto
# DELT
#############################################################################
import os, sys, time, math
import telnetlib

#############################################################################
# Global definitions
#############################################################################
# definicoes para comunicacao via rede (ADB/USB)
HOST = "localhost"
PORT = 8080
ADB_COMMAND = "adb forward tcp:" + str(PORT) + " tcp:" + str(PORT)

DATA_COMMAND = "A5S"
EXIT_COMMAND = "A5X"

# gravity
G0 = 0.978
# mean of magnetometers mesuares in normal conditions
MX_MEAN = -21.875#18.8750
MY_MEAN = -11.3750
MZ_MEAN = 39.8750

#############################################################################
# class Android
#############################################################################
class Android:
	#########################################################################
	# construtor
	#########################################################################
	def __init__(self):
	
		# Lembre-se de executar o comando: "adb forward tcp:PORT tcp:PORT"
		os.system(ADB_COMMAND)
		
		# cria canal de comunicacao
		try:
			self.tn = telnetlib.Telnet(HOST, PORT)
		except:
			print "Android not found!!!"
		
		# imu
		self.ax = self.ay = self.az = 0.0
		self.gx = self.gy = self.gz = 0.0 # nao implementado
		self.mx = self.my = self.mz = 0.0
		# gps
		self.lat = self.lon = self.vel = 0.0
		self.x = self.y = 0.0
		self.sat = 0
		self.utmzone = 0
		# time
		self.t = 0
		
		# angulos internos [rad]
		self.phi = 0.0
		self.theta = 0.0
		self.psi = 0.0

		# Filter Butterworth
		self.b = 0.1367
		self.a = -0.7265
		self.ax_filter = 0.0
		self.ay_filter = 0.0
		self.az_filter = 0.0
		
	#########################################################################
	# Requsita dados do celular via USB
	#########################################################################
	def getAndroidData(self):
		
		# requsita dados
		self.tn.write(DATA_COMMAND)
		
		# le dados com 1 segundo de espera
		try:
			msg = self.tn.read_until("\n", 1)
			if not msg:
				return False
		except:
			print "Android not found!!!"
			sys.exit(0)
		
		# parser dos dados
		data = msg.split()
		#
		self.ax = float(data[1])
		self.ay = float(data[2])
		self.az = float(data[3])
		#
		self.mx = float(data[4]) - MX_MEAN
		self.my = float(data[5]) - MY_MEAN
		self.mz = float(data[6]) - MZ_MEAN
		#
		self.lat = float(data[7])
		self.lon = float(data[8])
		self.vel = float(data[9])
		self.sat = int(data[10])
		#
		self.t  = int(data[11])
		
		# converte (lat,lon) em formato digital para metros
		self.x, self.y, self.utmzone = self.wgs2utm(self.lat, self.lon)
		
		return True
		
	#########################################################################
	# Accelerometer data [m/s^2]
	#########################################################################
	def getAccelData(self):
		self.ax_filter = self.b*self.ax - self.a*self.ax_filter
		self.ay_filter = self.b*self.ay - self.a*self.ay_filter
		self.az_filter = self.b*self.az - self.a*self.az_filter
		#return self.ax, self.ay, self.az
		return self.ax_filter, self.ay_filter, self.az_filter
	
	#########################################################################
	# Compass data
	#########################################################################
	def getCompassData(self):
		return self.mx, self.my, self.mz
		
	#########################################################################
	# Gyro data
	#########################################################################
	def getGyroData(self):
		return self.gx, self.gy, self.gz
		
	#########################################################################
	# GPS data
	#########################################################################
	def getGPSData(self):
		return self.lat, self.lon, self.vel#, self.sat

	#########################################################################
	# GPS Pos metros
	#########################################################################
	def getGPSPos(self):
		return self.x, self.y
		
	#########################################################################
	# Digital format (dddd.mmm) 2 meters(m)
	#########################################################################
	def wgs2utm(self, Lat, Lon):
	
		# coordinates in radians
		lat = Lat*(math.pi/180.0)
		lon = Lon*(math.pi/180.0)
		
		# WGS84 parameters
		a = 6378137 			#semi-major axis
		b = 6356752.314245 		#semi-minor axis
		e = math.sqrt(1 - pow(b/a, 2)) 	#eccentricity
		e2 = pow(e, 2)
		e4 = pow(e, 4)
		e6 = pow(e, 6)
		
		# UTM parameters
		# lat0 = 0 						#reference latitude, not used here
		Lon0 = math.floor(Lon/6)*6 + 3 	#reference longitude in degrees
		lon0 = Lon0*(math.pi/180.0)		#in radians
		k0 = 0.9996 					#scale on central meridian
		
		FE = 500000 			#false easting
		FN = (Lat < 0)*10000000 #false northing
		
		# Equations parameters
		eps = e2/(1 - e2) # e prime square
		# N: radius of curvature of the earth perpendicular to meridian plane
		# Also, distance from point to polar axis
		N = a/math.sqrt(1 - e2*pow(math.sin(lat),2))
		T = pow(math.tan(lat),2)
		C = ((e2)/(1-e2))*pow(math.cos(lat), 2)
		A = (lon-lon0)*math.cos(lat)
		
		# M: true distance along the central meridian from the equator to lat
		M = a*( +(1- e2/4 - 3*e4/64 - 5*e6/256 )*lat
				-(3*e2/8 + 3*e4/32 + 45*e6/1024 )*math.sin(2*lat)
				+(15*e4/256 + 45*e6/1024 )*math.sin(4*lat)
				-(35*e6/3072 )*math.sin(6*lat) )
				
		# easting
		x = FE + k0*N*( A+(1-T+C)*pow(A,3)/6 + ( 5-18*T+pow(T,2)+72*C-58*eps )*pow(A,5)/120 )
		
		# northing
		# M(lat0) = 0 so not used in following formula
		y = FN + k0*M + k0*N*math.tan(lat)*((pow(A,2)/2) 
									+ (5 - T + 9*C + 4*pow(C,2))*(pow(A,4)/24) 
									+ (61 - 58*T + pow(T,2) + 600*C - 330*eps)*pow(A,6)/720)
		
		# UTM zone
		utmzone = int(math.floor(Lon0/6) + 31)
		
		return round(x,4), round(y,4), utmzone
		
	#########################################################################
	# Time data
	#########################################################################
	def getTime(self):
		return self.t
		
	#########################################################################
	# Roll, Pitch and Yaw data [rad] (filtered or not)
	#########################################################################
	def getAngles(self):
	
		# get accelerometers
		ax, ay, az = self.getAccelData()
		# get magnetometers
		mx, my, mz = self.getCompassData()
		
		# norma do vetor gravidade medido (evita problemas no calculo do asin())
		anorm = math.sqrt(pow(ax,2) + pow(ay,2) + pow(az,2))
		# robo esta caindo :(
		if anorm == 0.0:
			anorm = G0
		
		# pitch
		theta = math.asin(ax/anorm)
		# roll
		try:
			phi = math.asin(ay/(-anorm*math.cos(theta)))
		except:
			phi = math.copysign(math.pi, ay)
		# yaw
		mag_x = mx*math.cos(theta) + my*math.sin(phi)*math.sin(theta) + mz*math.cos(phi)*math.sin(theta)
		mag_y = my*math.cos(phi) - mz*math.sin(phi)
		#mag_x = mx
		#mag_y = my
		psi = math.atan2(-mag_y, mag_x)
		# garante yaw entre [0...2pi]
		psi = psi % (2.0*math.pi)

		#filter
		#psi = self.b*psi - self.a*self.phi
		
		# seta angulos internos
		self.phi = phi
		self.theta = theta
		self.psi = psi
			
		return phi, theta, psi-self.deg2rad(270)
		
	#########################################################################
	def rad2deg(self, radians):
		degrees = 180.0 * radians / math.pi
		return degrees
	#########################################################################
	def deg2rad(self, degrees):
		radians = math.pi * degrees / 180.0
		return radians
		
	#########################################################################
	# Show GPS data
	#########################################################################
	def gpsShow(self):
		print '-------------------------------------------------------------'
		print '<GPS data>'
		
		'''aux = '\tQuality: '
		
		# Print GPS fix quality
		if self._fix_quality == 0:
			print aux + redprint('GPS invalid')
		elif self._fix_quality == 1:
			print aux + greenprint('GPS fix (SPS)')
		elif self._fix_quality == 2:
			print aux + greenprint('DGPS fix')
		elif self._fix_quality == 3:
			print aux + 'PPS fix'
		elif self._fix_quality == 4:
			print aux + greenprint('Real Time Kinematic')
		elif self._fix_quality == 5:
			print aux + 'Float RTK'
		elif self._fix_quality == 6:
			print aux + 'Estimated (dead reckoning)'
		elif self._fix_quality == 7:
			print aux + 'Manual input mode'
		elif self._fix_quality == 8:
			print aux + 'Simulation mode'
		else:
			print aux + redprint('UNDEFINED MODE')

		aux = '\t3D fix: '
		# Print GPS 3D FIX
		if self._fix3d == 1:
			print aux + redprint('no fix')
		elif self._fix3d == 2:
			print aux + greenprint('2D fix')
		elif self._fix3d == 3:
			print aux + greenprint('3D fix')
		else:
			print redprint('UNDEFINED FIX')
				
		# Print status
		aux = '\tStatus: '
		if self._status == 'A':
			print aux + greenprint('Active')
		elif self._status == 'V':
			print aux + redprint('Void')
		else:
			print redprint('UNDEFINED STATUS')'''
		
		print 'No sats:', self.sat
		print 'Latitude: ', self.lat
		print 'Longitude:', self.lon
		print 'X[m]: ', self.x, '\tUTM zone ', + self.utmzone
		print 'Y[m]: ', self.y
		print 'Speed:', self.vel
		#print 'Orientation:', self._orientation
		print '-------------------------------------------------------------'
		
	#########################################################################
	# destrutor
	#########################################################################
	def __del__(self):
		# encerra aplicativo
		self.tn.write(EXIT_COMMAND)
		# fecha conexao telnet
		self.tn.close()

#############################################################################
# main
#############################################################################

