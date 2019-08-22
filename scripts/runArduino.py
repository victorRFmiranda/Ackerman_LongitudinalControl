#!/usr/bin/env python
import os
import time
import rospy
import serial
import warnings
import serial.tools.list_ports

######## Buscando a porta do Arduino ##############
arduino_ports = [
    p.device
    for p in serial.tools.list_ports.comports()
    if 'USB2.0-Serial' in p.description
]
if not arduino_ports:
    raise IOError("No Arduino found")
if len(arduino_ports) > 1:
    warnings.warn('Multiple Arduinos found - using the first')

os.system("gnome-terminal -x bash -c 'source ~/ros_carrinho/devel/setup.bash; rosrun rosserial_python serial_node.py _port:="+str(arduino_ports[0])+" _baud:=57600'")
