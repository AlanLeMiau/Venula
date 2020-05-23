# **************************************************
# *	\    ^ ^ 	Project:		Venula
# *	 )  (="=)	Program name:	twistCam.py
# *	(  /   ) 	Author:			Alan Fuentes
# *	 \(__)|| 	https://github.com/AlanLeMiau
# * This program take the orientation of two marks
# * One mark blue and one green, and move a servo
# * trought serial port using an arduino to control
# * the servomotor
# **************************************************

import cv2
import time
import serial
import numpy as np
import cameraProcessing as camProc
#from serial import *
#from serial import Serial

def scan(num_ports = 10, verbose = True):
    dispositivos_serie = []
    if verbose:
    	print("Escaneando " + str(num_ports) + " puertos serie:")
        
    for i in range(num_ports):
        mssg = "puerto %d: " %i
        try:
            s = serial.Serial("COM"+str(i),57600) #57600
            time.sleep(0.5)
            mssg = mssg + "\n OK --> %s" %s.portstr 
            dispositivos_serie.append(s.portstr)
            s.close()
        except:
            mssg = mssg + "\n NO"
        
        if verbose:
        	print(mssg)

    return dispositivos_serie

def main():
	b = 90
	verde_bajos = np.array([49,50,50], dtype=np.uint8) 
	verde_altos = np.array([80, 255, 255], dtype=np.uint8)        
	azul_bajos  = np.array([100,100,20],dtype=np.uint8)
	azul_altos  = np.array([125,255,255],dtype=np.uint8)

	cap = cv2.VideoCapture(0)
	try:
		port = serial.Serial("COM3",57600)#, timeout=0, writeTimeout=0)
		time.sleep(2)
		print("Conected")    		
		while(1):
			ret,frame = cap.read()
			frame = cv2.flip(frame, -1)
			(height, width, layers) = frame.shape
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			
			blobG = cv2.inRange(hsv, verde_bajos, verde_altos)
			blob_Green = camProc.cleanImage(blobG)

			blobB = cv2.inRange(hsv, azul_bajos, azul_altos)
			blob_Blue = camProc.cleanImage(blobB)

			(xg, yg, frame) = camProc.contours(blob_Green, frame, (0,255,0))
			(xb, yb, frame) = camProc.contours(blob_Blue, frame,  (255,0,0))
			# print(xb,yb,xg,yg)
			angle = camProc.getAngle(xg, yg, xb, yb)
			angle = int(angle)
			# print (angle)		
			cv2.putText(frame,str(angle),(int(width/2),int(height/2)),cv2.FONT_HERSHEY_SIMPLEX,1,(100,255,255),2)
			cv2.imshow('frame',frame)
			
			b = b + angle
			if (b<0):
				b = 0
			if (b>180):
				b = 180

			print("Sending data ... " + str(b))
			port.write([b])
			port.write(b'.')

			if (cv2.waitKey(1000) == 27): # 27 = ESC
				break

		cv2.destroyAllWindows()
		cap.release()
		port.close()
		print("\n Program finished")
			

	except (KeyboardInterrupt, SystemExit):
		cv2.destroyAllWindows()
		cap.release()
		port.close()
		print("\n Program aborted")

if __name__ == '__main__':
	if scan():
		main()
	else:
		print("Any device conected")

