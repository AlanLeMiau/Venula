# -*- coding: utf-8 -*-
"""
@author: Mariana_
"""
import cv2
import numpy as np

def cleanImage(img):
	# Kernel para transformaciones morfologicas
	kernel = np.ones((5, 5)) 
	# Transformaciones morfológicas para eliminar el ruido que se obtiene en la imagen
	dilacion = cv2.dilate(img, kernel, iterations=1)
	erosion = cv2.erode(dilacion, kernel, iterations=1)
	# Filtro gaussiano y se aplica un umbral de otsu
	filtrado = cv2.GaussianBlur(erosion, (5, 5), 0)
	ret, thresh = cv2.threshold(filtrado, 127, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	return thresh	

def contours(blob,frame, textColor, verbose = True):
	# Detección de contornos
	contours, hierarchy = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	(x, y) = (0, 0)
	for c in contours:              #######PUNTO 1 VERDE
		area = cv2.contourArea(c)
		if area > 1000:
			M = cv2.moments(c)  #centroide
			if (M["m00"]==0):
				M["m00"] = 1
			x = int(M["m10"] / M["m00"])
			y = int(M["m01"] / M["m00"])
			if (verbose):
				font = cv2.FONT_HERSHEY_SIMPLEX
				cv2.circle(frame, (x,y), 7, textColor, -1)#dibuja circulo en el centroide
				cv2.putText(frame, '{},{}'.format(x,y),(x+10,y), font, 0.75,textColor,1,cv2.LINE_AA)
				nuevoContorno = cv2.convexHull(c)
				cv2.drawContours(frame, [nuevoContorno], 0, textColor, 3)
		else:
			continue

	return (x, y, frame)

def getAngle(xf, yf, xm, ym):
	dy = ym - yf
	dx = xm - xf
	angle = np.arctan2(dy,dx) * (180/np.pi)
	return angle

def main():
	verde_bajos = np.array([49,50,50], dtype=np.uint8) 
	verde_altos = np.array([80, 255, 255], dtype=np.uint8)        
	azul_bajos  = np.array([100,100,20],dtype=np.uint8)
	azul_altos  = np.array([125,255,255],dtype=np.uint8)

	cap = cv2.VideoCapture(0)
	
	while (1):
		ret,frame = cap.read()
		frame = cv2.flip(frame, -1)
		(height, width, layers) = frame.shape
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		
		blobG = cv2.inRange(hsv, verde_bajos, verde_altos)
		blob_Green = cleanImage(blobG)

		blobB = cv2.inRange(hsv, azul_bajos, azul_altos)
		blob_Blue = cleanImage(blobB)

		(xg, yg, frame) = contours(blob_Green, frame, (0,255,0))
		(xb, yb, frame) = contours(blob_Blue, frame,  (255,0,0))
		# print(xb,yb,xg,yg)
		angle = getAngle(xb, yb, xg, yg)
		angle = int(angle)
		# print (angle)		
		cv2.putText(frame,str(angle),(int(width/2),int(height/2)),cv2.FONT_HERSHEY_SIMPLEX,1,(100,255,255),2)
		cv2.imshow('frame',frame)
		if (cv2.waitKey(1) == 27):
			break

	cv2.destroyAllWindows()
	cap.release()
		
if __name__ == '__main__':
	main()
