#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, LaserScan
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

OBJ_COLOR = ""

# Para calcular el contorno de area maxima que se obtiene en la imagen
def max_contour_area(frame, into_hsv, lower_limit, upper_limit):
    # Creacion de la mascara usando la funcion inRange().
    # Esto producira una imagen donde el color de los objetos
    # que caen dentro de este rango se volveran blancos, mientras
    # que el resto seran negros
    mask=cv2.inRange(into_hsv,lower_limit,upper_limit)

    # Esto le dara color a la mascara (los objetos blancos se volveran del color real).
    colour_mask=cv2.bitwise_and(frame,frame,mask=mask)

    # Procesamiento de la imagen para obtener los contornos
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    erosion = cv2.erode(opening,kernel,iterations = 3)
    contours, _ = cv2.findContours(erosion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    area = 0

    if len(contours) != 0:
        # Busca el contorno mas grande (c) segun el area
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        cv2.drawContours(colour_mask, c, -1, (128, 84, 231), 3)

    return area, colour_mask

# Para calcular el contorno de area maxima que se obtiene en la imagen (en el caso del color rojo se necesitan
# mezclar dos umbrales de HSV y por ello tiene una funcion especifica)
def max_contour_area_red(frame, into_hsv, Red_L_limit_1, Red_L_limit_2, Red_U_limit_1, Red_U_limit_2):
    # Creacion de la mascara usando la funcion inRange().
    # Esto producira una imagen donde el color de los objetos
    # que caen dentro de este rango se volveran blancos, mientras
    # que el resto seran negros
    r_lower_mask=cv2.inRange(into_hsv,Red_L_limit_1,Red_U_limit_1)
    r_upper_mask=cv2.inRange(into_hsv,Red_L_limit_2,Red_U_limit_2)
    r_full_mask = r_lower_mask + r_upper_mask

    # Esto le dara color a la mascara (los objetos blancos se volveran del color real).
    colour_mask=cv2.bitwise_and(frame,frame,mask=r_full_mask)

     # Procesamiento de la imagen para obtener los contornos
    kernel = np.ones((5,5),np.uint8)
    opening_r = cv2.morphologyEx(r_full_mask, cv2.MORPH_OPEN, kernel)
    erosion_r = cv2.erode(opening_r,kernel,iterations = 3)
    contours_r, _ = cv2.findContours(erosion_r, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    area = 0

    if len(contours_r) != 0:
        # Busca el contorno mas grande (c) segun el area
        c = max(contours_r, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
        area = cv2.contourArea(c)
        cv2.drawContours(colour_mask, c, -1, (128, 84, 231), 3)

    return area, colour_mask
 
def callback(image_info):

    try:
        frame = bridge.imgmsg_to_cv2(image_info, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow('Camara',frame)
    cv2.waitKey(10)

    global OBJ_COLOR

    # Cambiar el formato de color de BGR a HSV, que se usara para crear una mascara
    into_hsv =cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    

    if OBJ_COLOR == "Rojo":
        cv2.destroyWindow('Green Detector')
        cv2.destroyWindow('Blue Detector')
        cv2.destroyWindow('Yellow Detector')

        # Rango de valores inferior del color rojo:
        Red_L_limit_1 = np.array([0, 100, 20])
        Red_U_limit_1 = np.array([10, 255, 255])
        # Rango de valores superior del color rojo:
        Red_L_limit_2 = np.array([160,100,20])
        Red_U_limit_2 = np.array([179,255,255])

        area_r, red = max_contour_area_red(frame, into_hsv, Red_L_limit_1, Red_L_limit_2, Red_U_limit_1, Red_U_limit_2)

        if area_r > 4000:
            pub.publish("Rojo")

        cv2.imshow('Red Detector',red) # Para mostrar la imagen del objeto rojo detectado

    elif OBJ_COLOR == "Verde":
        cv2.destroyWindow('Red Detector')
        cv2.destroyWindow('Blue Detector')
        cv2.destroyWindow('Yellow Detector')
        
        Green_L_limit=np.array([40,0,0]) # Limite inferior del color verde
        Green_U_limit=np.array([86,255,255]) # Limite superior del color verde

        area_g, green = max_contour_area(frame, into_hsv, Green_L_limit, Green_U_limit)

        if area_g > 4000:
            pub.publish("Verde")

        cv2.imshow('Green Detector',green) # Para mostrar la imagen del objeto verde detectado

    elif OBJ_COLOR == "Azul":
        cv2.destroyWindow('Red Detector')
        cv2.destroyWindow('Green Detector')
        cv2.destroyWindow('Yellow Detector')
        
        Blue_L_limit=np.array([98,50,50]) # Limite inferior del color azul
        Blue_U_limit=np.array([139,255,255]) # Limite superior del color azul

        area_b, blue = max_contour_area(frame, into_hsv, Blue_L_limit, Blue_U_limit)

        if area_b > 4000:
            pub.publish("Azul")

        cv2.imshow('Blue Detector',blue) # Para mostrar la imagen del objeto azul detectado

    elif OBJ_COLOR == "Amarillo":
        cv2.destroyWindow('Red Detector')
        cv2.destroyWindow('Green Detector')
        cv2.destroyWindow('Blue Detector')
        
        Yellow_L_limit=np.array([20,100,100]) # Limite inferior del color amarillo
        Yellow_U_limit=np.array([30,255,255]) # Limite superior del color amarillo

        area_y, yellow = max_contour_area(frame, into_hsv, Yellow_L_limit, Yellow_U_limit)

        if area_y > 4000: # CAMBIAR DE 4000 A 500 (ROBOT REAL)
            pub.publish("Amarillo")

        cv2.imshow('Yellow Detector',yellow) # Para mostrar la imagen del objeto amarillo detectado


# Para guardar el color del objeto deseado
def callback_2(colour_info):
    global OBJ_COLOR
    OBJ_COLOR = colour_info.data


if __name__ == "__main__":
    bridge = CvBridge()
    rospy.init_node('obj_detection')
    rospy.Subscriber('/camera/rgb/image_raw', Image, callback)
    rospy.Subscriber('/colour_obj', String, callback_2)
    pub = rospy.Publisher('/obj_detected', String, queue_size = 10)
    rospy.spin()
