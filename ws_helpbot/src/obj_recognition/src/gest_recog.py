#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import mediapipe as mp
from std_msgs.msg import String

# Variables globales como contadores para determinar el color del objeto buscado
# segun el numero de dedos levantados:
counter_r = 0
counter_g = 0
counter_b = 0
counter_y = 0
 
def callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Procesa la imagen RGB para localizar las manos e identificar los landmarks
    results = hands.process(frame)
    multiLandMarks = results.multi_hand_landmarks

    # Si detecta multiples landmarks significa que hay una mano
    if multiLandMarks:
        # DIBUJO DE LOS LANDMARKS EN LA IMAGEN ---------------------------------
        handList = []
        for handLms in multiLandMarks:
            mpDraw.draw_landmarks(frame, handLms, mp_Hands.HAND_CONNECTIONS)
            for idx, lm in enumerate(handLms.landmark):
              h, w, c = frame.shape
              cx, cy = int(lm.x * w), int(lm.y * h)
              handList.append((cx, cy))
        
        for point in handList:
            cv2.circle(frame, point, 10, (255, 255, 0), cv2.FILLED)
        # ----------------------------------------------------------------------

        # PARA MOSTRAR LA CUENTA DE NUMEROS LEVANTADOS EN LA IMAGEN ...........................
        upCount = 0
        for coordinate in finger_Coord:
            if handList[coordinate[0]][1] < handList[coordinate[1]][1]:
                upCount += 1
        if handList[thumb_Coord[0]][0] > handList[thumb_Coord[1]][0]:
            upCount += 1
        cv2.putText(frame, str(upCount), (150,150), cv2.FONT_HERSHEY_PLAIN, 12, (0,255,0), 12)
        #......................................................................................

        # En cada uno de los casos, se ha esperado a que se reciban 50 muestras de cada uno de los
        # posibles casos, para que no publique automaticamente y no se pueda reaccionar al cambio
        
        # 1 DEDO LEVANTADO = ROJO
        if upCount == 1:
            global counter_r
            counter_r = counter_r + 1
            if counter_r == 50:
                pub.publish("Rojo")
                counter_r = 0
        
        # 2 DEDOS LEVANTADOS = VERDE
        elif upCount == 2:
            global counter_g
            counter_g = counter_g + 1
            if counter_g == 50:
                pub.publish("Verde")
                counter_g = 0
        
        # 3 DEDOS LEVANTADOS = AZUL
        elif upCount == 3:
            global counter_b
            counter_b = counter_b + 1
            if counter_b == 50:
                pub.publish("Azul")
                counter_b = 0
        
        # 4 DEDOS LEVANTADOS = AMARILLO
        elif upCount == 4:
            global counter_y
            counter_y = counter_y + 1
            if counter_y == 50:
                pub.publish("Amarillo")
                counter_y = 0
        
        # 0 O 5 DEDOS LEVANTADOS
        else:
            pub.publish("Unknown")

    cv2.imshow("Counting number of fingers", frame)
    cv2.waitKey(1)



if __name__ == "__main__":
    # Declaracion de los objetos MediaPipe necesarios
    mp_Hands = mp.solutions.hands # Deteccion de manos en la imagen de entrada
    hands = mp_Hands.Hands() # Procesamiento de las manos detectadas
    mpDraw = mp.solutions.drawing_utils # Dibujo de las conexiones de las manos y landmarks

    # Declaracion de las coordenadas del indice y el pulgar para determinar si un dedo esta levantado o bajado
    finger_Coord = [(8, 6), (12, 10), (16, 14), (20, 18)]
    thumb_Coord = (4,2)

    bridge = CvBridge()

    rospy.init_node('gest_detection')
    rospy.Subscriber('/camera/rgb/image_raw', Image, callback)
    pub = rospy.Publisher('/colour_obj', String, queue_size = 10)
    rospy.spin()
