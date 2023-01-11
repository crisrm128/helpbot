#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
#from kobuki_msgs.msg import Sound <-- PARA EL ROBOT REAL
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
import subprocess

# Abre un archivo en modo lectura
f = open('/home/cristina/ws_helpbot/src/helpbot/src/maps/poses.txt', 'r')

# Funcion para buscar la posicion del robot segun el id del aruco
def find_aruco_pose(id):
    global f
    # Coloca el puntero al principio del archivo
    f.seek(0)
    for line in f:
        if "Aruco " + str(id) + ":" in line:
            # Esta es la linea buscada
            break
    
    # Vector que almacena los parametros de la linea usando como separador el caracter espacio
    values = line.split(" ")
    values = values[2:]  # Elimina el identificador del aruco ("Aruco X:")

    pose = [float(x) for x in values] # Se convierten los valores a flotantes

    return pose

# Funcion para volver a la posicion inicial donde esta el usuario
def go_to_user_pose():
    pose_init = find_aruco_pose(10)
    # Se llama a la accion pasando como parametros de entrada la posicion deseada del robot
    process = subprocess.Popen(["rosrun", "helpbot", "test_movebase.py", str(pose_init[0]), str(pose_init[1]), str(pose_init[2]), \
       str(pose_init[3]), str(pose_init[4]), str(pose_init[5]), str(pose_init[6])])
    # Se espera hasta que termine el proceso de llamada
    process.wait()


def callback(goal_colour):
    #sound = Sound() <-- PARA EL ROBOT REAL
    # Bucle para recorrer las 4 posibles posiciones (correspondientes a los 4 arucos y objetos)
    for i in range(1, 5):
        pose_act = find_aruco_pose(i) # Se obtiene la posicion a la que debe ir el robot
        
        # Se llama a la accion pasando como parametros de entrada la posicion deseada del robot
        process = subprocess.Popen(["rosrun", "helpbot", "test_movebase.py", str(pose_act[0]), str(pose_act[1]), str(pose_act[2]), \
        str(pose_act[3]), str(pose_act[4]), str(pose_act[5]), str(pose_act[6])])
        # Se espera hasta que termine el proceso de llamada
        process.wait()

        # Prueba a ver si se recibe un objeto detectado, con tiempo maximo de 5 segundos
        try:
            colour_msg = rospy.wait_for_message("/obj_detected", String, timeout=5)
        except rospy.exceptions.ROSException:
            # No se recibió un mensaje en el tiempo especificado
            pass
        else:
            # Si el color del objeto visto es el mismo que el objeto deseado
            if colour_msg.data == goal_colour.data:
                #sound.value = Sound.ON <-- PARA EL ROBOT REAL
                #sound_pub.publish(sound) <-- PARA EL ROBOT REAL
                print('¡Conseguido!')
                break # Termina el bucle de busqueda entre las posibles posiciones

    print('Volver a la posicion inicial')
    go_to_user_pose()


if __name__ == "__main__":
    bridge = CvBridge()
    rospy.init_node('navigation_helpbot')
    # Antes de nada, va a la posicion incicial donde esta el usuario
    go_to_user_pose()
    rospy.Subscriber('/colour_obj', String, callback) 
    #sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10) <-- PARA EL ROBOT REAL
    rospy.spin()