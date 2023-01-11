#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
import subprocess

Aruco_markers_id = [10, 1, 2, 3, 4] # IDs de los arucos que se deben ver
Aruco_markers_seen = [0, 0, 0, 0, 0] # Para saber si los arucos han sido vistos o no
Pose_Aruco = [Pose() for _ in range(5)] # Para guardar las poses del robot en cada uno de los arucos

once = False
velocity_below_threshold = False

def check_velocity(velocity):
    # Para comprobar que la velocidad es nula, se mira si en todos los campos de velocidad hay un valor cercano a cero
    # (no es cero exacto porque existen casos donde las velocidades resultan numeros muy pequeños):
    if abs(velocity.linear.x) < 0.001 and abs(velocity.linear.y) < 0.001 and abs(velocity.linear.z) < 0.001 \
    and abs(velocity.angular.x) < 0.001 and abs(velocity.angular.y) < 0.001 and abs(velocity.angular.z) < 0.001:
        return True
    return False

def pose_estimation(ID):

    # Comprobacion de la version inicial pero que no afectaria
    if ID != 0:
        
        # Para cada uno de los posibles arucos que se pueden ver
        for i in range(len(Aruco_markers_id)):
            # Si el ID visto actualmente coincide con uno de los posibles arucos y todavia no se habia visto
            if ID == Aruco_markers_id[i] and bool(Aruco_markers_seen[i]) == False:
                print('¡He detectado un Aruco! Con ID: ' + str(ID))
                print('-----------------------')
                print('Sitúa el robot justo delante del Aruco y espera 3 segundos')
                print(' ')

                global velocity_below_threshold

                start_time = rospy.Time.now()

                while True:
                    # Espera a que se reciba un mensaje de velocidad en el topic correspondiente
                    #cmd_vel_msg = rospy.wait_for_message("/mobile_base/commands/velocity", Twist) <---- PARA EL ROBOT REAL
                    cmd_vel_msg = rospy.wait_for_message("/cmd_vel", Twist)

                    # Si la velocidad esta por debajo del umbral
                    if check_velocity(cmd_vel_msg) == True:
                        # Si el flag todavia no se ha activdado, se activa y se actualiza el tiempo de inicio
                        # de conteo
                        if not velocity_below_threshold:
                            velocity_below_threshold = True
                            start_time = rospy.Time.now()
                    else:
                        #Si la velocidad esta por encima del umbral, se resetea el flag y el tiempo de inicio
                        velocity_below_threshold = False
                        start_time = rospy.Time.now()

                    # Si la velocidad ha estado por debajo del umbral por 3 segundos o mas
                    if velocity_below_threshold and (rospy.Time.now() - start_time).to_sec() >= 3:
                        # Espera a que se reciba un mensaje de posicion del robot (odometria) en el topic correspondiente
                        odom_msg = rospy.wait_for_message("/odom", Odometry)
                        # Guarda el campo correspondiente a la posicion
                        pose = odom_msg.pose.pose

                        # Se almacena en el vector de posiciones, en el indice correspondiente al aruco visto
                        Pose_Aruco[i] = pose

                        # Cadena de texto de formato: "Aruco X: x y z qx qy qz qw"
                        pose_str = 'Aruco ' + str(Aruco_markers_id[i]) + ': ' + str(round(pose.position.x,2)) + ' ' + str(round(pose.position.y,2)) + ' ' + str(round(pose.position.z,2)) + ' ' + \
                        str(round(pose.orientation.x,2)) + ' ' + str(round(pose.orientation.y,2)) + ' ' + str(round(pose.orientation.z,2)) + ' ' + str(round(pose.orientation.w,2)) + '\n'

                        # Escribe la cadena en el fichero de texto anteriormente abierto
                        f.write(pose_str)

                        print('Posición guardada')

                        #Actualiza el aruco como ya visto o visitado
                        Aruco_markers_seen[i] = 1
                        
                        # Termina el bucle infinito y pasa al siguiente aruco
                        break
    

def callback(aruco_id):

    # Aruco que esta viendo ahora mismo el turtlebot
    ID = int(aruco_id.data) 

    # Estimar la posicion del robot para el aruco visto
    pose_estimation(ID)  

    global once
    finished = True # Se inicializa y comprueba cada vez que se vea un aruco

    # Mientras todavia no se hayan terminado de ver todos los arucos posibles
    for i in range(len(Aruco_markers_seen)):
        if bool(Aruco_markers_seen[i]) == False:
            finished = False
                
    # Si ya se han visto todos los arucos posibles
    if finished == True:
        if once == False: # Es la primera y unica vez que se entra en esta condicion
            print('¡Terminado!')
            once = True
            # Se guarda el mapa
            subprocess.call("rosrun map_server map_saver -f $HOME/ws_helpbot/src/helpbot/src/maps/mi_mapa", shell=True)
            # Se termina el proceso de mapeado con SLAM
            subprocess.call("rosnode kill /turtlebot3_slam_gmapping", shell=True)
            # Se cierra (y por tanto se guarda) el fichero
            f.close()
                

if __name__ == "__main__":
    bridge = CvBridge()
    # Abre un archivo en modo escritura
    f = open('/home/cristina/ws_helpbot/src/helpbot/src/maps/poses.txt', 'w')
    rospy.init_node('aruco_mapping')
    rospy.Subscriber('/aruco_detect_id', String, callback)
    rospy.spin()