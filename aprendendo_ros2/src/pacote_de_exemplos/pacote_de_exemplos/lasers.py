import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3


from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
import numpy as np
from numpy import random
from math import *
import matplotlib.pyplot as plt
import time 


class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug ('Definido o nome do nó para "R2D2"')

        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug ('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.get_logger().debug ('Definindo o publisher de controle do robo: "/cmd_Vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

    
    def listener_callback_laser(self, msg):
        self.laser = msg.ranges



        
    def run(self):
         
        self.get_logger().debug ('Executando uma iteração do loop de processamento de mensagens.')
        rclpy.spin_once(self)

        self.get_logger().debug ('Definindo mensagens de controde do robô.')
        self.ir_para_frente = Twist(linear=Vector3(x= 0.2,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))
        self.parar          = Twist(linear=Vector3(x= 0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))


        self.get_logger().info ('Entrando no loop princial do nó.')
        while(rclpy.ok):
            rclpy.spin_once(self)
            if min((self.laser[ 50:70])) is not None or min((self.laser[ 0:10])) is not None or self.min((self.laser[170:190])) is not None:
                self.get_logger().info ('Atualizando as distancias lidas pelo laser.')
                self.distancia_direita   = min((self.laser[ 50:70])) # -90 a -10 graus
                self.distancia_frente    = min((self.laser[ 0:10])) # -10 a  10 graus
                self.distancia_esquerda  = min((self.laser[170:190])) #  10 a  90 graus

            self.get_logger().info ('LaserLeft: ' + str(self.distancia_esquerda) + " LaserFront: " + str(self.distancia_frente) + "LaserRight: " + str(self.distancia_direita))




    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')


# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = R2D2()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if __name__ == '__main__':
    main()  

