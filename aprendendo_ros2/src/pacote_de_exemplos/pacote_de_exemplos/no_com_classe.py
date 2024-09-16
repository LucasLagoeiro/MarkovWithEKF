import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


# Cria o nó do ROS como uma clase do python
class MeuNo(Node):

    # Contrutor do nó
    def __init__(self):
        # Aqui é definido o nome do nó
        super().__init__('no_com_classe')

        self.get_logger().debug ('Definindo o subscriber do laser: "/scan"')
        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)


    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
    



        # # Define o nível do logger
        # logger = self.get_logger()
        # logger.set_level(LoggingSeverity.INFO)

    # Aqui o seu nó está executando no ROS
    def run(self):
        # Executa uma iteração do loop de processamento de mensagens.
        while(rclpy.ok):
            rclpy.spin_once(self)
            self.get_logger().info ('Atualizando as distancias lidas pelo laser.')
            self.distancia_direita   = min((self.laser[  0: 80])) # -90 a -10 graus
            self.distancia_frente    = min((self.laser[ 80:100])) # -10 a  10 graus
            self.distancia_esquerda  = min((self.laser[100:180])) #  10 a  90 graus
            self.get_logger().info ("Valores dos lasers: " + " E: " + str(self.distancia_esquerda) + " D: " + str(self.distancia_direita) )



    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')


# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = MeuNo()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    
if __name__ == '__main__':
    main()    




