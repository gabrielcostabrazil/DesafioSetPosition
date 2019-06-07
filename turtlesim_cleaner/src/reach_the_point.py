#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
 
 
class TurtleBot:
 
    def __init__(self):
        # NODE: Cria o NODE 'turtlebot_controller' e garante que
        # seja um unico node (usando anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)
        
        # PUBLISHER: envia mensagem (publica) no topico '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # SUBSCRIBER: Um assinante (Subscriber) para o topico '/turtle1/pose'. self.update_pose e chamado
        # quando a mensagem do tipo Pose (posicao) e recebido
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """o update_pose e uma funcao de retorno utilizada pelo subscriber e saver en self.pose attibute"""
        """Funcao de retorno de chamada que e chamada quando uma nova mensagem do tipo Pose e recebida pelo assinante (subscriber)."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Distancia Euclidiana entre a posicao atual e a posicao goal"""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose): 
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
    
    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        """Move turtlesim para o goal"""
        """cria o objeto goal_pose que vai receber os inputs do usuario"""
        goal_pose = Pose()

        # Receber os inputs do usuario para posicao final
        goal_pose.x = input("Qual posicao em X que devo mover? ")
        goal_pose.y = input("Qual posicao em Y que devo mover? ")
    
        # Definir tolerancia para o erro(e.g. 0.01).
        distance_tolerance = input("Qual tolerancia para o erro? (colocar acima de 0, sugestao: 0.1): ")

        """Declaracao do objeto que seja publicado (assinado) em /turtle1/cmd_vel """
        vel_msg = Twist()

        """no while loop a Distancia continuara sendo publicada ate que a distancia da tartaruga"""
        """seja menor que a tolerancia em distance_tolerance"""
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
    
            # Velocidade linear no eixo-X.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Velicidade angular no eixo-Z.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publicando a vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publica a taxa (rate) desejada.
            self.rate.sleep()

        # Parando o rorobo apos o termino do movimento.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # Se apertarmos Ctrl + C, paramos o node.
        rospy.spin()


if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass