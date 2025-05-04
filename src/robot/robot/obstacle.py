# Importation de la bibliothèque ROS pour Python
import rclpy
# Pour créer des noeuds ROS
from rclpy.node import Node
# Message type pour commander la vitesse du robot
from geometry_msgs.msg import Twist
# Message type pour recevoir les données du laser
from sensor_msgs.msg import LaserScan

# Classe principale pour contrôler le robot
class RobotController(Node):
    def __init__(self):
        # Initialisation du noeud avec le nom 'robot_control'
        super().__init__('robot_control')
        # Création d'un publisher pour envoyer les commandes de vitesse
        self.vel_publisher = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        # Création d'un subscriber pour recevoir les données du laser
        self.laser_subscription = self.create_subscription(LaserScan, '/robot2/scan', self.laser_callback, 10)
        # Vitesse normale du robot
        self.speed = 0.3
        # Distance minimale aux obstacles (initialisée à l'infini)
        self.min_distance = float('inf')  
        # Seuil à partir duquel le robot commence à ralentir
        self.slow_down_threshold = 1.0

        # Message d'information au démarrage
        self.get_logger().info("Démarrage du contrôleur automatique avec détection d'obstacles")
        # Appel de la fonction pour démarrer le mouvement
        self.start_moving()

    # Fonction pour démarrer le mouvement du robot
    def start_moving(self):
        # Création d'un message Twist pour la vitesse
        twist = Twist()
        # Définition de la vitesse linéaire
        twist.linear.x = self.speed
        # Publication de la commande de vitesse
        self.vel_publisher.publish(twist)
        # Affichage de la vitesse initiale
        self.get_logger().info(f"Démarrage automatique : vitesse initiale est : {self.speed} m/s")

    # Fonction appelée à chaque réception de données laser
    def laser_callback(self, msg):
        # Filtrage des distances valides (supérieures à 0.5m et non infinies)
        valid_ranges = [d for d in msg.ranges if d > 0.5 and d < float('inf')]
        # Calcul de la distance minimale aux obstacles
        self.min_distance = min(valid_ranges) if valid_ranges else float('inf')
        
        # Création d'un nouveau message Twist
        twist = Twist()
        
        # Si obstacle très proche (moins de 0.7m)
        if self.min_distance < 0.7:
            # Arrêt complet du robot
            twist.linear.x = 0.0  
            # Message d'avertissement
            self.get_logger().warn("Obstacle détecté")
        # Si obstacle dans la zone de ralentissement
        elif self.min_distance < self.slow_down_threshold:
            # Calcul de la réduction de vitesse proportionnelle à la distance
            speed_reduction = (self.min_distance / self.slow_down_threshold) * self.speed
            # Vitesse réduite (minimum 0.1 m/s)
            twist.linear.x = max(0.1, speed_reduction)
            # Information sur le ralentissement
            self.get_logger().info(f"Obstacle proche, ralentissement à {twist.linear.x:.2f} m/s")
        # Si pas d'obstacle proche
        else:
            # Vitesse normale
            twist.linear.x = self.speed

        # Envoi de la commande de vitesse
        self.vel_publisher.publish(twist)

# Fonction principale
def main():
    # Initialisation de ROS
    rclpy.init()
    # Création de l'instance du contrôleur
    node = RobotController()
    # Lancement de la boucle principale
    rclpy.spin(node)
    # Arrêt propre de ROS
    rclpy.shutdown()

# Point d'entrée du programme
if __name__ == '__main__':
    main()