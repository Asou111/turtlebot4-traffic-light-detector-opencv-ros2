# Importation des bibliothèques nécessaires
import rclpy  # Bibliothèque ROS pour Python
from rclpy.node import Node  # Pour créer des noeuds ROS
import cv2  # OpenCV pour le traitement d'image
import numpy as np  # Numpy pour les calculs numériques
from sensor_msgs.msg import Image  # Type de message pour les images
from geometry_msgs.msg import Twist  # Type de message pour les commandes de vitesse
from cv_bridge import CvBridge  # Pour convertir entre ROS Image et OpenCV

class TrafficLightDetector(Node):
    def __init__(self):  # __init__ avec 2 underscores
        # Initialisation du noeud ROS avec le nom 'traffic_light_detector'
        super().__init__('traffic_light_detector')
        
        # Initialisation de CvBridge pour la conversion d'images
        self.bridge = CvBridge()
        
        # Création d'un abonnement au topic d'image de la caméra
        self.image_sub = self.create_subscription(
            Image,
            '/robot2/oakd/rgb/preview/image_raw',  # Topic de l'image
            self.image_callback,  # Fonction de callback
            10  # Taille de la file d'attente
        )
        
        # Création d'un publisher pour les commandes de vitesse
        self.vel_publisher = self.create_publisher(Twist, 'robot2/cmd_vel', 10)
        
        # Définition des vitesses pour chaque couleur de feu
        self.speed_red = 0.0  # Arrêt complet pour feu rouge
        self.speed_yellow = 0.05  # Ralentissement pour feu jaune
        self.speed_green = 0.15  # Vitesse normale pour feu vert
        
        # Vitesse actuelle, initialisée à vert par défaut
        self.speed = self.speed_green
        
        # Indicateur de détection de feu rouge
        self.red_detected = False  
        
        # Création d'un timer pour mettre à jour la vitesse régulièrement
        self.create_timer(0.1, self.update_speed)

    def detect_traffic_light(self, frame):
        # Conversion de l'image en espace colorimétrique HSV (plus facile pour la détection de couleur)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Définition des plages de couleur pour le rouge (2 plages car le rouge est à cheval sur 0°)
        red_lower1, red_upper1 = np.array([0, 150, 100]), np.array([10, 255, 255])
        red_lower2, red_upper2 = np.array([170, 150, 100]), np.array([180, 255, 255])
        
        # Définition des plages de couleur pour le jaune
        yellow_lower, yellow_upper = np.array([20, 100, 100]), np.array([40, 255, 255])
        
        # Définition des plages de couleur pour le vert
        green_lower, green_upper = np.array([40, 60, 60]), np.array([85, 255, 255])
        
        # Création des masques pour chaque couleur
        red_mask = cv2.inRange(hsv, red_lower1, red_upper1) + cv2.inRange(hsv, red_lower2, red_upper2)
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        
        # Opération morphologique pour éliminer le bruit
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        
        # Détection des contours pour chaque couleur
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Couleur détectée par défaut
        detected_color = "NONE"
        
        # Traitement des contours rouges détectés
        for contour in red_contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if radius > 10:  # Filtre les petits contours
                center = (int(x), int(y))
                radius = int(radius)
                # Dessine un cercle autour du feu rouge détecté
                cv2.circle(frame, center, radius, (0, 0, 255), 3)
                # Ajoute un texte "RED"
                cv2.putText(frame, "RED", (center[0] - radius, center[1] - radius - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                detected_color = "RED"
        
        # Traitement des contours jaunes détectés
        for contour in yellow_contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if radius > 10:
                center = (int(x), int(y))
                radius = int(radius)
                # Dessine un cercle autour du feu jaune
                cv2.circle(frame, center, radius, (0, 255, 255), 3)
                # Ajoute un texte "YELLOW"
                cv2.putText(frame, "YELLOW", (center[0] - radius, center[1] - radius - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
                detected_color = "YELLOW"
        
        # Traitement des contours verts détectés
        for contour in green_contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if radius > 10:
                center = (int(x), int(y))
                radius = int(radius)
                # Dessine un cercle autour du feu vert
                cv2.circle(frame, center, radius, (0, 255, 0), 3)
                # Ajoute un texte "GREEN"
                cv2.putText(frame, "GREEN", (center[0] - radius, center[1] - radius - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                detected_color = "GREEN"
        
        # Retourne la couleur détectée et l'image annotée
        return detected_color, frame

    def image_callback(self, msg):
        # Conversion du message ROS en image OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Détection de la couleur du feu et récupération de l'image traitée
        light_color, processed_frame = self.detect_traffic_light(frame)
        
        # Logique de contrôle de vitesse en fonction de la couleur détectée
        if light_color == "RED":
            self.speed = self.speed_red
            self.red_detected = True  # Arrêt et maintien en position
        elif light_color == "GREEN":
            self.speed = self.speed_green
            self.red_detected = False  # Reprendre la marche
        elif light_color == "YELLOW" and not self.red_detected:
            self.speed = self.speed_yellow
        elif light_color == "NONE" and self.red_detected:
            self.speed = self.speed_red  # Restez à l'arrêt en cas de détection d'un feu rouge

        # Affichage des informations dans la console
        self.get_logger().info(f"Traffic Light: {light_color}, Speed: {self.speed}")
        
        # Affichage de l'image avec les détections
        cv2.imshow("Traffic Light Detection", processed_frame)
        cv2.waitKey(1)

    def update_speed(self):
        # Création d'un message Twist pour la commande de vitesse
        twist = Twist()
        twist.linear.x = self.speed  # Vitesse linéaire
        twist.angular.z = 0.0  # Pas de rotation
        # Publication de la commande de vitesse
        self.vel_publisher.publish(twist)

def main(args=None):
    # Initialisation de ROS
    rclpy.init(args=args)
    
    # Création de l'instance du détecteur de feu tricolore
    node = TrafficLightDetector()
    
    # Lancement de la boucle ROS
    rclpy.spin(node)
    
    # Nettoyage à l'arrêt
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":  
    main()
