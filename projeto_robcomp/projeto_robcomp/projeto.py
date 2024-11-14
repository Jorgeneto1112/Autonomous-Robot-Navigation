import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import numpy as np
from robcomp_util.odom import Odom
import time
from robcomp_interfaces.msg import DetectionArray, Detection  

# Adicione aqui os imports necessários

class SeguidorNode(Node, Odom):

    def __init__(self):
        super().__init__('seguidor_node')
        Odom.__init__(self)
        self.timer = self.create_timer(0.25, self.control)
        self.aruco_detected_time = None
        self.should_turn = False
        self.detected_once = 0
        self.running = True
        self.robot_state = 'centraliza'
        self.state_machine = {
        'para': self.para,
        'centraliza': self.centraliza,
        'segue': self.segue,
        'virar': self.virar
    }


        self.twist = Twist()
        self.cx = -1
        self.cy = -1
        self.w = 0
        self.posicao_x_inicial = (self.x, self.y)
        self.i = 0
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        self.saiu_da_area = 0
        self.bridge = CvBridge()
        # Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.flag_sub = self.create_subscription(
            String,
            '/vision/image_flag', # Mude o nome do tópico
            self.flag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.aruco_subscriber = self.create_subscription(
            DetectionArray, 
            '/aruco_detection', 
            self.aruco_callback, 10)
        

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def flag_callback(self, msg):
        self.running = bool(msg.data)

    def aruco_callback(self, msg):
        for detection in msg.deteccoes:
            aruco_id = detection.classe
            print(aruco_id[2])
            # Verifica se o ID é o alvo e se o tempo ainda não foi salvo
            if self.detected_once == 0: 
                print('entrei')
                if int(aruco_id[2]) == 0:
                
                    self.aruco_detected_time = time.time()  # Armazena o tempo uma vez
                    self.should_turn = True
                    self.detected_once = 1  # Atualiza a flag para impedir alterações
                    print("Tempo de detecção do Aruco salvo:", self.aruco_detected_time)
                


    def virar(self):

        self.twist.angular.z = -0.2
        erro = self.goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro))
        if abs(erro) <= np.deg2rad(2):
            self.twist = Twist()
            self.robot_state = 'segue'


    def image_callback(self, msg):
        if self.running:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # if Image
            
            height, width, _ = cv_image.shape
            self.w = width // 2  

            imagem_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])
            mask_hsv = cv2.inRange(imagem_hsv, lower_yellow, upper_yellow)
            mask = cv2.morphologyEx(mask_hsv, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            contornos, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
            if contornos:
                largest_contour = max(contornos, key=cv2.contourArea)

                
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    self.cx = int(M["m10"] / M["m00"]) 
                    self.cy = int(M["m01"] / M["m00"]) 
                    cv_image = cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 3)
                else:
                    self.cx = -1
            else:
                self.cx = -1  

            cv2.imshow('Linha Amarela', cv_image)
            cv2.waitKey(1)  
        
    def para(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

    def centraliza(self):
        if self.cx >= 0:  # Apenas se a linha for detectada
            error = self.w - self.cx  
            self.twist.angular.z = 0.003 * error
            self.twist.linear.x = 0.1  
            
            # Se a linha estiver suficientemente centralizada, mude para 'segue'
            if abs(error) < 15:  
                self.robot_state = 'segue'
        else:
            # Se a linha não for detectada, procurar a linha girando
            self.twist.angular.z = 0.1  # Girar para a direita
            self.twist.linear.x = 0.0   # Parar enquanto gira

            
    def segue(self):
        self.twist.linear.x = 0.1  # Velocidade de seguir a linha
        if self.cx >= 0:  # Se a linha estiver visível
            error = self.w - self.cx
            self.twist.angular.z = 0.003 * error
            
            # Verifique se já passaram 3 segundos desde que o Aruco foi detectado
            if self.should_turn and self.aruco_detected_time is not None:
                diferenca_tempo = time.time() - self.aruco_detected_time
                print(diferenca_tempo)
                if diferenca_tempo >= 18:
                    self.twist = Twist()
                    self.robot_state = 'virar'
                    self.goal_yaw = self.yaw - np.pi/2
                    self.should_turn = False  # Reinicie a flag após iniciar a virada
        else:
            self.twist.angular.z = 0.1  # Girar para procurar a linha
            self.twist.linear.x = 0.0

           



    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = SeguidorNode()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__== '_main_':
    main()