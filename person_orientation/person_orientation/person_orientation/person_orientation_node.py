import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist, PoseStamped
import cv2
import mediapipe as mp
import math
from cv_bridge import CvBridge
import numpy as np
from tf2_msgs.msg import TFMessage
from rclpy.duration import Duration

class PersonOrientationNode(Node):

    def __init__(self):
        super().__init__('person_orientation_node')
        
        # Inicializa o CvBridge
        self.bridge = CvBridge()

        # Subscreve aos tópicos da câmera, LiDAR e TF
        self.image_subscriber = self.create_subscription(
            Image, '/a200_0000/sensors/camera_0/color/image', self.image_callback, 10)
        self.lidar_subscriber = self.create_subscription(
            LaserScan, '/a200_0000/sensors/lidar2d_0/scan', self.lidar_callback, 10)
        self.tf_subscriber = self.create_subscription(
            TFMessage, '/a200_0000/tf', self.tf_callback, 10)

        # Publicadores
        self.orientation_publisher = self.create_publisher(Float32, '/person_orientation', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/a200_0000/cmd_vel', 10)
        self.explore_resume_publisher = self.create_publisher(Bool, '/a200_0000/explore/resume', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/a200_0000/goal_pose', 10)

        # Inicializa o MediaPipe para detecção de pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

        # Variáveis de controle
        self.goal_sent = False
        self.last_person_angle = None
        self.obstacle_distance = None

        # Variáveis para armazenar a posição do robô no mundo (obtida via TF)
        self.robot_world_x = None
        self.robot_world_y = None
        self.robot_yaw = None  # Em radianos
        self.erro_x = 1
        self.erro_a = 1

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.flip(cv_image, 0)
        except Exception as e:
            self.get_logger().error(f"Erro ao converter a imagem: {e}")
            return

        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)

        if results.pose_landmarks:
            # Acrescentado: Publica false para parar a exploração ao detectar uma pessoa
            self.explore_resume_publisher.publish(Bool(data=False))
            
            # Obter landmarks dos ombros e pulsos
            shoulder_left = results.pose_landmarks.landmark[11]  # Ombro esquerdo
            shoulder_right = results.pose_landmarks.landmark[12]   # Ombro direito
            wrist_left = results.pose_landmarks.landmark[15]       # Pulso esquerdo
            wrist_right = results.pose_landmarks.landmark[16]      # Pulso direito
            
            # Denormaliza para pixels
            image_width, image_height = image_rgb.shape[1], image_rgb.shape[0]
            shoulder_left_x = shoulder_left.x * image_width
            shoulder_left_y = shoulder_left.y * image_height
            shoulder_right_x = shoulder_right.x * image_width
            shoulder_right_y = shoulder_right.y * image_height
            wrist_left_x = wrist_left.x * image_width
            wrist_left_y = wrist_left.y * image_height
            wrist_right_x = wrist_right.x * image_width
            wrist_right_y = wrist_right.y * image_height
            
            # Calcula o centro dos ombros e a posição média dos pulsos
            shoulder_center_x = (shoulder_left_x + shoulder_right_x) / 2
            wrist_avg_x = (wrist_left_x + wrist_right_x) / 2
            wrist_avg_y = (wrist_left_y + wrist_right_y) / 2

            # Vetor do centro dos ombros até a média dos pulsos
            delta_x = wrist_avg_x - shoulder_center_x
            delta_y = wrist_avg_y - ((shoulder_left_y + shoulder_right_y) / 2)
            angle_rad = math.atan2(delta_y, delta_x)
            angle_deg = math.degrees(angle_rad)
            
            # Publica a orientação detectada
            self.publish_orientation(angle_deg)
            self.last_person_angle = angle_deg
            
            # Calcular erro horizontal (para centralização da pessoa na imagem)
            self.error_a = (image_width / 2) - shoulder_center_x  
            Kp = 0.005  
            angular_z = Kp * self.error_a
            try:
                self.erro_x = (self.obstacle_distance - 2)
                linear_x = Kp * 100 * self.erro_x
            except Exception as e:
                linear_x = 0.0

            self.publish_cmd_vel(angular_z, linear_x)

            # Verifica se os erros são pequenos para disparar a publicação do goal
            threshold_linear = 0.1
            threshold_angular = 0.1
            print('Aproximando, Erro x: ',self.erro_x,'Erro angular: ',self.error_a)
            if (abs(self.erro_x) < threshold_linear and 
                abs(self.error_a) < threshold_angular):
                print('erro ta pequeno')
                if self.robot_world_x is not None and self.robot_yaw is not None:
                    self.publish_goal_pose_manual()
                    self.goal_sent = True
                    self.get_logger().info("Goal enviado; encerrando nó.")
                    # Encerra o nó após publicar o goal
                    rclpy.shutdown()
                else:
                    self.get_logger().warn("Posição do robô não disponível; não é possível converter o goal.")
                        # Executa a sequência de parada de exploração e envio do goal pose (apenas uma vez)
            if not self.goal_sent:
                bool_msg = Bool()
                bool_msg.data = False
                self.explore_resume_publisher.publish(bool_msg)
                self.get_logger().info("Publicado /a200_0000/explore/resume com data: False")
                self.goal_sent = True
        else:
            self.get_logger().info("Nenhuma pessoa detectada na imagem.")

    def lidar_callback(self, msg):
        # Processa os dados do LiDAR para obter a distância até o obstáculo
        ranges = list(msg.ranges)
        total_medidas = len(ranges)
        indice_frente = total_medidas // 2
        largura_janela = 5
        inicio = max(0, indice_frente - largura_janela)
        fim = min(total_medidas, indice_frente + largura_janela + 1)
        self.obstacle_distance = min(ranges[inicio:fim])

    def publish_orientation(self, angle):
        msg = Float32()
        msg.data = angle
        self.orientation_publisher.publish(msg)

    def tf_callback(self, msg: TFMessage):
        # Filtra e imprime os TFs com child_frame_id "base_link" e armazena a posição do robô no mundo
        for transform in msg.transforms:
            if transform.child_frame_id == "base_link":
                '''self.get_logger().info(
                    f"TF recebido - Frame: {transform.header.frame_id} -> child_frame_id: {transform.child_frame_id}\n"
                    f"  Tradução: x={transform.transform.translation.x:.2f}, "
                    f"y={transform.transform.translation.y:.2f}, "
                    f"z={transform.transform.translation.z:.2f}\n"
                    f"  Rotação: x={transform.transform.rotation.x:.2f}, "
                    f"y={transform.transform.rotation.y:.2f}, "
                    f"z={transform.transform.rotation.z:.2f}, "
                    f"w={transform.transform.rotation.w:.2f}"
                )'''
                # Armazena a posição e a orientação do robô (no referencial "odom")
                self.robot_world_x = transform.transform.translation.x
                self.robot_world_y = transform.transform.translation.y
                # Converte o quaternion para yaw (em radianos)
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w
                self.robot_yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

    def publish_cmd_vel(self, angular_z, linear_x):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = -angular_z  # Inverte o sinal se necessário
        self.cmd_vel_publisher.publish(twist_msg)

    def publish_goal_pose_manual(self):
        """
        Calcula a goal pose no referencial do robô e a converte manualmente para o referencial do mundo ("odom")
        usando a posição e orientação do robô armazenadas.
        O ponto desejado no referencial do robô é: (d, 1.0), onde d é a distância do obstáculo.
        """
        psi = math.radians(self.last_person_angle)  # Ângulo detectado a partir da imagem
        d = self.obstacle_distance  # Distância medida pelo LiDAR
        
        # Ponto desejado no referencial do robô: (x = d, y = 1.0)
        x_goal_robot = d
        y_goal_robot = 1.0

        # Conversão manual para o referencial do mundo ("odom"):
        # Se o robô está em (x_r, y_r) com orientação theta (self.robot_yaw),
        # então o ponto no mundo é:
        #   x_goal_world = x_r + cos(theta)*x_goal_robot - sin(theta)*y_goal_robot
        #   y_goal_world = y_r + sin(theta)*x_goal_robot + cos(theta)*y_goal_robot
        x_goal_world = self.robot_world_x + math.cos(self.robot_yaw)*x_goal_robot - math.sin(self.robot_yaw)*y_goal_robot
        y_goal_world = self.robot_world_y + math.sin(self.robot_yaw)*x_goal_robot + math.cos(self.robot_yaw)*y_goal_robot

        # Define a orientação desejada no mundo:
        #   world_yaw = robot_yaw + psi
        world_yaw = self.robot_yaw + psi
        qz = math.sin(world_yaw/2.0)
        qw = math.cos(world_yaw/2.0)

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = x_goal_world
        goal_msg.pose.position.y = y_goal_world
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw

        self.goal_pose_publisher.publish(goal_msg)
        self.get_logger().info(f"Goal pose publicado (odom): x={x_goal_world:.2f}, y={y_goal_world:.2f}, yaw={math.degrees(world_yaw):.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = PersonOrientationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
