#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.window_name = "camera"
        self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.twist_msg = Twist()
        self.point = None
        self.image_height = 512
        self.image_width = 700
        self.circle_radius = 10
        self.line_color = (0, 255, 0)


        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.param_markers = aruco.DetectorParameters()
        
    def draw_horizontal_line(self, image):
        line_position = image.shape[0] // 2
        cv2.line(image, (0, line_position), (image.shape[1], line_position), (255, 255, 255), 2)


    def listener_callback(self, image_data):
        cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        gray_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        marker_corners, marker_IDs, _ = aruco.detectMarkers(gray_frame, self.marker_dict, parameters=self.param_markers)

        if marker_IDs is not None and len(marker_IDs) > 0:
            for i in range(len(marker_IDs)):
                aruco.drawDetectedMarkers(cv_image, marker_corners, marker_IDs)
                cX, cY = np.int0(np.mean(marker_corners[i][0], axis=0))
                cv2.drawMarker(cv_image, (cX, cY), (0, 0, 255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=10, thickness=2)
                result = 1 if cY < cv_image.shape[0] // 2 else 0
                print("Result:", result)


                if result == 1:
                    self.twist_msg.linear.x = 2.0
                elif result == 0:
                    self.twist_msg.linear.x = -2.0
                else:
                    self.twist_msg.linear.x = 0.0
                self.twist_msg.linear.y = 0.0
                self.twist_msg.linear.z = 0.0
                self.twist_msg.angular.x = 0.0
                self.twist_msg.angular.y = 0.0
                self.twist_msg.angular.z = 0.0
                self.publisher.publish(self.twist_msg)
        self.draw_horizontal_line(cv_image)
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1)
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_node = MinimalSubscriber()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




