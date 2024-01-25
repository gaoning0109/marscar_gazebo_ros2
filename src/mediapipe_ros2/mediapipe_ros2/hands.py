#!/usr/bin/env python3

import copy
import csv
import itertools
import os
import sys
import cv2
from cv_bridge import CvBridge
import mediapipe as mp

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from marscar_gazebo_ros2.msg import Cableposseqq 
from ament_index_python import get_package_share_directory
mediapipe_dir = get_package_share_directory('mediapipe_ros2')
keypoint_classifier_dir = os.path.join(mediapipe_dir, 'keypoint_classifier')
sys.path.append(keypoint_classifier_dir)
from keypoint_classifier import KeyPointClassifier
resource_dir = os.path.join(mediapipe_dir, 'resource')


class Mediapipe(Node):
    def __init__(self) -> None:
        # ROS2 init
        super().__init__('mp_hands')
        
        self.bridge = CvBridge()
        self.create_subscription(Image,"/camera1/image_raw",self.imageflow_callback, 10)
        self.pub_gesture = self.create_publisher(String, '/recognized_gesture', 10)
        self.pub_gesture_gazebo = self.create_publisher(Cableposseqq, '/cable/cable_pub_testarray', 30)
        self.command = String()

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
                    static_image_mode=False,
                    max_num_hands=2,
                    min_detection_confidence=0.7)

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        self.keypoint_classifier = KeyPointClassifier(model_path=keypoint_classifier_dir + '/keypoint_classifier.tflite')
        with open(keypoint_classifier_dir + '/keypoint_classifier_label.csv',
                encoding='utf-8-sig') as f:
            keypoint_classifier_labels = csv.reader(f)
            self.keypoint_classifier_labels = [
                row[0] for row in keypoint_classifier_labels
            ]

        stop_img = cv2.imread(resource_dir + '/stop.png')
        front_img = cv2.imread(resource_dir + '/front.png')
        back_img = cv2.imread(resource_dir + '/back.png')
        left_img = cv2.imread(resource_dir + '/left.png')
        right_img = cv2.imread(resource_dir + '/right.png')
        img_mask = cv2.cvtColor(stop_img, cv2.COLOR_BGR2GRAY)

        self.img_mask = cv2.GaussianBlur(img_mask, (7,7), cv2.BORDER_DEFAULT)

        self.image_dict = {
            'stop': stop_img,
            'front': front_img,
            'back': back_img,
            'left': left_img,
            'right': right_img,
        }

    def which_command(self, gestures):
        if 'right_palm' in gestures and 'left_palm' in gestures:
            return 'front'
        if 'right_palm' in gestures:
            return 'right'
        if 'left_palm' in gestures:
            return 'left'
        if 'right_close' in gestures and 'left_close' in gestures:
            return 'back'
        return 'stop'

    def imageflow_callback(self,msg:Image) -> None:
        img_bgr = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        img_flipped = cv2.flip(img_bgr, 1)
        img_rgb = cv2.cvtColor(img_flipped, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)

        annotated_image = cv2.flip(img_bgr.copy(), 1)
        if results.multi_hand_landmarks is not None:
            gestures_detected = []
            for hand_landmarks in results.multi_hand_landmarks:
                
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style())

                landmark_list = self.calc_landmark_list(annotated_image, hand_landmarks)

                pre_processed_landmark_list = self.pre_process_landmark(
                    landmark_list)
                hand_sign_id = self.keypoint_classifier(pre_processed_landmark_list)

                gestures_detected.append(str(self.keypoint_classifier_labels[hand_sign_id]))

            command = self.which_command(gestures_detected)
            # annotated_image[self.img_mask != 0] = self.image_dict[command][self.img_mask != 0]

            self.command.data = command
            self.pub_gesture.publish(self.command)
        else:
            # annotated_image[self.img_mask != 0] = self.image_dict['stop'][self.img_mask != 0]
            pass

        scale_percent = 200 # percent of original size
        width = int(annotated_image.shape[1] * scale_percent / 100)
        height = int(annotated_image.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(annotated_image, dim, interpolation = cv2.INTER_AREA)

        cv2.imshow("mediapipe_hands",resized)
        cv2.waitKey(1)
        
    def pre_process_landmark(self, landmark_list):
        temp_landmark_list = copy.deepcopy(landmark_list)
        # Convert to relative coordinates
        base_x, base_y = 0, 0
        for index, landmark_point in enumerate(temp_landmark_list):
            if index == 0:
                base_x, base_y = landmark_point[0], landmark_point[1]

            temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
            temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y

        # Convert to a one-dimensional list
        temp_landmark_list = list(
            itertools.chain.from_iterable(temp_landmark_list))

        # Normalization
        max_value = max(list(map(abs, temp_landmark_list)))

        def normalize_(n):
            return n / max_value

        temp_landmark_list = list(map(normalize_, temp_landmark_list))

        return temp_landmark_list

    def calc_landmark_list(self, image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]
        a=Cableposseqq()
        landmark_point = []
        landmark_point3d = []
        # Key Point
        for _, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)
            landmark_z = landmark.z
            
            landmark_point.append([landmark_x, landmark_y])
            landmark_point3d.append([landmark.x, landmark.y,landmark.z])
            a.x.append(landmark.x*2)
            a.y.append(-landmark.z*2)
            a.z.append(-landmark.y*2+2)
        self.pub_gesture_gazebo.publish(a)
        # print(landmark_point3d)
        return landmark_point


def main(args = None):
    rclpy.init(args=args)
    mp_class = Mediapipe()

    try:
        rclpy.spin(mp_class)
    except KeyboardInterrupt:
        pass
    finally:
        mp_class.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()
