import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2
from cv_bridge import CvBridge
import mediapipe as mp
import collections
from digit_meter.utils import is_finger_raised
 

class DigitPublisher(Node):

    def __init__(self):
        super().__init__('digit_publisher')
        self.window = collections.deque()
        self.window_size = 3
        self.window_sum = 0
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hand = self.mp_hands.Hands(max_num_hands=1)
        self.camera_subscriber_ = self.create_subscription(Image, "/webcam_frame", self.image_callback, 10)
        self.digit_publisher_ = self.create_publisher(Int32, "/microROS/int32_subscriber", 10)
        self.finger_landmark_dict = {'index': (5, 8), 'middle': (9, 12), 'ring': (13, 16), 'pinky': (17, 20), 'thumb': (2, 4), 'wrist_id': 0}  #base, tip
        self.br = CvBridge()
        self.prev_num_fingers = 0

    def image_callback(self, bgr_frame_ros):
        bgr_frame = self.br.imgmsg_to_cv2(bgr_frame_ros)
        rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        result = self.hand.process(rgb_frame)
        hand_landmarks = result.multi_hand_landmarks
        
        # use only first hand
        if hand_landmarks: # if results.multi_hand_landmarks fails, continue
            first_hand = hand_landmarks[0]
            self.mp_drawing.draw_landmarks(bgr_frame, first_hand, self.mp_hands.HAND_CONNECTIONS)
            cur_num_raised_fingers = 0
            for finger in self.finger_landmark_dict.keys():
                if finger == 'wrist_id': continue
                finger_raised = is_finger_raised(finger, first_hand.landmark, self.finger_landmark_dict)
                if finger_raised:
                    cur_num_raised_fingers += 1
            if len(self.window) >= self.window_size:
                remove_value = self.window.popleft()
                self.window_sum -= remove_value 
            self.window.append(cur_num_raised_fingers)
            self.window_sum += cur_num_raised_fingers
            
            avg_num_raised_fingers = int(self.window_sum/self.window_size)
            if avg_num_raised_fingers != self.prev_num_fingers:
                msg = Int32()
                msg.data = int((5-avg_num_raised_fingers)/5*180)
                self.digit_publisher_.publish(msg)
                self.prev_num_fingers = avg_num_raised_fingers
            self.get_logger().info("num_fingers: {}".format(cur_num_raised_fingers))
            self.get_logger().info("avg num fingers: {}".format(avg_num_raised_fingers))
        cv2.imshow("camera", bgr_frame)
        cv2.waitKey(1)
        



def main(args=None):
    rclpy.init(args=args)
    digit_publisher = DigitPublisher()
    rclpy.spin(digit_publisher)
    digit_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()