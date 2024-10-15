import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

# help from : https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
class WebcamPublisher(Node):

    def __init__(self):
        super().__init__('webcam_publisher')

        self.frame_publisher_ = self.create_publisher(Image, "/webcam_frame", 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)

        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
   

    def timer_callback(self):
        success, bgr_frame = self.cap.read() 
        # convert frame from BGR to RGB
        if success:
            
            self.frame_publisher_.publish(self.br.cv2_to_imgmsg(bgr_frame))
            self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)

    webcam_publisher = WebcamPublisher()

    rclpy.spin(webcam_publisher)
    webcam_publisher.cap.release()
    webcam_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()