import rclpy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('webcam_publisher')

    # Create a publisher for camera frames and camera info
    image_pub = node.create_publisher(Image, 'camera/image', 10)
    camera_info_pub = node.create_publisher(CameraInfo, 'camera/camera_info', 10)

    # Open the webcam
    cap = cv2.VideoCapture(2)

    bridge = CvBridge()

    while rclpy.ok():
        ret, frame = cap.read()

        if ret:
            # Convert OpenCV image to ROS Image message
            img_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = node.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_link'
            image_pub.publish(img_msg)

            # Create and publish CameraInfo message (assuming a simple camera setup)
            camera_info = CameraInfo()
            camera_info.header = img_msg.header
            camera_info.height = frame.shape[0]
            camera_info.width = frame.shape[1]
            camera_info.distortion_model = 'plumb_bob'
            camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Use 'd' instead of 'D'
            camera_info.k = [600.0, 0.0, frame.shape[1] / 2.0, 0.0, 600.0, frame.shape[0] / 2.0, 0.0, 0.0, 1.0]  # Use 'k' instead of 'K'
            camera_info.p = [600.0, 0.0, frame.shape[1] / 2.0, 0.0, 0.0, 600.0, frame.shape[0] / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]  # Use 'p' instead of 'P'
            camera_info_pub.publish(camera_info)

    rclpy.spin(node)

    # Release the webcam and shutdown
    cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
