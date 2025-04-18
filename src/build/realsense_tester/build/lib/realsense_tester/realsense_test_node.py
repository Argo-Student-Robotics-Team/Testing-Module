#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class RealsenseTester(Node):
    def __init__(self):
        super().__init__('realsense_tester')
        self.get_logger().info("RealSense Testing Module Started")

        self.test_results = {
            'camera_connected': False,
            'color_stream': False,
            'depth_stream': False,
            'color_quality': False,
            'depth_quality': False
        }
        
        self.bridge = CvBridge()
        
        self.test_pub = self.create_publisher(Bool, '/realsense_test_status', 10)
        
        self.create_timer(5.0, self.run_tests)
        
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)
        
        self.color_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.info_callback,
            10)

    def color_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            #this should be adjusted
            if np.mean(cv_image) > 15 and self.check_active_publisher('/camera/camera/color/image_raw'):
                self.test_results['color_quality'] = True
            
            #this is for testing purposes
            cv2.imshow("Color Stream", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Color processing error: {str(e)}")

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            #this should be adjusted
            if np.count_nonzero(depth_image) > 100:
                self.test_results['depth_quality'] = True
                
            # this is for testing
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET)
            cv2.imshow("Depth Stream", depth_colormap)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Depth processing error: {str(e)}")

    def info_callback(self, msg):
        self.test_results['camera_connected'] = True

    def check_topics(self):
        topic_list = self.get_topic_names_and_types()
        topics = [t[0] for t in topic_list]
        
        self.test_results['color_stream'] = '/camera/camera/color/image_raw' in topics
        self.test_results['depth_stream'] = '/camera/camera/depth/image_rect_raw' in topics

    def run_tests(self):
        self.check_topics()
        
        result = Bool()
        result.data = all(self.test_results.values())
        
        self.test_pub.publish(result)
        
        self.get_logger().info("\n=== Test Results ===")
        for test, passed in self.test_results.items():
            status = "PASS" if passed else "FAIL"
            self.get_logger().info(f"{test.ljust(20)}: {status}")
        
        if result.data:
            self.get_logger().info("All tests passed successfully!")
        else:
            self.get_logger().warn("Some tests failed!")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = RealsenseTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('Shutting down due to keyboard interrupt')
    finally:
        tester.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()