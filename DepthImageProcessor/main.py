import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class DepthImageProcessor(Node):
    def __init__(self, camera_name):
        super().__init__('depth_image_processor')
        self.subscription_distance_from_depth = self.create_subscription(
            Image,
            camera_name + '/depth/image_rect_raw',
            self.distance_from_depth_callback,
            10
        )
        self.logger = self.get_logger()

    def distance_from_depth_callback(self, depth_msg):
        raw_data = depth_msg.data
        height = depth_msg.height
        width = depth_msg.width
        np_data = np.frombuffer(raw_data, dtype=np.uint16).reshape(height, width)

        for y in range(height):
            for x in range(width):
                if x % 100 != 0 or y % 100 != 0:
                    continue

                distance = np_data[y, x]
                print(f"y:{y} x:{x} {distance} mm")
                self.logger.info(f"y:{y} x:{x} {distance} mm")

def main(args=None):
    rclpy.init(args=args)
    camera_name = 'your_camera_name'
    depth_image_processor = DepthImageProcessor(camera_name)
    rclpy.spin(depth_image_processor)
    depth_image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

