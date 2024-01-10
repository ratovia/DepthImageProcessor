import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt
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

        # ロギングのための条件付きチェック
        for y in range(height):
            for x in range(width):
                if x % 100 == 0 and y % 100 == 0:
                    self.logger.info(f"y:{y} x:{x} {np_data[y, x]} mm")

        # 2次元データをプロット
        plt.imshow(np_data, cmap='hot', interpolation='nearest')
        plt.colorbar(label='Distance (mm)')
        plt.xlabel('X Pixel')
        plt.ylabel('Y Pixel')
        plt.title('Depth Image Distance Data')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    camera_name = 'camera_1'
    depth_image_processor = DepthImageProcessor(camera_name)
    rclpy.spin(depth_image_processor)
    depth_image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
