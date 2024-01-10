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
        self.initialized_graph = False

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

        # グラフの初期化
        if not self.initialized_graph:
            plt.ion()
            self.im = plt.imshow(np_data, cmap='hot', interpolation='nearest', vmin=0, vmax=2000)
            plt.colorbar(label='Distance (mm)')
            plt.xlabel('X Pixel')
            plt.ylabel('Y Pixel')
            plt.title('Depth Image Distance Data')
            self.initialized_graph = True
        else:
            # データのみを更新
            self.im.set_data(np_data)
            self.im.set_clim(vmin=0, vmax=2000)  # データ範囲の更新
            plt.draw()

        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    camera_name = 'camera'
    depth_image_processor = DepthImageProcessor(camera_name)
    rclpy.spin(depth_image_processor)
    depth_image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
