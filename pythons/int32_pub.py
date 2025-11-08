import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class Int32Publisher(Node):
    def __init__(self):
        super().__init__('int32_publisher')
        self.publisher_ = self.create_publisher(Int32, 'l_int32_subpub', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 每秒发布一次
        self.count = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.count
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = Int32Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()