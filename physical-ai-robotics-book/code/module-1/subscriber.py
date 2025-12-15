import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MuscleNode(Node):
    def __init__(self):
        super().__init__('muscle_node')
        
        # 1. Create subscription
        self.subscription = self.create_subscription(
            String,
            'motor_commands',
            self.listener_callback,
            10)
        
        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg):
        # 2. React to the message
        self.get_logger().info(f'Muscle Received: "{msg.data}" -> FLEXING LEG!')

def main(args=None):
    rclpy.init(args=args)
    muscle_node = MuscleNode()
    
    try:
        rclpy.spin(muscle_node)
    except KeyboardInterrupt:
        pass
    finally:
        muscle_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
