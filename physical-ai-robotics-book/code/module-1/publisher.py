import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        
        # 1. Create a publisher
        # Type: String
        # Topic Name: 'motor_commands'
        # Queue Size: 10 (If sub is slow, keep last 10 msgs)
        self.publisher_ = self.create_publisher(String, 'motor_commands', 10)
        
        # 2. Create a timer to fire every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Step Forward {self.i}'
        
        # 3. Publish the message
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Brain Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    brain_node = BrainNode()
    
    try:
        rclpy.spin(brain_node)
    except KeyboardInterrupt:
        pass
    finally:
        brain_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
