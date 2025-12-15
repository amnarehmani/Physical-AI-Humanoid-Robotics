import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        # Initialize the node with the name 'simple_node'
        super().__init__('simple_node')
        # Log a message to the console
        self.get_logger().info('The Nervous System is Active!')

def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of our node
    node = SimpleNode()
    
    # Keep the node running (listening for events)
    # Since this node does nothing else, it will just sit here
    # untill you press Ctrl+C
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
