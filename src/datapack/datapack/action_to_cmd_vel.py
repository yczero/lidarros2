import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ActionToCmdVel(Node):
    def __init__(self):
        super().__init__('action_to_cmd_vel')
        self.subscription = self.create_subscription(
            String,
            '/turtle_action',   # Windows에서 Publish
            self.action_callback,
            10
        )
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('action_to_cmd_vel 노드 시작')

    def action_callback(self, msg):
        action = msg.data
        twist = Twist()

        if action == 'forward':
            twist.linear.x = 1.0
            twist.angular.z = 0.0
        elif action == 'left':
            twist.linear.x = 0.0
            twist.angular.z = 1.0
        elif action == 'right':
            twist.linear.x = 0.0
            twist.angular.z = -1.0
        elif action == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)
        self.get_logger().info(f'액션: {action} → cmd_vel Publish')

def main(args=None):
    rclpy.init(args=args)
    node = ActionToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()