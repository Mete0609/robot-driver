import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState

class ControllerBridgeNode(Node):
    def __init__(self):
        super().__init__('controller_bridge_node')

        # 订阅控制器发布的整型速度指令（单位 pulses/10ms）
        self.left_cmd_sub = self.create_subscription(
            Int32, '/left_wheel_joint/command', self.left_cmd_callback, 10)
        self.right_cmd_sub = self.create_subscription(
            Int32, '/right_wheel_joint/command', self.right_cmd_callback, 10)

        # 发布目标电机脉冲速度（单位 pulses/10ms）
        self.left_speed_pub = self.create_publisher(Int32, 'target_left_speed', 10)
        self.right_speed_pub = self.create_publisher(Int32, 'target_right_speed', 10)

        # 订阅编码器速度反馈
        self.left_encoder_sub = self.create_subscription(
            Int32, 'left_wheel_speed', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(
            Int32, 'right_wheel_speed', self.right_encoder_callback, 10)

        # 发布 JointState
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # 小车参数（需与你实际参数一致）
        self.wheel_radius = 0.05  # meters
        self.encoder_to_rad = (2 * 3.1415926) / (60000 * 10)  # 60000 ticks/rev, 每10ms

        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.left_pos = 0.0
        self.right_pos = 0.0

        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz

    def left_cmd_callback(self, msg):
        self.left_speed_pub.publish(Int32(data=msg.data))

    def right_cmd_callback(self, msg):
        self.right_speed_pub.publish(Int32(data=msg.data))

    def left_encoder_callback(self, msg):
        self.left_velocity = msg.data * self.encoder_to_rad
        self.left_pos += self.left_velocity * 0.01  # 10ms 时间步长

    def right_encoder_callback(self, msg):
        self.right_velocity = msg.data * self.encoder_to_rad
        self.right_pos += self.right_velocity * 0.01

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [self.left_pos, self.right_pos]
        msg.velocity = [self.left_velocity, self.right_velocity]
        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
