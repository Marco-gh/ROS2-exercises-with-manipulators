import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander')
        
        self.pub0 = self.create_publisher(Float64, '/arm/joint0/cmd_pos', 10)
        self.pub1 = self.create_publisher(Float64, '/arm/joint1/cmd_pos', 10)
        self.pub2 = self.create_publisher(Float64, '/arm/joint2/cmd_pos', 10)
        self.pub3 = self.create_publisher(Float64, '/arm/joint3/cmd_pos', 10)

        self.sub = self.create_subscription(
            JointState, '/arm/joint_state', self.joint_state_cb, 10
        )

        self.m0 = Float64()
        self.m1 = Float64()
        self.m2 = Float64()
        self.m3 = Float64()

        self.timer = self.create_timer(0.1, self.send_cmds)

    def send_cmds(self):

        self.m0.data = 0.0
        self.m1.data = 0.0
        self.m2.data = self.m1.data + 0.01
        self.m3.data = 0.0

        self.pub0.publish(self.m0)
        self.pub1.publish(self.m1)
        self.pub2.publish(self.m2)
        self.pub3.publish(self.m3)

    def joint_state_cb(self, msg: JointState):
        self.get_logger().info(f'names={msg.name}, pos={msg.position}')

def main():
    rclpy.init()
    node = ArmCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()