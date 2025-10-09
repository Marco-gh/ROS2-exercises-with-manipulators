import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from math import pi, degrees

from robot_square.utils.inv_kin import inverse_kin_plan_3dof
from robot_square.utils.interpol import interpolator

class AnglePublisher(Node):
    def __init__(self):
        super().__init__('angle_publisher')
        self.publisher = self.create_publisher(JointState,'joint_states',10)
        timer_per = 0.15
        n = 25 # Numero campioni per interpolazione
        self.timer = self.create_timer(timer_per,self.timer_callback)

        self.joint1 = pi/6
        self.joint2 = pi/2
        self.joint3 = pi/2

        self.l1 = 0.5
        self.l2 = 0.5
        self.l3 = 0.5

        self.j = 0

        # Quadrato centrato in (x0,y0) con lato l
        l = 0.3
        h = l/2
        x0 = 0.25
        y0 = 0.25

        p1 = (x0-h,y0-h)
        p2 = (x0+h,y0-h)
        p3 = (x0+h,y0+h)
        p4 = (x0-h,y0+h)
        # self.sq = [p1,p2,p3,p4] # Solo i vertici del quadrato
        self.sq = interpolator(p1,p2,n)
        self.sq.extend(interpolator(p2,p3,n))
        self.sq.extend(interpolator(p3,p4,n))
        self.sq.extend(interpolator(p4,p1,n))

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1','joint2','joint3']

        try:
            self.joint1, self.joint2, self.joint3 = inverse_kin_plan_3dof(self.l1, self.l2, self.l3, self.sq[self.j][0], self.sq[self.j][1], pi/2)
            self.get_logger().info(f"self.j={self.j}")
            self.j = (self.j+1) % len(self.sq)
            self.get_logger().info(f"q1: {degrees(self.joint1):.2f}°, q2: {degrees(self.joint2):.2f}°, q3: {degrees(self.joint3):.2f}°")
        except ValueError as e:
            self.get_logger().warn(f"IK fallita: {e}")
            return

        msg.position = [self.joint1, self.joint2, self.joint3]
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    angle_publisher = AnglePublisher()

    rclpy.spin(angle_publisher)

    angle_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()