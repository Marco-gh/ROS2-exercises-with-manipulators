import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from math import pi

class AnglePublisher(Node):
    def __init__(self):
        super().__init__('angle_publisher')
        self.publisher = self.create_publisher(JointState,'joint_states',10)
        timer_period = 0.25
        self.timer = self.create_timer(timer_period,self.timer_callback)
        
        self.q1 = 0
        self.q2 = pi/2
        
        self.dir1 = 1
        self.dir2 = 1
        self.incr = pi/24

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1','joint2']
    
        # Incremento angolo
        if self.q1+self.incr < pi and self.dir1:
            self.q1 = self.q1 + self.incr
        else:
            self.q1 = self.q1 - self.incr
        if self.q1+self.incr >= pi:
            self.dir1 = 0
        elif self.q1-self.incr < 0:
            self.dir1 = 1

        # Incremento angolo
        if self.q2+self.incr < pi and self.dir2:
            self.q2 = self.q2 + self.incr
        else:
            self.q2 = self.q2 - self.incr
        if self.q2+self.incr >= pi:
            self.dir2 = 0
        elif self.q2-self.incr < 0:
            self.dir2 = 1

        # Pubblicazione sul topic
        msg.position = [self.q1, self.q2]
        self.publisher.publish(msg)
        self.get_logger().info(f"ANGOLO DEL BRACCIO: {msg}")

def main(args=None):
    rclpy.init(args=args)
    
    angle_publisher = AnglePublisher()

    rclpy.spin(angle_publisher)

    angle_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()