#!/usr/bin/env python3
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SixDofStatePublisher(Node):
    def __init__(self):
        super().__init__('sixdof_state_publisher')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        # Nomi dei giunti: DEVONO combaciare con l'URDF
        self.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']

        # Fasi iniziali e velocità angolari (rad/s) per ciascun giunto
        self.phi = [0.0, 0.7, 1.4, 2.1, 2.8, 3.5]
        self.omega = [0.5, 0.4, 0.6, 0.8, 0.7, 0.9]

        # Ampiezze (rispetta i limiti del tuo URDF!)
        self.amp = [pi/4, pi/6, pi/5, pi/3, pi/3, pi/2]

        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0/30.0, self.on_timer)  # 30 Hz

    def on_timer(self):
        # tempo in secondi
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        # traiettorie semplici: q_i(t) = A_i * sin(omega_i * t + phi_i)
        q = [ self.amp[i] * sin(self.omega[i]*t + self.phi[i]) for i in range(6) ]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q
        # (velocità e sforzo possono essere omessi)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = SixDofStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
