import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import cos, sin
import numpy as np
import matplotlib.pyplot as plt

class AngleSubscriber(Node):
    def __init__(self):
        super().__init__('angle_subscriber')
        self.create_subscription(JointState, 'angle_topic', self.listener_callback, 10)

        self.q1 = 0.0; self.q2 = 0.0
        self.l1 = 1.5; self.l2 = 1.5

        # setup grafico UNA SOLA VOLTA
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal', adjustable='box')
        R = self.l1 + self.l2
        self.ax.set_xlim(-R, R); self.ax.set_ylim(-R, R)
        self.ax.set_autoscale_on(False)
        self.ax.grid(True)

        # artist riutilizzabili
        self.qv1 = self.ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='r')
        self.qv2 = self.ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='b')
        self.pt  = self.ax.scatter([0], [0], c='k')

        plt.ion() # Abilita la modalità interattiva e la possibilità di ridisegnare
        self.fig.show()

    def listener_callback(self, msg):
        if len(msg.position) > 0: self.q1 = float(msg.position[0])
        if len(msg.position) > 1: self.q2 = float(msg.position[1])

        V1 = np.array([self.l1*cos(self.q1), self.l1*sin(self.q1)])
        V2 = np.array([self.l2*cos(self.q1+self.q2), self.l2*sin(self.q1+self.q2)])
        O2 = V1
        EE = V1 + V2

        self.qv1.set_offsets([[0, 0]])
        self.qv1.set_UVC(V1[0], V1[1])

        self.qv2.set_offsets([O2])
        self.qv2.set_UVC(V2[0], V2[1])

        self.pt.set_offsets([EE])

        self.fig.canvas.draw_idle()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = AngleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
