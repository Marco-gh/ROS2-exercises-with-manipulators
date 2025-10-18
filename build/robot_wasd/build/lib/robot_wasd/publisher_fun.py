import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from math import pi

import sys, select, termios, tty

class AnglesPublisher(Node):
    def __init__(self):
        super().__init__('angles_publisher')

        # Restituisce e salva gli attributi del terminale correnti,
        # così puoi ripristinarli dopo modifiche.
        # Con "ros2 launch..." non viene associato un vero e proprio tty al processo del nodo
        # che rimane in background -> va lanciato con con "ros2 run..."
        if sys.stdin.isatty():
            fd = sys.stdin.fileno()
            try:
                self.settings = termios.tcgetattr(fd)
                self.create_timer(0.1,self.loop_keybrd)
            except termios.error as e:
                self.get_logger().error(f"tcgetattr failed: {e}")
                self.settings = None
        else:
            self.get_logger().error("No TTY detected: skipping keyboard input setup.")
            self.settings = None
            # Nel caso non fosse possibile stare in ascolto si usa il teleop di turtlesim (caso in cui viene lanciato con "ros2 launch...")
            self.command_listener = self.create_subscription(Twist,'/turtle1/cmd_vel',self.cmd_listener_callback,10)

        self.publisher = self.create_publisher(JointState,'joint_states',10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.ang_callback)

        self.joint1 = 0.0
        self.joint2 = 0.0

    def loop_keybrd(self):
        if sys.stdin.isatty() and self.settings is not None:
            key = get_key(self.settings,0.1)
            self.get_logger().info(f"KEY: {repr(key)}")

            bound = pi/2
            incr = pi/6
            
            if key == 'w' and self.joint1<=bound:
                self.joint1 += incr
            elif key == 's' and self.joint1>=-bound:
                self.joint1 -= incr
            if key == 'a' and self.joint2<=bound:
                self.joint2 += incr
            elif key == 'd' and self.joint2>=-bound:
                self.joint2 -= incr

            # Pubblico sul secondo topic
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['joint1','joint2']
            msg.position = [self.joint1, self.joint2]
            self.publisher.publish(msg)
    
    def cmd_listener_callback(self, msg: Twist):
        # self.get_logger().info(
        #     f"lin=({msg.linear.x:.2f},{msg.linear.y:.2f},{msg.linear.z:.2f}) "
        #     f"ang=({msg.angular.x:.2f},{msg.angular.y:.2f},{msg.angular.z:.2f})"
        # )

        # UP-DOWN to increment/decrement q1, upper and lower bound: pi
        bound = pi/2
        incr = pi/6
        if msg.linear.x > 0 and self.joint1<=bound:
            self.joint1 += incr
        elif msg.linear.x < 0 and self.joint1>=-bound:
            self.joint1 -= incr
        # RHIGHT/LEFT to increment/decrement q2
        if msg.angular.z > 0 and self.joint2<=bound:
            self.joint2 += incr
        elif msg.angular.z < 0 and self.joint2>=-bound:
            self.joint2 -= incr

        # Pubblico sul secondo topic
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1','joint2']
        msg.position = [self.joint1, self.joint2]
        self.publisher.publish(msg)

        # Print direct kinematic on a plane
        # x = 0.5*cos(self.joint1) + 0.5*cos(self.joint1+self.joint2)
        # y = 0.5*sin(self.joint1) + 0.5*sin(self.joint1+self.joint2)
        # self.get_logger().info(f"X: {x}, Y: {y}")


    def ang_callback(self):
        return 0

# Funzione che opera a più basso livello e permette di leggere i 
# comandi da terminale (vedere in turtlesim teleop_twist_keyboard.py)
def get_key(settings,timeout):
    # Imposta il terminale in modalità raw sullo 
    # stdin del processo corrente. Canonicamente
    # i comandi arrivano dopo l'invio e vengono 
    # interpretati
    tty.setraw(sys.stdin.fileno())
    # Serve ad "avvisarti" quando c'è un input da tastiera in
    # modo non bloccante (aspettiamo sullo stdin del processo)
    # È un'intfaccia che semplifica l'accesso alla chiamata di
    # sistema "select()", permette di esaminare lo stato dei
    # descrittori di file dei canali di IN/OUT aperti
    try:
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''
    finally:
        if settings is not None:
            # Imposta gli attributi del terminale. In pratica applica
            # la configurazione settings al TTY.
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    

def main(args=None):
    rclpy.init(args=args)
    
    angle_publisher = AnglesPublisher()

    try:
        rclpy.spin(angle_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        angle_publisher.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, angle_publisher.settings)


if __name__ == '__main__':
    main()