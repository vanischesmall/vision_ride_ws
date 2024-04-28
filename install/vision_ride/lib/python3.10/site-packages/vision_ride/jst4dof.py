import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from .submodules.Joystick4DOF.jstAPI import JstAPI


class Jst4DOF(Node): # Publisher to <jst4dof_values>
    def __init__(self) -> object:
        super().__init__('jst4dof')
        self.publisher = self.create_publisher(String, 'jst4dof_values', 10)
        self.timer = self.create_timer(0.01, self.getNpub_values)
        
        self.jst = JstAPI(port = '/dev/ttyUSB0')

    def getNpub_values(self) -> None:
        msg, jst_pkg = String(), self.jst.read()
        
        msg.data = f'{jst_pkg[0]} {jst_pkg[1]} {jst_pkg[2]} {jst_pkg[3]}'
        self.publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    
    jst4dof = Jst4DOF()
    rclpy.spin(jst4dof)
    
    jst4dof.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 