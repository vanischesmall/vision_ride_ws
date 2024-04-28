import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .submodules.ODriveV36.ODriveAPI import ODriveAPI


class ODriver(Node): # Subscriber to <odriver_input> 
    def __init__(self) -> object:
        super().__init__('odriver')
        self.odriver_input = self.create_subscription(
            String, 
            'odriver_input', 
            self.parse_odriver_input,
            1)
        self.odriver_output = self.create_publisher(
            String,
            'odriver_output_vel',
            1)
        self.timer = self.create_timer(0.01, self.publish_odriver_vel)
        
        self.odrive = ODriveAPI(invertM0=True)
        self.get_logger().info('ODrive connected!')

    def parse_odriver_input(self, msg) -> None:
        cmd, val_l, val_r = msg.data.split()

        match cmd:
            case 'brake':
                self.odrive.brake()

            case 'set_vel':
                self.odrive.set_vel(0, float(val_l))
                self.odrive.set_vel(1, float(val_r))

            case 'stop':
                self.odrive.stop()

    def publish_odriver_vel(self) -> None:
        msg = String()
        msg.data = f'M0:{self.odrive.vel0}\nM1:{self.odrive.vel1}'
        self.odriver_output.publish(msg)

def main():
    rclpy.init()

    odriver = ODriver()
    try:
        rclpy.spin(odriver)
    except:
        odriver.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()
