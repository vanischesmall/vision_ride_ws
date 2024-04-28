from Joystick4DOF.API import JoystickAPI
from ODriveV36.API import ODriveAPI

from threading import Thread
from time import time, sleep


def main(): ...
def process(): ...

if __name__ == "__main__":
    jst = JoystickAPI()
    odriver = ODriveAPI(invertM0=True, accel_rate=10)

    odriver.start()

    m0_vel: float = 0 
    m1_vel: float = 0 
    fps_tmr: int = 0
    
    while True:
        try:
            jst.read()
            m0_vel = 0
            m1_vel = 0

            m0_vel = jst.joystY / 50 
            m0_vel = m0_vel + jst.joystX * 0.2 if m0_vel >= 0 else m0_vel - jst.joystX * 0.2
            m1_vel = jst.joystY / 50 
            m1_vel = m1_vel + jst.joystX * 0.2 if m1_vel <= 0 else m1_vel - jst.joystX * 0.2
                    
            odriver.m0.set_vel(m0_vel)
            odriver.m1.set_vel(m1_vel)
            print(m0_vel, m1_vel)


        except KeyboardInterrupt:
            odriver.stop()
            break
    odriver.stop()
