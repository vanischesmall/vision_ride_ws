from serial import Serial
JoystickPkg = tuple[bool, int, int, int]

class JoystickAPI:
    def __init__(self, 
                 port: str = '/dev/ttyUSB0',
                 raw: bool = False,
                 ) -> object:
        self.port = port
        self.raw = raw
        self.is_connected = False

        try:
            self.nano = Serial(self.port, 115200)
            self.is_connected = True
        except:
            self.nano = None

        self.button: bool = None
        self.rotate: int  = None
        self.joystX: int  = None
        self.joystY: int  = None

    def read(self) -> JoystickPkg:
        if self.nano is None or not self.nano.is_open:
            self.is_connected = False
            
            self.button = 0
            self.rotate = 0
            self.joystX = 0
            self.joystY = 0
            return self._return_values() 

        self.is_connected = True

        pkg = self.nano.read_until(b'!').decode('utf-8')
        self.button = int(pkg[0])
        self.rotate = int(pkg[1:4]) - 200
        self.joystX = int(pkg[4:7]) - 200
        self.joystY = int(pkg[7:10]) - 200
        
        return self._return_values()

    def _return_values(self) -> JoystickPkg:
        if self.raw:
            self.rotate += 200
            self.joystX += 200
            self.joystY += 200

        return self.button, \
               self.rotate, \
               self.joystX, \
               self.joystY


if __name__ == "__main__":
    jst = JstAPI('/dev/ttyUSB0')
    
    while True:
        jst.read()

        if jst.is_connected:
            print("Button ->", jst.button)
            print("Rotate ->", jst.rotate)
            print("JoystX ->", jst.joystX)
            print("JoystY ->", jst.joystY)
            print()
