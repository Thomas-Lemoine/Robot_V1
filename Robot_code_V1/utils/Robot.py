
from EnvDetection import EnvDetection
from Wheels import Wheels

class Robot:
    def __init__(self) -> None:
        self.env_detection = EnvDetection()
        self.wheels = Wheels()
    