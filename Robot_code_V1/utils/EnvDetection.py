from utils.settings import ECHO_PIN, MAX_DIST, SERVO_MAX_PULSE_WIDTH, SERVO_MIN_PULSE_WIDTH, SERVO_PIN, TRIG_PIN
from gpiozero import Servo, AngularServo, DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
from typing import Tuple, List

class ServoMain(AngularServo):
    def __init__(self, initial_angle, min_angle, max_angle, frame_width):
        factory = PiGPIOFactory()
        super().__init__(pin=SERVO_PIN, initial_angle=initial_angle, min_angle=min_angle, max_angle=max_angle, min_pulse_width=SERVO_MIN_PULSE_WIDTH, max_pulse_width=SERVO_MAX_PULSE_WIDTH, frame_width=frame_width, pin_factory=factory)

class ServoJump(ServoMain):
    def __init__(self, initial_angle, min_angle, max_angle, frame_width):
        super().__init__(initial_angle, min_angle, max_angle, frame_width)
    
    def move_to_angle(self, angle:float) -> None:
        assert -90 <= angle <= 90, "The angle must be between -90 degrees and 90 degrees."
        self.angle = angle


class DistanceSensorMain(DistanceSensor):
    def __init__(self, queue_len, threshold_distance, partial):
        factory = PiGPIOFactory()
        super().__init__(echo=ECHO_PIN, trigger=TRIG_PIN, queue_len=queue_len, max_distance=MAX_DIST, threshold_distance=threshold_distance, partial=partial, pin_factory=factory)

class EnvDetection:
    def __init__(self, servo=ServoJump(), dist_sensor=DistanceSensorMain()) -> None:
        if not isinstance(servo, AngularServo):
            raise ValueError(
                "The servo object must be an instance of the gpiozero.AngularServo class"
            )
        if not isinstance(dist_sensor, DistanceSensor):
            raise ValueError(
                "The distance sensor object must be an instance of the gpiozero.DistanceSensor class"
            )
        self.servo = servo
        self.dist_sensor = dist_sensor
    
    def sweep(
            self,
            start_angle=None, 
            end_angle=None, 
            stops_num=10, 
            period_per_stop = 0.5
    ) -> List[Tuple[float, float]]:
        
        distance_data = List[Tuple[float, float]]
        
        if start_angle == None:
            start_angle = self.servo.min_angle
        if end_angle == None:
            end_angle = self.servo.max_angle
        
        delta_angle = (end_angle-start_angle)/(stops_num-1)

        for i_stop in range(stops_num):
            self.servo.angle = start_angle + delta_angle*i_stop
            sleep(period_per_stop)
            measured_dist = self.dist_sensor.distance
            distance_data.append((self.servo.angle, measured_dist))
        
        return distance_data


