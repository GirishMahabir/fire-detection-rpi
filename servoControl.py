from gpiozero import Servo
from time import sleep


class ServoControl:
    def __init__(self, pin, min, max):
        self.servo = Servo(pin, min_pulse_width=min, max_pulse_width=max)
    
    def right(self):
        self.servo.min()
    
    def left(self):
        self.servo.max()
    
    def center(self):
        self.servo.mid()
    
    def stop(self):
        self.servo.detach()
        self.servo.close()
    
if __name__ == '__main__':
    maxPW = (1.3+0.45)/1000 
    minPW = (1.0-0.45)/1000 # increases lowers my left limit.

    servo = ServoControl(16, minPW, maxPW)
    servo.center()
    sleep(3)
    servo.right()
    sleep(2)
    servo.center()
    sleep(3)
    servo.left()
    sleep(2)
    servo.center()
    sleep(3)
    servo.stop()