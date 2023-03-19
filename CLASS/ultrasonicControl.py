import RPi.GPIO as GPIO
import time

class UltrasonicData:
    def __init__(self, echo, trigger):
        self.echo = echo
        self.trigger = trigger
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.setup(self.trigger, GPIO.OUT)

    
    def get_distance(self):
        # set Trigger to HIGH
        GPIO.output(self.trigger, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.trigger, False)
    
        StartTime = time.time()
        StopTime = time.time()
    
        # save StartTime
        while GPIO.input(self.echo) == 0:
            StartTime = time.time()
    
        # save time of arrival
        while GPIO.input(self.echo) == 1:
            StopTime = time.time()
    
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
    
        return distance
    
    def clean(self):
        GPIO.cleanup()

 
if __name__ == '__main__':
    #set GPIO Pins
    GPIO_TRIGGER = 21
    GPIO_ECHO = 20

    # Initialize class
    ultrasonic = UltrasonicData(GPIO_ECHO, GPIO_TRIGGER)

    try:
        while True:
            dist = ultrasonic.get_distance()
            print ("Measured Distance = %.1f cm" % dist)
            time.sleep(1)
    
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        ultrasonic.clean()
