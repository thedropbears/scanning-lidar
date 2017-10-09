import smbus
import threading
import time
import RPi.GPIO as GPIO
from collections import deque


class LIDARLite(threading.Thread):

    ADDR = 0x62
    I2C_BUS_NO = 1
    INTERRUPT_GPIO_PIN = 4

    def __init__(self):
        super().__init__()
        self.bus = smbus.SMBus(LIDARLite.I2C_BUS_NO)
        self.address = LIDARLite.ADDR
        self.since_bias_correction = 0  # used to perform a reciever bias correction every n measurements
        self.bias_correction_interval = 100
        self._terminate = threading.Event()

    def run(self):
        print("LIDAR Lite thread initialized on address %s" % (self.address))
        self.initialize()
        while True:
            if self._terminate.is_set():
                self.cleanup()
                break

            GPIO.wait_for_edge(LIDARLite.INTERRUPT_GPIO_PIN, GPIO.FALLING)  # blocks until we see a falling edge on the LIDAR
            tm = time.time()

            # actually read from the lidar
            dist_cm = self.read()
            print(dist_cm)

            self.measurement_buffer.append((tm, dist_cm/100))

            self.since_bias_correction += 1
            # every so often, we need to perform a bias correction routine
            if self.since_bias_correction >= self.bias_correction_interval:
                self.bias_correction_mode()
                self.since_bias_correction = -1
            elif self.since_bias_correction == 0:
                self.normal_mode()

    def initialize(self):

        # LIDAR Lite initialisation over I2C
        self.bus.write_byte_data(self.address, 0x11, 0xff)  # continuously repeat measure command
        self.bus.write_byte_data(self.address, 0x45, 0x03)  # command repeat interval - should be approximately 2000/Hz
        self.bus.write_byte_data(self.address, 0x04, 0x35)  # acquisition mode control register. set to interrupt and run fast
        self.bus.write_byte_data(self.address, 0x00, 0x04)  # initially make measurement with reciever bias correction
        self.bus.write_byte_data(self.address, 0x12, 0x03)  # number of acquisitions per measurement

        # Enable the interrupt pin for interrupt
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.INTERRUPT_GPIO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.last_tm = time.time()  # so we can capture the dt between interrupts from the lidar - for debugging purposes
        self.measurement_buffer = deque([], maxlen=1000)  # fills with a list of (tm, dist) tuples. deque to stop filling memory and crashing

    def read(self):
        """ Read the distance from the LIDAR's registers, in cm """
        high_byte = self.bus.read_byte_data(self.address, 0x0f)
        low_byte = self.bus.read_byte_data(self.address, 0x10)
        return (high_byte << 8) + low_byte

    def flush(self):
        """ Return the recent sensor measurements, and clear the measurement buffer """
        ret_list = list(self.measurement_buffer)
        self.measurement_buffer.clear()
        return ret_list

    def bias_correction_mode(self):
        self.bus.write_byte_data(self.address, 0x00, 0x04)  # set the LIDAR to perform a bias correction routine

    def normal_mode(self):
        self.bus.write_byte_data(self.address, 0x00, 0x03)  # set the LIDAR to just measure

    def terminate(self):
        self._terminate.set()

    def cleanup(self):
        print("Terminating LIDAR Lite read loop")
        GPIO.cleanup()
