import machine
import time
import utime

class UltrasonicManager:
    def __init__(self, trig_pin, echo_pin, timeout_us=1000000):
        """
        Initializes the ultrasonic sensor with specified trigger and echo pins.
        """
        self.trig = machine.Pin(trig_pin, machine.Pin.OUT)
        self.echo = machine.Pin(echo_pin, machine.Pin.IN)
        self.timeout_us = timeout_us  # Timeout in microseconds for waiting for echo

        # Ensure trigger is low at the start
        self.trig.value(0)
        time.sleep_us(5)  # Small delay to stabilize the sensor


    def read_distance(self):
        """
        Reads the distance from the ultrasonic sensor in centimeters.
        Implements a timeout to avoid infinite waits.
        """
        # Send trigger pulse
        self.trig.low()
        utime.sleep_us(2)
        self.trig.high()
        utime.sleep_us(10)
        self.trig.low()

        # Wait for echo to go high, with timeout
        start_time = utime.ticks_us()
        while self.echo.value() == 0:
            if utime.ticks_diff(utime.ticks_us(), start_time) > self.timeout_us:
                return None  # Timeout, no signal detected

        # Record time when echo goes high
        signal_on = utime.ticks_us()

        # Wait for echo to go low, with timeout
        while self.echo.value() == 1:
            if utime.ticks_diff(utime.ticks_us(), signal_on) > self.timeout_us:
                return None  # Timeout, signal too long

        # Record time when echo goes low
        signal_off = utime.ticks_us()

        # Calculate the time passed and convert to distance
        time_passed = utime.ticks_diff(signal_off, signal_on)
        distance_cm = (time_passed * 0.0343) / 2  # Convert to distance in cm
        return distance_cm
