import machine
import math
import neopixel
import network
import random
import secrets
import time
import mqtt

MAX_U16 = (1 << 16) - 1 # Maximum 16-bit unsigned value
MAX_U8 = (1 << 8) - 1   # Maximum 8-bit unsigned value

class Command:
    """
    A simple class to represent the states of MQTT commands.
    It has three states: START, STOP, and OTHER.
    """
    START = 'start'
    STOP = 'stop'
    OTHER = 'other'

class NetworkManager:
    """
    Manages Wi-Fi connection and MQTT communication.
    Handles subscribing to topics and processing incoming messages.
    """
    def __init__(self):
        """
        Initialize the NetworkManager.
        Sets up Wi-Fi, MQTT client, and default callback to OTHER
        """
        self.wlan = network.WLAN(network.STA_IF)

        # TODO;daniel; what is going on with my personal broker stuff? Should I just use this lmaoo
        # like i can't transfer the stuff in the example code because i don't know what micropython can even take
        self.mqtt_broker = 'broker.hivemq.com'
        self.user = 'daniel'
        self.password = '<PASSWORD>'
        self.port = 1883
        self.topic_sub = 'ME35-24/nightlight'
        # self.topic_pub = 'ME35-24/tell'
        self.client_id = 'ME35_not_chris'
        self.client = mqtt.MQTTClient('ME35_not_chris',self.mqtt_broker, self.port, keepalive=60)

        self.curr_callback = Command.OTHER

    def connect_to_internet(self):
        """
        Connects to the Wi-Fi using credentials from the secrets file.
        Waits for connection up to 10 seconds, then returns success/failure.
        :return: True if connected, False otherwise
        """
        self.wlan.active(True)
        if not self.wlan.isconnected():
            print('Connecting to network...')
            self.wlan.connect(secrets.mysecrets['SSID'], secrets.mysecrets['key'])

            max_wait = 10 # 10 second timeout
            while not self.wlan.isconnected() and max_wait > 0:
                time.sleep(1)
                max_wait -= 1
                print('Waiting for connection...')

        if self.wlan.isconnected():
            print(f'Network connected, IP {self.wlan.ifconfig()[0]}')
            return True
        else:
            print('Failed to connect to the network.')
            return False

    def callback(self, topic, msg):
        """
        Callback function for MQTT.
        Decodes the message and updates the command (START, STOP, OTHER).
        :param topic: The topic of the MQTT message
        :param msg: The message payload
        """
        print(topic.decode(), msg.decode())
        msg = msg.decode()
        if msg == 'start':
            self.curr_callback = Command.START
        elif msg == 'stop':
            self.curr_callback = Command.STOP
        else:
            self.curr_callback = Command.OTHER

    def connect_to_MQTT(self):
        """
        Connects to the MQTT broker and subscribes to a specified topic.
        Also sets the MQTT callback to the NetworkManager's callback function.
        """
        self.client.connect()
        print(f'Connected to {self.mqtt_broker}s MQTT')
        self.client.set_callback(self.callback)
        self.client.subscribe(self.topic_sub.encode())

    def check_callback(self):
        """
        Checks for incoming MQTT messages and processes the callback.
        :return: The current command (START, STOP, OTHER)
        """
        self.client.check_msg()
        return self.curr_callback

class BuzzerManager:
    """
    Manages the PWM-based buzzer on the device.
    Handles starting, stopping, and checking the buzzer's state.
    """
    def __init__(self):
        """
        Initializes the BuzzerManager with default settings.
        Configures PWM frequency and initializes variables for buzzing duration.
        """
        self.f = machine.PWM(machine.Pin('GPIO18', machine.Pin.OUT))
        self.f.freq(440)
        self.start_time = 0
        self.is_buzzing = False
        self.time_duration = 0.5 * 10**9 # 0.5s in ns

    def start_buzzing(self):
        """
        Starts the buzzer with a preset duty cycle.
        Sets the time when the buzzer started buzzing.
        """
        self.is_buzzing = True
        self.start_time = time.time_ns()
        self.f.duty_u16(1000)

    def check_buzzing(self):
        """
        Checks if the buzzing duration has exceeded the limit.
        Stops the buzzer if the time limit is exceeded.
        """
        if self.is_buzzing and time.time_ns() - self.start_time > self.time_duration:
            self.stop_buzzing()

    def stop_buzzing(self):
        """
        Stops the buzzing by setting the duty cycle to zero.
        """
        self.is_buzzing = False
        self.f.duty_u16(0)

class NeoPixelManager:
    """
    Manages NeoPixel LED control.
    Handles setting and changing LED colors as well as turning it off.
    """
    def __init__(self):
        """
        Initializes the NeoPixel LED with the default off color.
        Sets the current LED color and applies it.
        """
        self.off_color = (0, 0, 0)
        self.color = self.off_color
        self.led = neopixel.NeoPixel(machine.Pin(28), 1)
        self.set_curr_color()

    def random_color(self):
        """
        Sets the NeoPixel to a random color and updates the LED.
        """
        self.color = (random.randint(0, MAX_U8), random.randint(0, MAX_U8), random.randint(0, MAX_U8))
        self.set_curr_color()

    def set_curr_color(self):
        """
        Applies the currently set color to the NeoPixel and updates the LED.
        """
        self.led[0] = self.color
        self.led.write()

    def off(self):
        """
        Turns off the NeoPixel by setting it to the off color.
        """
        self.led[0] = self.off_color
        self.led.write()

class ButtonManager:
    """
    Manages the button input.
    Detects button presses by comparing the current and previous states.
    """
    def __init__(self):
        """
        Initializes the button with the specified pin and sets the initial state.
        """
        self.button = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)
        self.prev_state = self.button.value()

    def is_pressed(self):
        """
        Checks if the button was pressed (transitions from unpressed to pressed).
        :return: True if the button is newly pressed, False otherwise
        """
        current_state = self.button.value()
        ret_val = False

        # New press when curr value is pressed (0) and prev is unpressed (1)
        if current_state == 0 and self.prev_state == 1:
            ret_val = True

        self.prev_state = self.button.value()
        return ret_val

class LEDManager:
    """
    Manages LED brightness using PWM.
    Controls the brightness in a sinusoidal pattern over time.
    """
    def __init__(self):
        """
        Initializes the LED with a default frequency and duty cycle of zero.
        """
        self.loop_time = 2 # seconds
        self.f = machine.PWM(machine.Pin('GPIO0', machine.Pin.OUT))
        self.f.freq(50)
        self.f.duty_u16(0)

    def updateBrightness(self, time_stamp):
        """
        Updates the LED brightness based on a sinusoidal wave pattern over time.
        :param time_stamp: The time in nanoseconds used to calculate the brightness
        """
        # convert time_stamp to seconds
        time_stamp = time_stamp / 10**9
        # This is a function that goes from 0 to 1 in a sinusoidal wave (one cycle per loop time)
        percent_brightness = (-math.cos((2 * math.pi / self.loop_time) * time_stamp) + 1) / 2
        self.f.duty_u16(int(MAX_U16 * percent_brightness))

    def off(self):
        """
        Turns off the LED by setting the duty cycle to zero.
        """
        self.f.duty_u16(0)

class MainManager:
    """
    Coordinates the entire system, managing the main loop and communication.
    Handles starting and stopping based on MQTT commands.
    """
    def __init__(self):
        """
        Initializes the MainManager by setting up the necessary components.
        Connects to Wi-Fi and MQTT.
        """
        self.running = True
        self.start_time = time.time_ns()
        self.elapsed_time = time.time_ns() - self.start_time

        self.led_manager = LEDManager()
        self.neopixel_manager = NeoPixelManager()
        self.button_manager = ButtonManager()
        self.buzzer_manager = BuzzerManager()
        self.network_manager = NetworkManager()

        self.network_manager.connect_to_internet()
        self.network_manager.connect_to_MQTT()

    def run(self):
        """
        Main loop of the system.
        Checks for MQTT commands, updates components (LED, NeoPixel, buzzer),
        and reacts to button presses.
        """
        while True:
            callback = self.network_manager.check_callback()

            if callback == Command.STOP:
                self.stop()
            elif callback == Command.START:
                self.start()

            if self.running:
                self.elapsed_time = time.time_ns() - self.start_time
                self.led_manager.updateBrightness(self.elapsed_time)
                self.buzzer_manager.check_buzzing()
                if self.button_manager.is_pressed():
                    self.neopixel_manager.random_color()
                    self.buzzer_manager.start_buzzing()
            time.sleep(0.01)

    def stop(self):
        """
        Stops the system, turning off all components and halting the main loop.
        """
        self.running = False
        self.led_manager.off()
        self.neopixel_manager.off()
        self.buzzer_manager.stop_buzzing()

    def start(self):
        """
        Starts the system, resuming the main loop and reapplying LED colors.
        """
        self.running = True
        self.neopixel_manager.set_curr_color()

manager = MainManager()
manager.run()
