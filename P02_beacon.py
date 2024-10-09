import asyncio
import machine
import math
import neopixel
import network
import random
import secrets
import time
import MQTT
import struct

MAX_U16 = (1 << 16) - 1 # Maximum 16-bit unsigned value
MAX_U8 = (1 << 8) - 1   # Maximum 8-bit unsigned value

class Command:
    """
    A simple class to represent the states of MQTT commands.
    It has three states: START, STOP, and OTHER.
    """
    START = 'on'
    STOP = 'off'
    BUZZ = 'buzz'
    PAIR_ON = 'pair-on'
    PAIR_OFF = 'pair-off'
    OTHER = 'other'

class NetworkManager:
    """
    Manages Wi-Fi connection and MQTT communication.
    Handles subscribing to topics and processing incoming messages.
    """
    def __init__(self,indicator=None):
        """
        Initialize the NetworkManager.
        Sets up Wi-Fi, MQTT client, and default callback to OTHER
        """
        self.wlan = network.WLAN(network.STA_IF)

        # TODO;daniel; what is going on with my personal broker stuff? Should I just use this lmaoo
        # like i can't transfer the stuff in the example code because i don't know what micropython can even take
        self.indicator = indicator
        self.using_mqtt = False
        self.using_internet = False
        self.message_used = True
        self.mqtt_broker = 'broker.hivemq.com'
        self.user = 'daniel'
        self.password = '<PASSWORD>'
        self.port = 1883
        self.topic_sub = 'ME35-24/minecraft'
        # self.topic_pub = 'ME35-24/tell'
        self.client_id = 'ME35_not_chris'
        self.client = MQTT.MQTTClient('ME35_not_chris',self.mqtt_broker, self.port, keepalive=60)

        self.curr_callback = Command.OTHER

    def get_status_message(self):
        status_codes = {
            self.wlan.STAT_IDLE: 'Idle',
            self.wlan.STAT_CONNECTING: 'Connecting...',
            self.wlan.STAT_WRONG_PASSWORD: 'Wrong Password',
            self.wlan.STAT_NO_AP_FOUND: 'No Access Point Found',
            self.wlan.STAT_CONNECT_FAIL: 'Connection Failed',
            self.wlan.STAT_GOT_IP: 'Connected',
        }
        return status_codes.get(self.wlan.status(), 'Unknown Status')

    def connect_to_internet(self):
        """
        Connects to the Wi-Fi using credentials from the secrets file.
        Waits for connection up to 10 seconds, then returns success/failure.
        :return: True if connected, False otherwise
        """
        self.wlan.active(True)
        ret = False
        if self.wlan.isconnected() and True:
            print(f'Network connected, IP {self.wlan.ifconfig()[0]}')
            self.using_internet = True
            ret = True
        else:
            self.wlan.connect(secrets.mysecrets['SSID'], secrets.mysecrets['key'])
            print('Connecting to network...')

            max_wait = 10  # 10-second timeout
            while max_wait > 0:
                # status_message = self.get_status_message()
                # print(status_message)

                if self.wlan.isconnected():
                    print(f'Network connected, IP {self.wlan.ifconfig()[0]}')
                    self.using_internet = True
                    ret = True

                time.sleep(1)
                max_wait -= 1
        if ret is False:
            print('Failed to connect to the network.')
        self.update_network_status()
        return ret

    def callback(self, topic, msg):
        """
        Callback function for MQTT.
        Decodes the message and updates the command (START, STOP, OTHER).
        :param topic: The topic of the MQTT message
        :param msg: The message payload
        """
        print(topic.decode(), msg.decode())
        msg = msg.decode()

        command_map = {
            Command.START: Command.START,
            Command.STOP: Command.STOP,
            Command.BUZZ: Command.BUZZ,
            Command.PAIR_ON: Command.PAIR_ON,
            Command.PAIR_OFF: Command.PAIR_OFF,
        }

        self.curr_callback = command_map.get(msg, Command.OTHER)
        self.message_used = False

    def connect_to_MQTT(self):
        """
        Connects to the MQTT broker and subscribes to a specified topic.
        Also sets the MQTT callback to the NetworkManager's callback function.
        """
        try:
            self.client.connect()
            print(f'Connected to {self.mqtt_broker}s MQTT')
            self.client.set_callback(self.callback)
            self.client.subscribe(self.topic_sub.encode())
            self.using_mqtt = True
        except Exception as e:
            print("Didn't connect to MQTT")
            asyncio.sleep(1)

        self.update_network_status()

    def check_callback(self):
            """
            Checks for incoming MQTT messages and processes the callback.
            :return: The current command (START, STOP, OTHER)
            """
            if self.using_mqtt:
                try:
                    self.client.check_msg()
                    if self.message_used is False:
                        self.message_used = True
                        return self.curr_callback
                except Exception as e:
                    print('MQTT callback failed')
                    self.using_mqtt = False
                    self.update_network_status()
                    self.connect_to_MQTT()
                    return Command.OTHER

    def update_network_status(self):
        """
        Updates the status of the network by controlling the indicator LED based on the internet and MQTT connection states.
        """
        if not self.using_internet:
            self.indicator.off()
        elif not self.using_mqtt:
            self.indicator.set_brightness(0.02)
        else:
            self.indicator.on()

    def publish_message(self, message):
        """
        Publishes a message to the MQTT broker on the subscribed topic.
        """
        self.client.publish(self.topic_sub.encode(), message.encode())

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
        self.curr_time = time.time_ns()

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
        # print(f"Is buzzing: {self.is_buzzing} and {(time.time_ns() - self.start_time)/(1e9)}")
        self.curr_time = time.time_ns()
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
    def __init__(self, pin_num=20):
        """
        Initializes the button with the specified pin and sets the initial state.
        """
        self.button = machine.Pin(pin_num, machine.Pin.IN, machine.Pin.PULL_UP)
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

    def is_being_pressed(self):
        return self.is_pressed()

class LEDManager:
    """
    Manages LED brightness using PWM.
    Controls the brightness in a sinusoidal pattern over time.
    """
    def __init__(self, pin_num=0):
        """
        Initializes the LED with a default frequency and duty cycle of zero.
        """
        self.loop_time = 2 # seconds
        self.f = machine.PWM(machine.Pin('GPIO' + str(pin_num), machine.Pin.OUT))
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

    def on(self):
        self.f.duty_u16(int(MAX_U16))

    def set_brightness(self, brightness):
        if brightness < 0 or brightness > 1:
            print('Brightness out of range: Only values between 0 and 1 valid')
        else:
            self.f.duty_u16(int(MAX_U16 * brightness))

    # def

class AccelerometerManager:
    def __init__(self, scl, sda, addr = 0x62):
        """
        Initializes the accelerometer with I2C communication and sets up parameters for shaking detection.
        """
        self.addr = addr
        self.i2c = machine.I2C(1,scl=scl, sda=sda, freq=100000)
        self.connected = False
        self.prev_shaking = False  # To track the previous shaking state
        self.last_shake_time = 0
        self.shake_interval = 0.5 * 1e9 # in nanoseconds

        if self.is_connected():
            print('connected')
            self.write_byte(0x11,0) #start data stream

    def is_connected(self):
        """
        Checks if the accelerometer is connected via I2C and returns the connection status.
        """
        options = self.i2c.scan()
        print(options)
        self.connected = self.addr in options
        return self.connected

    def read_accel_mag(self):
        """
        Reads the acceleration vector from the accelerometer and returns its magnitude.
        """
        vector = self.read_accel()
        return math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)

    def is_shaking(self):
        """
        Determines if the accelerometer is detecting shaking based on a predefined magnitude threshold.
        """
        return self.read_accel_mag() > 20000 # Value obtained from testing

    def is_shooketh(self):
        """
        Detects the transition from not shaking to shaking.
        Returns True only once per shaking event, and False otherwise.
        """
        current_time = time.time_ns()
        shaking = self.is_shaking()
        # print(f"Shaking: {shaking}, Interval: {(current_time - self.last_shake_time)/(1e9)}")
        if shaking:
            if (current_time - self.last_shake_time) >= self.shake_interval:
                self.last_shake_time = current_time
                return True
            self.last_shake_time = current_time
        return False

    def read_accel(self):
        """
        Reads the raw acceleration data from the accelerometer and returns it as a vector.
        """
        buffer = self.i2c.readfrom_mem(self.addr, 0x02, 6) # read 6 bytes starting at memory address 2
        return struct.unpack('<hhh',buffer)

    def write_byte(self, cmd, value):
        """
        Writes a byte to a specific register on the accelerometer via I2C.
        """
        self.i2c.writeto_mem(self.addr, cmd, value.to_bytes(1,'little'))

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
        self.led_mode_inverted = False
        self.is_paired = False

        scl_pin = machine.Pin('GPIO27', machine.Pin.OUT)
        sda_pin = machine.Pin('GPIO26', machine.Pin.OUT)

        self.beacon_led = LEDManager(15)
        self.network_led = LEDManager(21)
        self.neopixel_manager = NeoPixelManager()
        self.button_manager = ButtonManager(16)
        self.buzzer_manager = BuzzerManager()
        self.accelerometer_manager = AccelerometerManager(scl_pin, sda_pin)
        self.network_manager = NetworkManager(indicator=self.network_led)

        self.network_led.off()
        self.beacon_led.off()

        self.network_manager.connect_to_internet()
        self.network_manager.connect_to_MQTT()

        self.run_async()

    def run_async(self):
        """
        Starts the asynchronous event loop to manage the system’s main tasks (callbacks, shaking detection, etc.).
        """
        thread = asyncio.get_event_loop()
        thread.create_task(self.check_callback())
        thread.create_task(self.shaking_loop())
        thread.create_task(self.update_beacon())
        thread.create_task(self.button())
        thread.run_forever()

    async def button(self):
        """
        Monitors the button state and toggles the LED mode when pressed, also publishes a message via MQTT.
        """
        while True:
            await asyncio.sleep(0.1)
            if self.running:
                if self.button_manager.is_pressed():
                    self.led_mode_inverted = not self.led_mode_inverted
                    self.network_manager.publish_message('something')

    async def check_callback(self):
        """
        Continuously checks for incoming MQTT messages using the network manager's callback.
        Based on the received command:
        - STOP: Stops the system.
        - START: Starts the system.
        - PAIR_ON: Enables pairing mode.
        - PAIR_OFF: Disables pairing mode.
        - BUZZ: Activates the buzzer.
        Runs in an infinite loop with a 0.1-second delay between checks.
        """
        while True:
            await asyncio.sleep(0.1)
            callback = self.network_manager.check_callback()
            if callback == Command.STOP:
                self.stop()
            elif callback == Command.START:
                self.start()
            elif callback == Command.PAIR_ON:
                self.is_paired = True
            elif callback == Command.PAIR_OFF:
                self.is_paired = False
            elif callback == Command.BUZZ:
                self.buzz()

    def buzz(self):
        """
        Activates the buzzer to start buzzing.
        """
        self.buzzer_manager.start_buzzing()


    async def shaking_loop(self):
        """
        Continuously checks for shaking and changes the NeoPixel color while buzzing when shaking is detected.
        """
        while True:
            await asyncio.sleep(0.01)
            if self.running:
                self.buzzer_manager.check_buzzing()
                if self.accelerometer_manager.is_shooketh():
                    self.neopixel_manager.random_color()
                    self.buzzer_manager.start_buzzing()

    async def update_beacon(self):
        """
        Updates the brightness of the beacon LED based on accelerometer readings, adjusting brightness according to movement.
        """
        prev_accel_val = self.accelerometer_manager.read_accel_mag()
        while True:
            await asyncio.sleep(0.01)
            if self.running:
                curr_accel_val = self.accelerometer_manager.read_accel_mag()
                using_accel_val = max(curr_accel_val, prev_accel_val - (prev_accel_val - 20000) * 0.01)
                if not self.led_mode_inverted:
                    brightness = max(min(1.0, -((1 / (40000 - 20000)) * (using_accel_val - 20000)) + 1), 0.0)
                else:
                    brightness = min(max(0.0, ((1 / (40000 - 20000)) * (using_accel_val - 20000))), 1.0)
                self.beacon_led.set_brightness(brightness)
                prev_accel_val = using_accel_val

    def stop(self):
        """
        Stops the system, turning off all components and halting the main loop.
        """
        self.running = False
        self.beacon_led.off()
        self.network_led.off()
        self.neopixel_manager.off()
        self.buzzer_manager.stop_buzzing()

    def start(self):
        """
        Starts the system, resuming the main loop and reapplying LED colors.
        """
        self.running = True
        self.neopixel_manager.set_curr_color()

manager = MainManager()
