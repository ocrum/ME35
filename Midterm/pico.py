import machine
import asyncio
from network import NetworkManager
from ultrasonic import UltrasonicManager
from midi import MIDIManager
import random

class LightSensorManager:
    def __init__(self, pin):
        self.pin = pin

    def read_light(self):
        return self.pin.read_u16()

    def is_dark(self):
        # return False
        return self.read_light() < 10000

class MainManager:
    def __init__(self):
        self.running = True
        self.is_crazy = False

        trig_pin_1 = machine.Pin('GPIO21', machine.Pin.OUT)
        echo_pin_1 = machine.Pin('GPIO20', machine.Pin.IN)

        trig_pin_2 = machine.Pin('GPIO17', machine.Pin.OUT)
        echo_pin_2 = machine.Pin('GPIO16', machine.Pin.IN)

        self.network = NetworkManager(pub='ME35-24/theremin_out', sub='ME35-24/theremin', client_id='daniel_pico')
        self.network.connect_to_internet()
        self.network.connect_to_MQTT()
        self.us_vol = UltrasonicManager(trig_pin_1, echo_pin_1)
        self.us_pitch = UltrasonicManager(trig_pin_2, echo_pin_2)
        self.light = LightSensorManager(machine.ADC(2))
        self.midi = MIDIManager(use_bluetooth=True)

        self.max_vol_dist = 100
        self.max_pitch_dist = 100

        self.run()

    def run(self):
        """
        Starts the asynchronous event loop to manage the system’s main tasks (callbacks, shaking detection, etc.).
        """
        # print("starting loop")
        thread = asyncio.get_event_loop()
        thread.create_task(self.check_callback())
        thread.create_task(self.instrument_loop())
        thread.run_forever()

    async def check_callback(self):
        """
        Continuously checks for incoming MQTT messages using the network manager's callback.
        """
        while True:
            await asyncio.sleep(0.1)
            callback = self.network.check_callback()
            if callback == 'stop':
                self.stop()
            elif callback == 'start':
                self.start()
            elif callback == 'none':
                self.is_crazy = False
            elif callback == 'surprise':
                self.is_crazy = True

    async def instrument_loop(self):
        while True:
            await asyncio.sleep(0.1)
            if self.running and not self.light.is_dark():
                vol = min(self.us_vol.read_distance(), self.max_vol_dist)/self.max_vol_dist
                pitch = min(self.us_pitch.read_distance(),self.max_pitch_dist)/self.max_pitch_dist

                # If crazy mode is on, add random variance to the pitch
                if self.is_crazy:
                    variance = random.uniform(-0.3, 0.3)  # Adjust variance range as needed
                    pitch += variance

                    # Ensure pitch stays within valid bounds (0 to 1)
                    pitch = max(0, min(1, pitch))

                print(f"Vol: {vol:.2f} cm", end=" ")
                print(f"Pitch: {pitch:.2f} cm")

                note, velocity = self.midi.play_monophonic_note(vol, pitch)
                self.network.publish_message(f"output note: {note}, vel: {velocity}")
            else:
                print(f"Light sensor pause: {self.light.read_light()}")
                self.midi.stop_all()

    def stop(self):
        """
        Stops the system, turning off all components and halting the main loop.
        """
        self.running = False
        self.midi.stop_all()

    def start(self):
        """
        Starts the system
        """
        self.running = True

MainManager()
