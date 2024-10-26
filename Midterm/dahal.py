from machine import Pin, SoftI2C, PWM, ADC
import time
import servo
import ssd1306
from network import NetworkManager
import mqtt
import secrets
import re
import math

# MIDI note names for values 0-127
MIDI_NOTE_NAMES = [
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
]

def midi_pitch_to_note_name(pitch):
    """
    Converts a MIDI pitch value to a note name.
    """
    octave = max((pitch // 12) - 1,0)
    note = MIDI_NOTE_NAMES[pitch % 12]
    return f"{note}{octave}"

def volume_to_percentage(volume):
    """
    Converts a MIDI volume (0-127) to a percentage (0-100).
    """
    return (volume / 127) * 100

def calculate_std_dev(data):
    """
    Calculates the standard dev of the data in a simple manner without using the statistics library.
    """
    if len(data) < 2:
        return 0

    mean = sum(data) / len(data)
    squared_diffs = [(x - mean) ** 2 for x in data]
    std_dev = sum(squared_diffs) / len(data)
    return math.sqrt(std_dev)

class MainManager:
    def __init__(self):
        self.running = True

        self.network = NetworkManager(sub='ME35-24/theremin_out', client_id='daniel_dahal')
        self.network.connect_to_internet()
        self.network.connect_to_MQTT()

        # Initialize I2C for the OLED display
        self.i2c = SoftI2C(scl=Pin(7), sda=Pin(6))
        self.screen = ssd1306.SSD1306_I2C(128, 64, self.i2c)

        # Initialize the servo
        self.motor = servo.Servo(Pin(2))

        # To track the last 10 notes
        self.note_history = []
        self.curr_pitch = 0
        self.curr_volume = 0

    def run(self):
        while True:
            while self.running:
                msg = self.network.check_callback()

                if msg is not None:
                    match = re.search(r'output note: (\d+), vel: (\d+)', msg)
                    if match:
                        self.curr_pitch = int(match.group(1)) % 128
                        self.curr_volume = int(match.group(2)) % 128

                # Update note history and display pitch and volume
                self.update_note_history(self.curr_pitch)
                self.display_pitch_volume(self.curr_pitch, self.curr_volume)

                self.control_servo()

                # Small delay to observe changes more smoothly
                time.sleep(0.1)

    def display_pitch_volume(self, pitch, volume):
        # Convert pitch to note name and volume to percentage
        note_name = midi_pitch_to_note_name(int(pitch))
        volume_percent = volume_to_percentage(volume)

        # Clear screen and display formatted pitch and volume
        self.screen.fill(0)  # Clear the screen
        self.screen.text(f'Pitch: {note_name}', 0, 0, 1)
        self.screen.text(f'Volume: {volume_percent:.1f}%', 0, 16, 1)
        self.screen.show()

    def update_note_history(self, pitch):
        # Add the new pitch to the history
        self.note_history.append(pitch)
        # Keep only the last 10 notes
        if len(self.note_history) > 10:
            self.note_history.pop(0)

    def control_servo(self):
        dev = calculate_std_dev(self.note_history)

        max_dev = 50
        angle = 180 - min(max(dev / max_dev * 180, 0), 180)

        # Move the servo to the calculated angle
        self.motor.write_angle(int(angle))
        print(f"Angle: {angle:.1f} , Dev: {dev:.1f}")

# Instantiate and run the manager
manager = MainManager()
manager.run()