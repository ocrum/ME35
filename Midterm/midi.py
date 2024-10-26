from BLE_CEEO import Yell
import time
import math

class MIDIManager:
    def __init__(self, use_bluetooth=True):
        self.use_bluetooth = use_bluetooth
        if self.use_bluetooth:
            self.bt = Yell('Theremin', verbose = False, type = 'midi')
            self.bt.connect_up()

        self.NOTE_ON = 0x90
        self.NOTE_OFF = 0x80
        self.STOP_NOTES = 123
        self.RESET = 0xFF

        self.prev_note = 0

    def play_monophonic_note(self, volume, pitch):
        """
        TODO
        :param volume: Value 0 to 1 (0 silent, 1 max volume)
        :param pitch: Value 0 to 1 (0 lowest pitch note, 1 max pitch note)
        """
        self.stop_note(self.prev_note)
        self.prev_note = pitch
        return self.play_note(volume, pitch)

    def send_midi_message(self, cmd, pitch, velocity):
        """
        Helper function to send a MIDI message with the given command, pitch, and velocity.
        """
        timestamp_ms = time.ticks_ms()
        tsM = (timestamp_ms >> 7 & 0b111111) | 0x80
        tsL = 0x80 | (timestamp_ms & 0b1111111)

        channel = 0
        channel = 0x0F & channel
        c = cmd | channel

        note = math.floor(pitch * 128)
        velocity = math.floor(velocity * 128)

        payload = bytes([tsM, tsL, c, note, velocity])

        if self.use_bluetooth:
            self.bt.send(payload)

        return note, velocity

    def play_note(self, volume, pitch):
        """
        Plays a note with the specified volume and pitch.
        """
        return self.send_midi_message(self.NOTE_ON, pitch, volume)

    def stop_note(self, pitch):
        """
        Stops a note with the specified pitch.
        """
        self.send_midi_message(self.NOTE_OFF, pitch, 0)


    def stop_all(self):
        """
        Stops all currently playing notes by sending the 'All Notes Off' message (Control Change 123).
        """
        channel = 0  # Assuming channel 0, adjust if using other channels
        control_change_cmd = 0xB0 | (channel & 0x0F)  # CC message for channel 0
        control_number = self.STOP_NOTES  # Control Change 123 for All Notes Off
        control_value = 0  # Typically 0 for this message

        if self.use_bluetooth:
            # Send Control Change 123 for All Notes Off
            self.bt.send(bytes([control_change_cmd, control_number, control_value]))

    def disconnect(self):
        self.bt.disconnect()
