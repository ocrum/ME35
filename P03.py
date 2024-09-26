import time
import random
from Tufts_ble import Sniff, Yell
import asyncio
import neopixel
from machine import Pin, PWM

class ZombieGame():
    def __init__(self):
        self.player_type = 'H'  # Initialize as Human ('H') or Zombie ('Z')
        self.infection_string = '7'
        self.running = True
        self.discriminator = '!'
        self.rssi_thresh = -60
        self.num_teams = 13
        self.hit_array = [0] * self.num_teams  # Array that tracks the hits from the teams
        self.start_times = [0] * self.num_teams  # Stores the start time when a zombie enters range
        self.in_ranges = [False] * self.num_teams  # Stores whether each team is currently in range

        self.c = Sniff(self.discriminator, verbose=False)
        self.p = Yell()

        # Initialize Buzzer PWM on GPIO18
        self.buzzer_pin = Pin('GPIO18', Pin.OUT)
        self.buzzer = PWM(self.buzzer_pin)
        self.buzzer.duty_u16(0)  # Start with the buzzer off

        # Initialize NeoPixel on GPIO28 with 1 pixel
        self.np_pin = Pin(28, Pin.OUT)
        self.np = neopixel.NeoPixel(self.np_pin, 1)
        self.color = (0, 255, 0)  # Start with green
        self.set_neopixel_color(self.color)  # Initialize NeoPixel to green

        self.main()

    def set_neopixel_color(self, color):
        """
        Sets the NeoPixel color.

        :param color: Tuple representing the RGB color, e.g., (255, 0, 0) for red.
        """
        if self.color != color:
            self.color = color
            self.np[0] = color
            self.np.write()

    async def human(self):
        """
        In charge of running all human-related tasks.
        """
        tasks = asyncio.gather(
            self.central(),
            self.indicate_human(),
        )
        await tasks

    async def indicate_human(self):
        """
        Controls the NeoPixel to display colors based on zombie proximity.
        - Blue when any zombie is in range.
        - Green when no zombies are in range.
        """
        while self.running and self.player_type == 'H':
            if any(self.in_ranges):
                # At least one zombie is in range
                self.set_neopixel_color((0, 0, 255))  # Blue color
            else:
                # No zombies in range
                self.set_neopixel_color((0, 255, 0))  # Green color
            await asyncio.sleep(0.1)

        # Optionally, turn off NeoPixel when exiting human mode
        self.set_neopixel_color((0, 0, 0))  # Off

    async def central(self):
        """
        Handles scanning and processing incoming signals.
        """
        try:
            self.c.scan(0)   # 0ms = scans forever
            while self.running and self.player_type == 'H':
                # Get last message and RSSI
                curr_msg = self.c.last
                curr_rssi = self.c.last_rssi
                if curr_msg:  # If a new message is received
                    # Clear the flags for the next advertisement
                    self.c.last = ''
                    self.c.last_rssi = ''

                    # Process the hit
                    await self.hit_checker(curr_rssi, curr_msg)

                    await asyncio.sleep(0.1)
                await asyncio.sleep(0.01)
        finally:
            self.c.stop_scan()

    async def peripheral(self):
        """
        Handles advertising when the player is a zombie.
        """
        while self.running and self.player_type == 'Z':
            self.p.advertise(f'!{self.infection_string}')
            await asyncio.sleep(0.1)

        self.p.stop_advertising()

    async def indicate_zombie(self):
        """
        Generates zombie-like growling sounds and maintains red NeoPixel when a zombie.
        """
        try:
            self.set_neopixel_color((255, 0, 0))  # Red color

            while self.running and self.player_type == 'Z':
                # Frequency Modulation:
                # Random frequency between 80Hz and 150Hz for a low rumbling effect
                freq = random.randint(80, 150)
                self.buzzer.freq(freq)

                # Duty Cycle Variation:
                # Random duty cycle between 500 and 2000 out of 65535 for raspiness
                duty = random.randint(500, 2000)
                self.buzzer.duty_u16(duty)

                # Random Timing:
                # Wait for a short random period to make the sound less predictable
                sleep_time = random.uniform(0.05, 0.2)  # Sleep between 50ms to 200ms
                await asyncio.sleep(sleep_time)
        finally:
            # Turn off the buzzer when done
            self.buzzer.duty_u16(0)
            # Optionally, turn off NeoPixel when exiting zombie mode
            self.set_neopixel_color((0, 0, 0))  # Off

    async def zombie(self):
        """
        In charge of running all zombie-related tasks.
        """
        tasks = asyncio.gather(
            self.peripheral(),
            self.indicate_zombie()
        )
        await tasks

    async def hit_checker(self, curr_rssi, curr_msg):
        """
        Checks if a zombie is hitting the player based on RSSI and message.

        :param curr_rssi: The RSSI value of the received signal.
        :param curr_msg: The message received indicating a hit.
        """
        print(f"Calling hit_checker with RSSI: {curr_rssi} and message: {curr_msg}")
        try:
            team_number = int(curr_msg[1:])  # Shave off the first character (discriminator)
        except ValueError:
            print(f"Invalid message format: {curr_msg}")
            return

        if team_number < 0 or team_number >= self.num_teams:
            print(f"Invalid team number: {team_number}")
            return

        prev_in_range = self.in_ranges[team_number]
        curr_in_range = (int(curr_rssi) >= self.rssi_thresh)
        self.in_ranges[team_number] = curr_in_range

        time_since_start = (time.time() - self.start_times[team_number])

        print(f"Time since start: {time_since_start:.2f}s | In range: {curr_in_range}")
        # Check if the zombie/team has been in range within 3 seconds
        if not prev_in_range and curr_in_range:
            print("Starting hit timer")
            self.start_times[team_number] = time.time()  # Add a start time for that team
        elif (time_since_start <= 3) and curr_in_range:
            print("In range but not hit yet")
        elif (time_since_start <= 3) and not curr_in_range:
            print("Got out of range")
            self.start_times[team_number] = 0  # Reset start time
        elif time_since_start > 3 and curr_in_range:  # This is a hit!!
            self.hit_array[team_number] += 1  # Increment hits array at team's index
            self.start_times[team_number] = time.time()  # Reset start time
            print('Hit registered! Total hits from this team:', self.hit_array[team_number])
            await self.indicate_tag()
            await self.play_tag_sound()

        # Check if there have been 3 hits from one team/zombie
        if self.hit_array[team_number] >= 3 and self.player_type == 'H':
            self.player_type = 'Z'  # Turn into zombie
            self.infection_string = str(team_number)  # Set the infection string to the given code
            print('Transitioning from Human to Zombie!')
            # This will cause human tasks to stop and zombie tasks to start

    async def indicate_tag(self):
        """
        Provides visual feedback when a human is tagged.
        """
        # Flash NeoPixel yellow to indicate a tag
        self.set_neopixel_color((255, 255, 0))  # Yellow color
        await asyncio.sleep(0.5)
        # After flashing, set the color based on current state
        if self.player_type == 'H':
            if any(self.in_ranges):
                self.set_neopixel_color((0, 0, 255))  # Blue color
            else:
                self.set_neopixel_color((0, 255, 0))  # Green color

    async def play_tag_sound(self):
        """
        Provides audio feedback when a human is tagged.
        """
        # Simple beep sound
        self.buzzer.freq(1000)
        self.buzzer.duty_u16(3000)
        await asyncio.sleep(0.2)
        self.buzzer.duty_u16(0)

    def main(self):
        """
        Starts the game by running either human or zombie tasks based on the player type.
        """
        try:
            while self.running:
                if self.player_type == 'H':
                    asyncio.run(self.human())
                elif self.player_type == 'Z':
                    asyncio.run(self.zombie())
        except KeyboardInterrupt:
            # Handle the KeyboardInterrupt to stop sounds and turn off LEDs
            print("Program interrupted. Turning off buzzer and LED...")
            self.buzzer.duty_u16(0)       # Turn off the buzzer
            self.set_neopixel_color((0, 0, 0))  # Turn off the NeoPixel
            self.running = False          # Stop the main loop

            # Save the infection_string to a file
            try:
                with open("infection_string.txt", "w") as file:
                    file.write(self.infection_string)
                print(f"Infection string '{self.infection_string}' saved to infection_string.txt")
            except Exception as e:
                print(f"Error saving infection string: {e}")

game = ZombieGame()