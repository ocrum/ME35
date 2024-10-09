import asyncio
import sensor
import time
import math
import machine

class DriveManager:
    def __init__(self, m1a, m1b, m2a, m2b):
        """
        Initializes the PWM control for two motors.
        m1a, m1b: Pins for motor 1 control.
        m2a, m2b: Pins for motor 2 control.
        """
        freq = 50
        self.m1a = machine.PWM("P" + str(m1a), freq=freq)
        self.m1b = machine.PWM("P" + str(m1b), freq=freq)
        self.m2a = machine.PWM("P" + str(m2a), freq=freq)
        self.m2b = machine.PWM("P" + str(m2b), freq=freq)

    def set_motor(self, motor_a, motor_b, speed, forward_min=0.0, backwards_min=0.0):
        """
        Sets PWM for motor_a and motor_b based on the desired speed.
        speed > 0: Forward, speed < 0: Reverse, speed = 0: Stop.
        forward_min, backwards_min: Minimum PWM for forward/reverse.
        """
        forward_min = abs(forward_min)
        backwards_min = abs(backwards_min)

        if speed > 0:
            speed = min((1 - forward_min) * speed + forward_min, 1)
            motor_a.duty_u16(int(65535 * abs(speed)))
            motor_b.duty_u16(0)
        elif speed < 0:
            speed = min((1 - backwards_min) * abs(speed) + backwards_min, 1)
            motor_a.duty_u16(0)
            motor_b.duty_u16(int(65535 * abs(speed)))
        else:
            motor_a.duty_u16(0)
            motor_b.duty_u16(0)

    def drive(self, speed, direction):
        """
        Controls the 2 mootrs given how much you want to move forwards/backwards and how much you want to move left and right
        For understanding how this works look at this desmos link: https://www.desmos.com/calculator/pw44wo3yfg
        drive(0, 0) -> no movement
        drive(1, 0) -> fast forward
        drive(-1, 0) -> fast backwards
        drive(0.5, 0) -> half fast forward
        drive(1, 1) -> turn in place right
        dirve(1, -1) -> turn in place left
        drive(1, 0.5) -> pivot right around the right wheel
        drive(1, 0.75) -> bit right, bit forward
        """
        direction_bias = 0.1
        direction -= direction_bias
        y = speed * math.cos(90 * direction)
        x = speed * math.sin(90 * direction)
        speed_r = y + x
        speed_l = y - x
        self.set_motor(self.m2a, self.m2b, speed_r, forward_min=0.2, backwards_min=0.3)  # Control motor 2 (right side)
        self.set_motor(self.m1a, self.m1b, speed_l, forward_min=0.2, backwards_min=0.3)  # Control motor 1 (left side)

class CameraManager:
    def __init__(self):
        """
        Initializes the camera sensor and sets camera parameters (focal length, center).
        """
        self._init_sensor()

        self.f_x = (2.8 / 3.984) * 160
        self.f_y = (2.8 / 2.952) * 120
        self.c_x = 160 * 0.5
        self.c_y = 120 * 0.5

        self.clock = time.clock()

    def _init_sensor(self):
        """
        Resets and configures the camera sensor (flip, format, framesize).
        """
        sensor.reset()
        sensor.set_vflip(True)   # Flip the image vertically
        sensor.set_hmirror(True) # Flip the image horizontally
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QQVGA)
        sensor.skip_frames(time=2000)
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)

    def get_april_tag(self):
        """
        Captures an image and finds the closest AprilTag, returning its translation and rotation.
        """
        self.clock.tick()
        img = sensor.snapshot()

        # Get the closest tag
        closest_tag = None
        min_distance = float('inf')
        for tag in img.find_apriltags(fx=self.f_x, fy=self.f_y, cx=self.c_x, cy=self.c_y):
            if tag.z_translation < min_distance:
                closest_tag = tag
                min_distance = tag.z_translation

        tag_data = None
        if closest_tag is not None:
            img.draw_rectangle(closest_tag.rect, color=(255, 0, 0))
            img.draw_cross(closest_tag.cx, closest_tag.cy, color=(0, 255, 0))
            # Gather tag data
            tag_data = {
                'Tx': closest_tag.x_translation,
                'Ty': closest_tag.y_translation,
                'Tz': closest_tag.z_translation,
                'Rx': math.degrees(closest_tag.x_rotation),
                'Ry': math.degrees(closest_tag.y_rotation),
                'Rz': math.degrees(closest_tag.z_rotation),
            }

        return tag_data


class MainManager:
    def __init__(self):
        """
        Initializes the main control loop with CameraManager and DriveManager.
        """
        self.camera_manager = CameraManager()
        self.drive_manager = DriveManager(9, 7, 3, 1)

        self.running = True

        self.run()

    def run(self):
        """
        Starts the asyncio loop and continuously tracks the AprilTag.
        """
        loop = asyncio.get_event_loop()
        loop.create_task(self.track_tag())
        loop.run_forever()

    async def track_tag(self):
        """
        Asynchronously tracks the AprilTag, adjusting motor control based on x and z errors.
        """
        previous_time = time.ticks_ms()
        prev_x_error = 0
        prev_z_error = 0
        goal_z = -5
        kx_p = -0.01
        kx_d = -0.001
        kz_p = -0.1
        kz_d = -0.01
        while True:
            await asyncio.sleep(0.01)
            current_time = time.ticks_ms()  # Get the current time in milliseconds
            elapsed_time = time.ticks_diff(current_time, previous_time) / 1000.0 # Calculate elapsed time

            if self.running:
                tag_data = self.camera_manager.get_april_tag()
                if tag_data is not None:
                    x_error = tag_data.get('Tx')
                    z_error = tag_data.get('Tz') - goal_z

                    print(f"z: {z_error}")

                    p_x = x_error * kx_p
                    p_z = z_error * kz_p

                    if elapsed_time > 0:
                        d_x = (x_error - prev_x_error) / elapsed_time * kx_d
                        d_z = (z_error - prev_z_error) / elapsed_time * kz_d
                    else:
                        d_x = 0
                        d_z = 0

                    self.drive_manager.drive(p_z + d_z, p_x + d_x)

                    prev_x_error = x_error
                    prev_z_error = z_error
                else:
                    self.drive_manager.drive(0, 0)
                    prev_x_error = 0
                    prev_z_error = 0

            previous_time = current_time

manager = MainManager()
