class ServoManager:
    def __init__(self, pin_num=0):
        self.servo = machine.PWM("P" + str(pin_num), freq=50)
        self.max_degree = 180
        self.angle = 90
        self.set_angle(self.angle)

        self.use_velocity = True
        self.velocity = 0  # degrees / second

    @property
    def get_angle(self):
        return self.angle

    def set_angle(self, degrees):
        # Clamp degrees to be between 0 and 180
        if degrees < 0:
            degrees = 0
        elif degrees > 180:
            degrees = 180

        # Calculate the pulse width based on the clamped degrees
        pulse_width = 500 + (degrees / self.max_degree) * 2000  # Pulse width in microseconds
        self.servo.duty_ns(int(pulse_width * 1000))  # Convert microseconds to nanoseconds
        self.angle = degrees

    @property
    def get_velocity(self):
        return self.velocity

    def set_velocity(self, velocity):
        self.velocity = velocity

    async def velocity_manager(self):
        previous_time = time.ticks_ms()

        while True:
            await asyncio.sleep(0.01)  # Short sleep to yield control
            current_time = time.ticks_ms()  # Get the current time in milliseconds
            elapsed_time = time.ticks_diff(current_time, previous_time) / 1000.0

            if self.use_velocity:
                delta_angle = self.velocity * elapsed_time
                # print(f"A Vel: {self.velocity} E time: {elapsed_time} D ang: {delta_angle}")
                new_angle = self.angle + delta_angle
                self.set_angle(new_angle)

            previous_time = current_time  # Update previous time
