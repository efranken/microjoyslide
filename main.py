import asyncio
from machine import I2C, Pin
from trill import Bar
from touch import Touches1D
from time import ticks_ms, ticks_diff, ticks_us
import usb.device
import joystick

SCL_PIN = 9
SDA_PIN = 8
HAPTIC_PIN = 26
HAPTIC_PRESENCE = True  # Change to false if the haptic motor is not installed on the board
HAPTIC_PULSE_TIME = 0.08  # How long to pulse the haptic motor in seconds

SENSOR_MIN = 0  # Trill bar outputs 0 as min by default
SENSOR_MAX = 3200  # Trill bar outputs 3200 as max by default
SENSOR_CENTER = (SENSOR_MAX + SENSOR_MIN) // 2

JOYSTICK_MIN = -127  # Set by b'\x15\x81'  # Logical Minimum (-127) in HID descriptor of joystick.py
JOYSTICK_MAX = 127  # Set by  b'\x25\x7F'  # Logical Maximum (127) in HID descriptor of hid.py
JOYSTICK_CENTER = (JOYSTICK_MAX + JOYSTICK_MIN) // 2

THRESHOLD_PERCENTAGE = 0.15  # Percentage for how close to end of the left and right of the sensor are to be considered joystick max left or right
DEADZONE_PERCENTAGE = 0.04  # Percentage of wiggle room in the middle of the sensor for the joystick to be considered at the center
MOVEMENT_THRESHOLD_PERCENTAGE = 0.05  # Percentage of distance between 2 sensor polls for a touch to be considered as originating from the same source

THRESHOLD = int(SENSOR_MAX * THRESHOLD_PERCENTAGE)
DEADZONE_START = int(SENSOR_MAX * (0.5 - DEADZONE_PERCENTAGE / 2))
DEADZONE_END = int(SENSOR_MAX * (0.5 + DEADZONE_PERCENTAGE / 2))
MOVEMENT_THRESHOLD = int(SENSOR_MAX * MOVEMENT_THRESHOLD_PERCENTAGE)

MAX_TOUCHES = 5

class SteeringController:
    def __init__(self):
        self.i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
        self.bar = Bar(self.i2c)
        self.joystick = joystick.JoystickInterface()
        usb.device.get().init(self.joystick, builtin_driver=True)
        self.haptic = Pin(HAPTIC_PIN, Pin.OUT) if HAPTIC_PRESENCE else None
        self.previously_in_deadzone = False
        print(f"I2C initialized: {self.i2c}")
        print("Bar sensor initialized")
        print("Joystick initialized")
        if HAPTIC_PRESENCE:
            print("Haptic motor initialized")

    async def handle_steering(self, touch_location):
        steering_value = SENSOR_CENTER

        if touch_location <= THRESHOLD:
            steering_value = SENSOR_MIN
            self.previously_in_deadzone = False
        elif touch_location >= (SENSOR_MAX - THRESHOLD):
            steering_value = SENSOR_MAX
            self.previously_in_deadzone = False
        elif DEADZONE_START <= touch_location <= DEADZONE_END:
            steering_value = SENSOR_CENTER
            if HAPTIC_PRESENCE and not self.previously_in_deadzone:
                asyncio.create_task(self.handle_pin_pulse())
            self.previously_in_deadzone = True
        else:
            if touch_location < DEADZONE_START:
                steering_value = map_value(touch_location, THRESHOLD, DEADZONE_START, SENSOR_MIN, SENSOR_CENTER)
            else:
                steering_value = map_value(touch_location, DEADZONE_END, SENSOR_MAX - THRESHOLD, SENSOR_CENTER, SENSOR_MAX)
            self.previously_in_deadzone = False

        return steering_value

    async def handle_pin_pulse(self):
        print("Steering center - activating pin")
        self.haptic.value(1)
        await asyncio.sleep(HAPTIC_PULSE_TIME)
        self.haptic.value(0)
        print("Pin turned off")

    async def loop(self):
        try:
            while True:
                loop_start = ticks_us()
                
                touches = [{'location': -1, 'timestamp': 0, 'active': False} for _ in range(MAX_TOUCHES)]
                data = self.bar.read()
                touches_data = Touches1D(data)

                num_touches = len(touches_data.get_touches())
                current_time = ticks_ms()

                matched_touches = [False] * MAX_TOUCHES

                for i in range(num_touches):
                    current_location = touches_data.get_touches()[i][0]
                    matched = False

                    for j in range(MAX_TOUCHES):
                        if touches[j]['active'] and abs(touches[j]['location'] - current_location) <= MOVEMENT_THRESHOLD:
                            touches[j]['location'] = current_location
                            matched_touches[j] = True
                            matched = True
                            break

                    if not matched:
                        for j in range(MAX_TOUCHES):
                            if not touches[j]['active']:
                                touches[j]['location'] = current_location
                                touches[j]['timestamp'] = current_time
                                touches[j]['active'] = True
                                matched_touches[j] = True
                                break

                for i in range(MAX_TOUCHES):
                    if not matched_touches[i]:
                        touches[i]['active'] = False

                latest_touch_index = -1
                latest_timestamp = 0
                for i in range(MAX_TOUCHES):
                    if touches[i]['active'] and touches[i]['timestamp'] > latest_timestamp:
                        latest_touch_index = i
                        latest_timestamp = touches[i]['timestamp']

                if latest_touch_index != -1:
                    touch_location = touches[latest_touch_index]['location']
                    steering_value = await self.handle_steering(touch_location)
                else:
                    steering_value = SENSOR_CENTER

                joystick_value = map_value(steering_value, SENSOR_MIN, SENSOR_MAX, JOYSTICK_MIN, JOYSTICK_MAX)
                loop_end = ticks_us()
                loop_duration = ticks_diff(loop_end, loop_start)
                print(f"Steering value: {steering_value}, Joystick value: {joystick_value}, Loop Duration: {loop_duration} us")

                self.joystick.send_joystick(joystick_value)

        except OSError as e:
            print(f"Error reading from Bar sensor: {e}")

async def init():
    controller = SteeringController()
    await asyncio.sleep(0.1)
    return controller

def map_value(value_to_map, input_min, input_max, output_min, output_max):
    return (value_to_map - input_min) * (output_max - output_min) // (input_max - input_min) + output_min

async def main():
    controller = await init()
    await controller.loop()

if __name__ == '__main__':
    asyncio.run(main())