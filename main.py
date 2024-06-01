import asyncio
from machine import I2C, Pin
from trill import Bar
from touch import Touches1D
from time import ticks_ms, ticks_us, ticks_diff
import usb.device
import joystick

# Configuration constants
SCL_PIN = 9
SDA_PIN = 8
HAPTIC_PIN = 26
HAPTIC_PRESENCE = True  # Set to False if haptic motor is not installed
HAPTIC_PULSE_TIME = 0.08  # Pulse duration for haptic motor in seconds

SENSOR_MIN = 0  # Trill bar minimum output
SENSOR_MAX = 3200  # Trill bar maximum output
SENSOR_CENTER = (SENSOR_MAX + SENSOR_MIN) // 2

JOYSTICK_MIN = -127  # Joystick logical minimum
JOYSTICK_MAX = 127  # Joystick logical maximum
JOYSTICK_CENTER = (JOYSTICK_MAX + JOYSTICK_MIN) // 2

POLL_FREQUENCY = 0.004  # Polling frequency in seconds (250 Hz)

THRESHOLD_PERCENTAGE = 0.15  # Threshold percentage for max left/right
DEADZONE_PERCENTAGE = 0.04  # Deadzone percentage for joystick center
MOVEMENT_THRESHOLD_PERCENTAGE = 0.05  # Movement threshold percentage for touch

THRESHOLD = int(SENSOR_MAX * THRESHOLD_PERCENTAGE)
DEADZONE_START = int(SENSOR_MAX * (0.5 - DEADZONE_PERCENTAGE / 2))
DEADZONE_END = int(SENSOR_MAX * (0.5 + DEADZONE_PERCENTAGE / 2))
MOVEMENT_THRESHOLD = int(SENSOR_MAX * MOVEMENT_THRESHOLD_PERCENTAGE)

MAX_TOUCHES = 5

class SteeringController:
    def __init__(self):
        self.bar = None
        self.joystick = None
        self.haptic = None
        self.previously_in_deadzone = False
        self.haptic_active = False
        self.haptic_activation_time = 0

async def init(controller):
    """
    Initialize the I2C, Trill bar sensor, joystick, and haptic motor.
    """
    try:
        i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
        print(f"I2C initialized: {i2c}")

        devices = i2c.scan()
        if devices:
            print("I2C devices found:", [hex(device) for device in devices])
        else:
            print("No I2C devices found")
            raise Exception("No I2C devices found")
    except OSError as e:
        print(f"I2C initialization error: {e}")
        raise

    try:
        controller.bar = Bar(i2c)
        print("Bar sensor initialized")
    except Exception as e:
        print(f"Error initializing Bar sensor: {e}")
        raise

    try:
        controller.joystick = joystick.JoystickInterface()
        usb.device.get().init(controller.joystick, builtin_driver=True)
        print("Joystick initialized")
    except Exception as e:
        print(f"Error initializing joystick: {e}")
        raise
    
    if HAPTIC_PRESENCE:
        try:
            controller.haptic = Pin(HAPTIC_PIN, Pin.OUT)
            print("Haptic motor initialized")
        except Exception as e:
            print('Error initializing haptic motor')
            raise

    print("Waiting for joystick interface to open...")
    await asyncio.sleep(0.1)

async def handle_steering(controller, touch_location):
    """
    Handle the steering logic based on touch location.
    """
    steering_value = SENSOR_CENTER

    if touch_location <= THRESHOLD:
        steering_value = SENSOR_MIN
        controller.previously_in_deadzone = False
    elif touch_location >= (SENSOR_MAX - THRESHOLD):
        steering_value = SENSOR_MAX
        controller.previously_in_deadzone = False
    elif DEADZONE_START <= touch_location <= DEADZONE_END:
        steering_value = SENSOR_CENTER
        if HAPTIC_PRESENCE and not controller.previously_in_deadzone:
            controller.haptic_activation_time = ticks_ms()
            controller.haptic.value(1)
            controller.haptic_active = True
        controller.previously_in_deadzone = True
    else:
        if touch_location < DEADZONE_START:
            steering_value = map_value(touch_location, THRESHOLD, DEADZONE_START, SENSOR_MIN, SENSOR_CENTER)
        else:
            steering_value = map_value(touch_location, DEADZONE_END, SENSOR_MAX - THRESHOLD, SENSOR_CENTER, SENSOR_MAX)
        controller.previously_in_deadzone = False

    return steering_value

def update_haptic(controller):
    if controller.haptic_active and ticks_diff(ticks_ms(), controller.haptic_activation_time) >= int(HAPTIC_PULSE_TIME * 1000):
        controller.haptic.value(0)
        controller.haptic_active = False

async def loop(controller):
    """
    Main loop for reading sensor data and updating joystick values.
    """
    try:
        while True:
            loop_start = ticks_us()

            touches = [{'location': -1, 'timestamp': 0, 'active': False} for _ in range(MAX_TOUCHES)]
            data = controller.bar.read()
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
                steering_value = await handle_steering(controller, touch_location)
            else:
                steering_value = SENSOR_CENTER

            joystick_value = map_value(steering_value, SENSOR_MIN, SENSOR_MAX, JOYSTICK_MIN, JOYSTICK_MAX)

            controller.joystick.send_joystick(joystick_value)

            update_haptic(controller)

            loop_end = ticks_us()
            loop_duration = ticks_diff(loop_end, loop_start)
            print(f"Steering value: {steering_value}, Joystick value: {joystick_value}, Loop duration: {loop_duration} us")

            await asyncio.sleep(max(POLL_FREQUENCY - (loop_duration / 1_000_000), 0))

    except OSError as e:
        print(f"Error reading from Bar sensor: {e}")

def map_value(value_to_map, input_min, input_max, output_min, output_max):
    """
    Map a value from one range to another.
    """
    return (value_to_map - input_min) * (output_max - output_min) // (input_max - input_min) + output_min

async def main():
    """
    Main entry point for the program.
    """
    controller = SteeringController()
    try:
        await init(controller)
        await loop(controller)
    except Exception as e:
        print(f"Exception: {e}")

if __name__ == '__main__':
    asyncio.run(main())
