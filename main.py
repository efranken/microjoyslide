import asyncio
from machine import I2C, Pin
from trill import Bar
from touch import Touches1D
from time import ticks_ms
import usb.device
import joystick

SCL_PIN = 9
SDA_PIN = 8
HAPTIC_PIN = 26
HAPTIC_PRESENCE = True # change to false if haptic motor is not installed on board
HAPTIC_PULSE_TIME = .15 # how long to pulse haptic motor in seconds

SENSOR_MIN = 0 # trill bar outputs 0 as min by default
SENSOR_MAX = 3200 # trill bar outputs 3200 as max by default
SENSOR_CENTER = (SENSOR_MAX + SENSOR_MIN) // 2

JOYSTICK_MIN = -127 # set by b'\x15\x81'  # Logical Minimum (-127) in HID descriptor of joystick.py
JOYSTICK_MAX = 127 # set by  b'\x25\x7F'  # Logical Maximum (127) in HID descriptor of hid.py
JOYSTICK_CENTER = (JOYSTICK_MAX + JOYSTICK_MIN) // 2

POLL_FREQUENCY = 0.004  # seconds (250hz)

THRESHOLD_PERCENTAGE = 0.15 # percentage for how close to end of left and right of sensor are to be considered joystick max left or right
DEADZONE_PERCENTAGE = 0.02 # percentage of wiggle room in the middle of the sensor for the joystick to be considered at center
MOVEMENT_THRESHOLD_PERCENTAGE = 0.05 # percentage of distance between 2 sensor polls for a touch to be considered as originating from the same source

THRESHOLD = int(SENSOR_MAX * THRESHOLD_PERCENTAGE)
DEADZONE_START = int(SENSOR_MAX * (0.5 - DEADZONE_PERCENTAGE / 2))
DEADZONE_END = int(SENSOR_MAX * (0.5 + DEADZONE_PERCENTAGE / 2))
MOVEMENT_THRESHOLD = int(SENSOR_MAX * MOVEMENT_THRESHOLD_PERCENTAGE)

MAX_TOUCHES = 5

BAR = None 
JOYSTICK = None
HAPTIC = None

if HAPTIC_PRESENCE:
    PULSE_SENT = False

async def init():
    global BAR, JOYSTICK, HAPTIC

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
        BAR = Bar(i2c)
        print("Bar sensor initialized")
    except Exception as e:
        print(f"Error initializing Bar sensor: {e}")
        raise

    try:
        JOYSTICK = joystick.JoystickInterface()
        usb.device.get().init(JOYSTICK, builtin_driver=True)
        print("Joystick initialized")
    except Exception as e:
        print(f"Error initializing joystick: {e}")
        raise
    
    if HAPTIC_PRESENCE:
        try:
            HAPTIC = Pin(HAPTIC_PIN, Pin.OUT)
            print("Haptic motor initialized")
        except Exception as e:
            print(f'Error initializing haptic motor')
            raise

    print("Waiting for joystick interface to open...")
    await asyncio.sleep(0.1) 

# contains all logic for how raw sensor values should translate to joystick values
async def handle_steering(touch_location):
    if HAPTIC_PRESENCE:
        global PULSE_SENT
    steering_value = SENSOR_CENTER

    if touch_location <= THRESHOLD:
        steering_value = SENSOR_MIN
        PULSE_SENT = False
    elif touch_location >= (SENSOR_MAX - THRESHOLD):
        steering_value = SENSOR_MAX
        PULSE_SENT = False
    elif DEADZONE_START <= touch_location <= DEADZONE_END:
        steering_value = SENSOR_CENTER
        if HAPTIC_PRESENCE:
            PULSE_SENT = True
            asyncio.create_task(handle_pin_pulse())
    else:
        if touch_location < DEADZONE_START:
            steering_value = map_value(touch_location, THRESHOLD, DEADZONE_START, SENSOR_MIN, SENSOR_CENTER)
        else:
            steering_value = map_value(touch_location, DEADZONE_END, SENSOR_MAX - THRESHOLD, SENSOR_CENTER, SENSOR_MAX)
        PULSE_SENT = False

    return steering_value

# asynchronously pulse haptic motor while still accepting input
# will only be called if HAPTIC_PRESENCE = true
async def handle_pin_pulse():
    print("Steering center - activating pin")
    HAPTIC.value(1)
    await asyncio.sleep(HAPTIC_PULSE_TIME) 
    HAPTIC.value(0)
    print("Pin turned off")

# example
# (1000 - -127) * (127 - -127) // (3500 - 0) + 0
# 1000 * 254 // 3500 + 0
# 25400 // 3500
# 72
def map_value(value_to_map, input_min, input_max, output_min, output_max):
    mapped_value = (value_to_map - input_min) * (output_max - output_min) // (input_max - input_min) + output_min
    return mapped_value

async def loop():
    global JOYSTICK

    try:
        while True:
            touches = [{'location': -1, 'timestamp': 0, 'active': False} for _ in range(MAX_TOUCHES)]
            data = BAR.read()
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
                steering_value = await handle_steering(touch_location)
            else:
                steering_value = SENSOR_CENTER

            joystick_value = map_value(steering_value, SENSOR_MIN, SENSOR_MAX, JOYSTICK_MIN, JOYSTICK_MAX)
            print(f"Steering value: {steering_value}, Joystick value: {joystick_value}, pulse sent {PULSE_SENT}")
            JOYSTICK.send_joystick(joystick_value)

            await asyncio.sleep(POLL_FREQUENCY)

    except OSError as e:
        print(f"Error reading from Bar sensor: {e}")

async def main():
    try:
        await init()
        await loop()
    except Exception as e:
        print(f"Exception: {e}")


if __name__ == '__main__':
    asyncio.run(main())
