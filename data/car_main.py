'''*****************************************************************************************
Use the app Sunfounder Controller to control the Pico-4WD-Car

https://github.com/sunfounder/pico_4wd_car/tree/v2.0

Pico onboard LED status:
    - off: not working
    - always on: working
    - blink: error

*****************************************************************************************'''
import time
import sys
import motors as car
import sonar as sonar
import lights as lights
from speed import Speed
from grayscale import Grayscale
from ws import WS_Server
from machine import Pin

print("Running...\n")

LOG_FILE = "log.txt"

with open(LOG_FILE, "w") as log_f:
    log_f.write("")

onboard_led = Pin(25, Pin.OUT)

NAME = 'blue-car'

WIFI_MODE = "sta"
SSID = "iPhone (8)"
PASSWORD = "password"

GRAYSCALE_LINE_REFERENCE_DEFAULT = 10000
GRAYSCALE_CLIFF_REFERENCE_DEFAULT = 2000

NORMAL_SCAN_ANGLE = 180
NORMAL_SCAN_STEP = 5

signal_on_color = [80, 30, 0]
brake_on_color = [255, 0, 0] 

led_status = False

lights_brightness = 0.2
led_rear_min_brightness = 0.08
led_rear_max_brightness = 1

led_theme_code = 0
led_theme = {
    "0": [0, 0, 255], # blue
    "1": [255, 0, 255], # purple
    "2": [200, 0, 0], # red 
    "3": [128, 20, 0], # orange 
    "4": [128, 128, 0], # yellow 
    "5": [0, 128, 0], # green
}
led_theme_sum = len(led_theme)

move_status = 'stop'
is_move_last  = False
brake_light_status= False
brake_light_time = 0
brake_light_brightness = 255 # 0 ~ 255
brake_light_brightness_flag = -1 # -1 or 1

sonar_on = True
sonar_angle = 0
sonar_distance = 0

'''------------ Instantiate -------------'''
try:
    speed = Speed(8, 9)
    grayscale = Grayscale(26, 27, 28)
    ws = WS_Server(name=NAME, mode=WIFI_MODE, ssid=SSID, password=PASSWORD)
except Exception as e:
    onboard_led.off()
    sys.print_exception(e)
    with open(LOG_FILE, "a") as log_f:
        log_f.write('\n> ')
        sys.print_exception(e, log_f)
    sys.exit(1) # if ws init failed, exit

def signal_lights_handler():
    if move_status == 'left':
        lights.set_rear_left_color(signal_on_color)
        lights.set_rear_right_color(0x000000)
    elif move_status == 'right':
        lights.set_rear_left_color(0x000000)
        lights.set_rear_right_color(signal_on_color)
    else:
        lights.set_rear_left_color(0x000000)
        lights.set_rear_right_color(0x000000)

def brake_lights_handler():
    global is_move_last , brake_light_status, brake_light_time, led_status, brake_light_brightness
    global brake_light_brightness, brake_light_brightness_flag

    if move_status == 'stop':
        if brake_light_brightness_flag == 1:
            brake_light_brightness += 5
            if brake_light_brightness > 255:
                brake_light_brightness = 255
                brake_light_brightness_flag = -1
        elif brake_light_brightness_flag == -1:
            brake_light_brightness -= 5
            if brake_light_brightness < 0:
                brake_light_brightness = 0
                brake_light_brightness_flag = 1          
        brake_on_color = [brake_light_brightness, 0, 0]
        lights.set_rear_color(brake_on_color)
    else:
        if is_move_last:
            lights.set_rear_middle_color(0x000000)
        else:
            lights.set_rear_color(0x000000)
        is_move_last = True
        brake_light_brightness = 255

def bottom_lights_handler():
    global led_status
    if led_status:
        color = list(led_theme[str(led_theme_code)])
    else:
        color = [0, 0, 0]
    lights.set_bottom_color(color)


def on_receive(data):
    global led_status, led_theme_code, led_theme_sum, lights_brightness
    global sonar_on, sonar_angle, sonar_distance
    
    if sonar_on:
        sonar.set_sonar_scan_config(NORMAL_SCAN_ANGLE, NORMAL_SCAN_STEP)
        sonar_angle, sonar_distance, _ = sonar.sonar_scan()
    else:
        sonar_angle = 0
        sonar_distance = sonar.get_distance_at(sonar_angle)
    
    if 'motors' in data.keys():
        car.set_motors_power(data['motors'])
    
    # greyscale
    ws.send_dict['A'] = grayscale.get_value()
    # Speed measurement
    ws.send_dict['B'] = round(speed.get_speed(), 2) # uint: cm/s
    # Speed mileage
    ws.send_dict['C'] = speed.get_mileage() # unit: meter
    # # sonar and distance
    ws.send_dict['D'] = [sonar_angle, sonar_distance]
    ws.send_dict['E'] = sonar_distance
    
    bottom_lights_handler()
    signal_lights_handler()
    brake_lights_handler()

def main():
    sonar.servo.set_angle(0)
    car.move('stop')
    ws.on_receive = on_receive
    if ws.start():
        onboard_led.on()
        while True:
            if not ws.is_connected():
                car.move('stop', 0)
            ws.loop()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        sys.print_exception(e)
        with open(LOG_FILE, "a") as log_f:
            log_f.write('\n> ')
            sys.print_exception(e, log_f)
    finally:
        car.move("stop")
        lights.set_off()
        ws.set("RESET", timeout=2500)
        while True: # pico onboard led blinking indicates error
            time.sleep(0.25)
            onboard_led.off()
            time.sleep(0.25)
            onboard_led.on()