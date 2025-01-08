import ntp_mqtt  # Import the library you just created
from machine import Pin, I2C, Timer, ADC
import time
import json
import sht31

# --- Constants ---
RAIN_SENSOR_PIN = 2
WIND_SENSOR_PIN = 3
WIND_VANE_PIN = 26
I2C_SCL_PIN = 5
I2C_SDA_PIN = 4
SHT31_ADDRESS = 0x44
MQTT_TOPIC = "weather"  # Consider making this configurable
DEBOUNCE_TIME_MS = 15  # 15ms debounce time for the sensors
SAMPLE_PERIOD_MS = 6000  # Sample every 6 seconds
HEARTBEAT_PERIOD = 500
RECONNECT_DELAY = 5 # Delay between reconnect attempts
NTP_TOPIC = "ntp/time"  # Consider making this configurable

# Rain gauge constants
RAIN_RATE_MAX_LEN = 48 # 48 is equivalent to 48 * 6 seconds or 4.8 minutes or 288 seconds
RAIN_RATE_AVG_LEN = 10 # 10 is equivalent to 10 * 6 seconds or 1 minute

# Wind vane offset
VANE_OFFSET = 0  # Adjust this if needed

# --- Global variables ---
is_sample_required = False
timer_count = 0
tips = 0
contact_bounce_time = 0
last_tip_time = [0] * 10  # Keep track of the last 10 tip times
rain_rate_max = [0.0] * RAIN_RATE_MAX_LEN
rain_rate_avg = [0.0] * RAIN_RATE_AVG_LEN
total_tips = 0
zero_count = 0
last_heartbeat = 0

# Wind sensor variables
rotations = 0
wind_speed = 0.0
wind_speed_max = [0.0] * 120  # Keep track of the last 120 wind speeds (for gust calculation)
current_max = 0.0
last_wind_direction = 0

# --- Classes ---

class Debouncer:
    """Debounces a digital input."""

    def __init__(self, pin, mode, callback, debounce_time=DEBOUNCE_TIME_MS):
        self.pin = pin
        self.callback = callback
        self.debounce_time = debounce_time
        self.last_time = 0

        # Correctly set up IRQ trigger based on the desired mode
        if mode == "FALLING":
            self.pin.irq(trigger=Pin.IRQ_FALLING, handler=self._isr)
        elif mode == "RISING":
            self.pin.irq(trigger=Pin.IRQ_RISING, handler=self._isr)
        else:
            raise ValueError("Invalid mode. Must be 'FALLING' or 'RISING'.")

    def _isr(self, pin):
        current_time = time.ticks_ms()
        if (current_time - self.last_time) > self.debounce_time:
            self.last_time = current_time
            self.callback(pin)
            
# --- Callback Functions ---

def isr_rain_tip(pin):
    """Interrupt service routine for rain tip sensor."""
    global tips, last_tip_time
    tips += 1
    last_tip_time[tips % len(last_tip_time)] = time.ticks_ms() # Use modulo to avoid index error

def isr_rotation(pin):
    """Interrupt service routine for wind speed sensor."""
    global rotations
    rotations += 1

def isr_timer(timer):
    """Interrupt service routine for the sampling timer."""
    global is_sample_required, tim
    is_sample_required = True
    tim.init(period=5000, mode=machine.Timer.ONE_SHOT, callback=isr_timer) # Comment out timer
    
# --- Sensor Functions ---

def read_wind_vane():
    """Reads and calibrates the wind vane ADC value."""
    global VANE_OFFSET
    wind_vane = ADC(Pin(WIND_VANE_PIN))
    vane_value = wind_vane.read_u16()
    direction = map_value(vane_value, 0, 65535, 0, 359)
    cal_direction = (direction + VANE_OFFSET) % 360
    return cal_direction

def get_wind_heading(direction):
    """Converts wind direction in degrees to a compass heading."""
    headings = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    index = int((direction + 22.5) // 45) % 8
    return headings[index]

def read_sht31(i2c, addr=SHT31_ADDRESS):
    """Reads temperature and humidity from the SHT31 sensor."""
    th_sensor = sht31.SHT31(i2c, addr=addr)
    try:
        temperature, humidity = th_sensor.get_temp_humi()
        return temperature, humidity
    except Exception as e:
        print("Error reading SHT31:", e)
        return None, None

# --- Utility Functions ---

def map_value(value, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another."""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def add_leading_zero(value):
    """Adds a leading zero to single-digit numbers."""
    return "0" + str(value) if value < 10 else str(value)

# --- Data Processing Functions ---

def calculate_rain_rate(current_tips):
    """Calculates rain rate based on tip times."""
    global rain_rate_max, rain_rate_avg, zero_count, last_tip_time

    if current_tips == 0:
        zero_count += 1
        if zero_count >= RAIN_RATE_AVG_LEN: # Use constant here
            rain_rate_max.pop(0)
            rain_rate_max.append(0.0)
            rain_rate_avg.pop(0)
            rain_rate_avg.append(0.0)
            zero_count = 0
    else:
        zero_count = 0
        # Calculate rain rate for each tip interval
        for i in range(current_tips):
            interval = last_tip_time[(tips - current_tips + i) % len(last_tip_time)] - last_tip_time[(tips - current_tips + i -1) % len(last_tip_time)]
            if interval > 0:
                rain_rate = 3600.0 / interval * 0.2794 # 1 tip is 0.01 inches or 0.2794 mm
            else:
                rain_rate = 0.0
                
            rain_rate_max.pop(0)
            rain_rate_max.append(rain_rate)
            rain_rate_avg.pop(0)
            rain_rate_avg.append(rain_rate)
        
def calculate_wind_speed():
    """Calculates wind speed based on rotations."""
    global rotations, wind_speed, wind_speed_max, current_max
    wind_speed = rotations * 0.9 / (SAMPLE_PERIOD_MS / 1000) # 0.9 m/s per rotation per second, adjusted for sample period
    rotations = 0
    wind_speed_max.pop(0)
    wind_speed_max.append(wind_speed)
    current_max = max(wind_speed_max)

# --- Setup Function ---

def setup():
    global is_sample_required, timer_count, tips, total_tips, last_tip_time, contact_bounce_time, tim
    global rotations, wind_speed_max, last_wind_direction, i2c, SHT31_ADDRESS, th_sensor

    is_sample_required = False
    timer_count = 0
    tips = 0
    total_tips = 0
    contact_bounce_time = 0
    rotations = 0
    last_wind_direction = 0

    # Initialize rain sensor pin
    rain_sensor = Pin(RAIN_SENSOR_PIN, Pin.IN)
    rain_sensor.irq(trigger=Pin.IRQ_FALLING, handler=isr_rain_tip) # Comment out IRQ

    # Initialize wind sensor pin
    wind_sensor = Pin(WIND_SENSOR_PIN, Pin.IN)
    wind_sensor.irq(trigger=Pin.IRQ_FALLING, handler=isr_rotation) # Comment out IRQ

    # Initialize wind vane pin
    wind_vane = ADC(Pin(WIND_VANE_PIN))

    # Initialize timer interrupt
    tim = Timer(-1)  # Comment out timer
    tim.init(period=1000, mode=machine.Timer.ONE_SHOT, callback=isr_timer) # Comment out timer

    # Initialize SHT31 sensor
    i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq =400000)
    thSensor = sht31.SHT31(i2c, addr=0x44)

    print("Weather Station Start - lenRainRateMax:", RAIN_RATE_MAX_LEN, "(", RAIN_RATE_MAX_LEN,
          ") - lenRainRateAvg:", RAIN_RATE_AVG_LEN, "(", RAIN_RATE_AVG_LEN, ")")
    
    mqtt_client.send_heartbeat()

# --- Main Loop ---

def loop(mqtt_client):
    """Main loop for processing sensor data and sending MQTT messages."""
    global is_sample_required, tips, total_tips, last_tip_time, rain_rate_max, rain_rate_avg, zero_count, last_heartbeat
    global rotations, wind_speed, wind_speed_max, current_max, last_wind_direction, th_sensor, last_heartbeat

    print("Starting Main Loop")

    while True:
        # MQTT connection check and reconnection logic
        mqtt_client.process_incoming_data()
        if time.ticks_ms() - last_heartbeat > HEARTBEAT_PERIOD:
            if not mqtt_client.check_connection():
                print("MQTT connection lost. Reconnecting...")
                return False
            last_heartbeat = time.ticks_ms()

        if is_sample_required:
            is_sample_required = False
            current_tips = tips
            tips = 0
            
            # Rain Rate calculation
            calculate_rain_rate(current_tips)

            # Wind Speed calculation
            calculate_wind_speed()

            # Wind Direction
            wind_direction = read_wind_vane()
            if abs(wind_direction - last_wind_direction) > 5:
                last_wind_direction = wind_direction

            # Read temperature and humidity
            temperature, humidity = read_sht31(i2c)
            
            # Handle None values (sensor errors)
            temperature = temperature if temperature is not None else 0
            humidity = humidity if humidity is not None else 0

            # Get current timestamp
            timestamp = time.localtime()
            timestamp_text = "{:02d}/{:02d}/{} {:02d}:{:02d}:{:02d}".format(
                timestamp[1],
                timestamp[2],
                timestamp[0],
                timestamp[3],
                timestamp[4],
                timestamp[5],
            )
            
            # Create sensor data JSON
            sensor_data = {
                "weather": {
                    "timestamp": timestamp_text,
                    "rain": {
                        "rate": sum(rain_rate_avg) / len(rain_rate_avg),
                        "max": max(rain_rate_max),
                        "tips": current_tips,
                        "total": total_tips,
                    },
                    "wind": {
                        "speedMPH": wind_speed,
                        "gustMPH": current_max,
                        "direction": {
                            "degrees": int(wind_direction),
                            "heading": get_wind_heading(wind_direction),
                        },
                    },
                    "temperature": {
                        "celsius": int(temperature),
                        "humidity": int(humidity),
                    },
                }
            }

            # Send sensor data via MQTT
            print("Sending MQTT message:", json.dumps(sensor_data))
            ntp_mqtt.send_mqtt_message(mqtt_client, MQTT_TOPIC, json.dumps(sensor_data))
            total_tips += current_tips

# --- Main Program ---
if __name__ == "__main__":
    # Initialize network and MQTT client
    mqtt_client = ntp_mqtt.init_network_ch9121()

    setup()  # Run the setup function

    while True:
        while not mqtt_client.check_connection():
            print("MQTT Connection Lost. Reconnecting.")
            #mqtt_client.uart.deinit()
            mqtt_client = ntp_mqtt.init_network_ch9121()
        print("MQTT connection OK")
        
        ntp_mqtt.subscribe_time_sync(mqtt_client, NTP_TOPIC)

        #mqtt_client.process_incoming_data()
        loop(mqtt_client)
        time.sleep(1)