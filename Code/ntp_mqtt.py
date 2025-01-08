import time
import struct
from machine import RTC, Timer, UART, Pin
from mqtt_client import MQTTClient  # Assuming you have the optimized MQTTClient class
from ch9121 import CH9121

# --- Constants ---
NTP_TOPIC = "ntp/time"  # Consider making this configurable

# --- Callback Functions ---

def process_time_sync(topic, msg):
    """
    Callback function to process incoming time synchronization messages.

    Args:
        topic: The MQTT topic the message was received on.
        msg: The message payload (bytearray representing the timestamp).
    """
    if topic == NTP_TOPIC.encode('utf-8'):
        try:
            print("process_time_sync() received payload (hex):", msg.hex())
            
            # Check if the payload is long enough
            if len(msg) < 4:
                print("Error: NTP timestamp payload too short")
                return

            # Extract the last 4 bytes as the timestamp
            timestamp_bytes = msg[-4:]

            # Try different byte orders
            for byte_order in ["!", "<"]:  # Big-endian and little-endian
                try:
                    ntp_timestamp = struct.unpack(f"{byte_order}I", timestamp_bytes)[0]
                    print(f"  Unpacked timestamp ({'big-endian' if byte_order == '!' else 'little-endian'}): {ntp_timestamp}")

                    # Convert to seconds since the Unix epoch (1970)
                    #unix_timestamp = ntp_timestamp - NTP_DELTA

                    # Convert to a time tuple and set the RTC
                    tm = time.gmtime(ntp_timestamp)
                    RTC().datetime((tm[0], tm[1], tm[2], tm[6] + 1, tm[3], tm[4], tm[5], 0))

                    print("Time synced via MQTT:", tm)
                    return  # Exit after successful processing

                except struct.error:
                    print(f"  Could not unpack timestamp as {'big-endian' if byte_order == '!' else 'little-endian'} integer")

            print("Error: Unable to process timestamp from payload")

        except Exception as e:
            print("Error processing time sync message:", e)

# --- Network and MQTT Initialization ---

def init_network_ch9121(uart_id=1, tx_pin=20, rx_pin=21):
    """
    Initializes the CH9121 network controller with DHCP and configures MQTT.

    Args:
        uart_id: The UART ID to use (default 1).
        tx_pin: The TX pin number (default 20).
        rx_pin: The RX pin number (default 21).

    Returns:
        The configured MQTTClient object.
    """
    # Initialize UART for CH9121 configuration and MQTT communication
    uart = UART(uart_id, baudrate=115200, tx=Pin(tx_pin), rx=Pin(rx_pin))

    # Configure CH9121 (consider adding error handling and retries)
    eth = CH9121(uart_id, tx_pin=tx_pin, rx_pin=rx_pin, config=True)

    # Create and connect MQTTClient
    mqtt_client = MQTTClient(uart, process_time_sync)

    # Always reinitialize CH9121 before connecting
    eth.reinitialize_ch9121()

    # Wait for CH9121 to initialize before connecting
    time.sleep(5)

    # Connect to MQTT broker
    if not mqtt_client.connect():
        print("Initial MQTT connection failed.")
    #else:
        # Subscribe to NTP topic
        #subscribe_time_sync(mqtt_client, NTP_TOPIC)

    return mqtt_client

# --- Helper Functions ---

def subscribe_time_sync(mqtt_client, topic):
    """
    Subscribes to an MQTT topic for time synchronization messages.

    Args:
        mqtt_client: The MQTTClient object.
        topic: The MQTT topic to subscribe to.
    """
    try:
        if mqtt_client.subscribe(topic):
            print("Subscribed to time sync topic:", topic)
        else:
            print("Failed to subscribe to time sync topic:", topic)
    except Exception as e:
        print("Error subscribing to time sync topic:", e)

def send_mqtt_message(mqtt_client, topic, message):
    """
    Sends an MQTT message.

    Args:
        mqtt_client: The MQTTClient object.
        topic: The MQTT topic to publish to.
        message: The message payload.
    """
    try:
        mqtt_client.publish(topic, message)
        print("MQTT message sent:", topic, message)
    except Exception as e:
        print("Error sending MQTT message:", e)

# --- Example Usage (Main Loop) ---
# if __name__ == "__main__":
#     mqtt_client = init_network_ch9121()

#     # Example: Publish a message every 30 seconds
#     timer = Timer(-1)
#     timer.init(period=30000, mode=Timer.PERIODIC, callback=lambda t: send_mqtt_message(mqtt_client, b"test/topic", b"Hello from RP2040!"))

#     while True:
#        mqtt_client.process_incoming_data()
#        time.sleep(0.1) # Short delay to avoid busy-waiting