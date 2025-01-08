# config.py

DHCP = True
# For DHCP, comment out the lines below:
STATIC_IP = (192, 168, 15, 115)    # Define the static IP address
NETMASK = (255, 255, 255, 0)    # Define the netmask
GATEWAY = (192, 168, 15, 1)      # Define the gateway

DEVICE_NAME = "WeatherSen01"

MQTT_SERVER_IP = (192, 168, 10, 70)
MQTT_SERVER_PORT = 1883