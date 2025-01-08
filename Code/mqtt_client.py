from machine import UART, Pin
import time
import struct

# --- MQTT Control Packet Types ---
MQTT_CONTROL_PACKET_TYPES = {
    0x10: "CONNECT",
    0x20: "CONNACK",
    0x30: "PUBLISH",
    0x40: "PUBACK",
    0x50: "PUBREC",
    0x60: "PUBREL",
    0x70: "PUBCOMP",
    0x80: "SUBSCRIBE",
    0x90: "SUBACK",
    0xA0: "UNSUBSCRIBE",
    0xB0: "UNSUBACK",
    0xC0: "PINGREQ",
    0xD0: "PINGRESP",
    0xE0: "DISCONNECT",
}

class MQTTException(Exception):
    pass

class MQTTClient:
    def __init__(self, uart, callback=None):
        self.uart = uart
        self.callback = callback
        self.ClientID = "WeatherSen01"
        self._packet_id = 0
        self.last_packet_received = time.time()

        self.connect_message = self._build_connect_message()

        # Calculate Client ID length and extend connect message
        clientIDLength = len(self.ClientID)
        self.connect_message.extend(bytes([0x00, clientIDLength]))
        self.connect_message.extend(self.ClientID.encode())

        # Update remaining length
        self._update_connect_message_length()

    def _build_connect_message(self):
        """Builds the CONNECT message."""
        connect_message = bytearray([
            0x10,  # MQTT control packet type (CONNECT)
            0x00,  # Placeholder for Remaining Length, will be updated later
            0x00, 0x04,  # Length of the UTF-8 encoded protocol name
            0x4D, 0x51, 0x54, 0x54,  # MQTT string "MQTT"
            0x04,  # Protocol Level (MQTT 3.1.1)
            0x02,  # Connect Flags: Clean Session, No Will, No Will Retain, QoS = 0, No Will Flag, Keep Alive = 60 seconds
            0x00, 0x3C  # Keep Alive Time in seconds
        ])
        return connect_message
    
    def _update_connect_message_length(self):
        """Updates the remaining length in the connect message."""
        remaining_length = 10 + 2 + len(self.ClientID)  # 10 bytes fixed header + 2 bytes Client ID length + Client ID length
        self.connect_message[1] = remaining_length

    def connect(self):
        """Connects to the MQTT broker."""

        # Send connect message
        print("Sending CONNECT message (hex):", self.connect_message.hex())
        self.uart.write(self.connect_message)

        # Read the response from the server (with a timeout)
        try:
            start_time = time.time()
            timeout = 5  # Timeout in seconds
            while time.time() - start_time < timeout:
                response = self._read_uart_response(timeout=2)
                print(response)
                if response:
                    # Assuming the first byte is received correctly
                    ptype = response[0]
                    if (ptype & 0xF0) == 0x20: # Filter for CONNACK packets only
                        if response[0] == 0x20 and response[1] == 0x02:
                            if response[2] == 0x00 and response[3] == 0x00:
                                print("MQTT connection successful.")
                                return True
                            else:
                                print("MQTT connection failed with error code:", response[3])
                                return False
            print("No CONNACK response received from the server within timeout.")
            return False
        except Exception as e:
            print("Error reading server response:", e)
            return False

    def check_connection(self):
        """Checks if the MQTT client is connected to the broker using PINGREQ/PINGRESP."""
        try:
            self.ping()
            start_time = time.time()
            timeout = 5
            while time.time() - start_time < timeout:
                response = self._read_uart_response(timeout=1)
                if response:
                    if response[0] == 0xD0: # Check for PINGRESP
                        print("PINGRESP received")
                        return True
            print("No PINGRESP received within timeout.")
            return False
        except Exception as e:
            print("Error checking MQTT connection:", e)
            return False

    def publish(self, topic, msg, qos=0, retain=False):
        """Publishes a message to the MQTT broker."""
        packet = bytearray()
        packet.append(0x30 | (qos << 1) | retain)
        remaining_length = 2 + len(topic) + len(msg)
        if qos > 0:
            remaining_length += 2
        packet.extend(self._encode_remaining_length(remaining_length))
        packet.extend(self._encode_string(topic))
        if qos > 0:
            packet_id = self._generate_packet_id()
            packet.extend(packet_id.to_bytes(2, 'big'))
        packet.extend(msg.encode('utf-8'))
        print("Sending PUBLISH message (hex):", packet.hex())
        self.uart.write(packet)
        # Handle QOS 1 and 2 acknowledgements here if needed.

    def _generate_packet_id(self):
        """Generates a unique packet identifier (for QOS > 0)."""
        self._packet_id = (self._packet_id + 1) % 65536 or 1
        return self._packet_id

    def _encode_remaining_length(self, length):
        """Encodes the remaining length field for an MQTT packet."""
        encoded = bytearray()
        while True:
            digit = length % 128
            length //= 128
            if length > 0:
                digit |= 0x80
            encoded.append(digit)
            if length == 0:
                break
        return encoded

    def _encode_string(self, s):
        """Encodes a string for an MQTT packet (length followed by UTF-8 bytes)."""
        return bytearray(len(s).to_bytes(2, 'big')) + s.encode('utf-8')

    def subscribe(self, topic, qos=0):
        """Subscribes to a topic."""
        packet = bytearray()
        packet.append(0x82)
        remaining_length = 2 + 2 + len(topic) + 1
        packet.extend(self._encode_remaining_length(remaining_length))
        packet_id = self._generate_packet_id()
        packet.extend(packet_id.to_bytes(2, 'big'))
        packet.extend(self._encode_string(topic))
        packet.append(qos)
        print("Sending SUBSCRIBE message (hex):", packet.hex())
        self.uart.write(packet)

        # Wait for a SUBACK packet (with a timeout)
        start_time = time.time()
        timeout = 5
        while time.time() - start_time < timeout:
            response = self._read_uart_response(timeout=2)
            if response:
                if response[0] == 0x90: # Check for SUBACK
                    print("SUBACK received")
                    return True
        print("No SUBACK received within timeout.")
        return False

    def _read_uart_response(self, timeout=1):
        start_time = time.time()
        response = b''
        while (time.time() - start_time) < timeout:
            if self.uart.any():
                response += self.uart.read(self.uart.any())  # Read all available bytes
                print("Partial response (hex):", response.hex())
                if b'\x20\x02\x00\x00' in response:  # Check for CONNACK
                    print("CONNACK found in response")
                    return response
            time.sleep(0.01)
        print("Timeout receiving response from UART")
        print("Received (hex):", response.hex())
        return response

    def _recv_packet(self):
        remaining_length = 0
        bytes_processed = 0

        # Read the first byte (control packet type and flags)
        packet_type_flags = self.uart.read(1)
        if not packet_type_flags:
            # Timeout, no data received
            return None
        else:
            packet_type = packet_type_flags[0]
            packet_type_name = MQTT_CONTROL_PACKET_TYPES.get(packet_type & 0xF0)
            if packet_type_name:
                print(f"Control Packet Received: {packet_type_name} (hex: {packet_type_flags.hex()})")
            else:
                print(f"Received unknown Control Packet (hex): {packet_type_flags.hex()}")
            bytes_processed = 1

        # Handle based on packet type
        if packet_type & 0xF0 == 0xD0: # PINGRESP
            print("Received PINGRESP")
            self.last_packet_received = time.time()
            bytes_processed = 2
            return None

        elif packet_type & 0xF0 == 0x90: # SUBACK
            suback_payload = self.uart.read(3)
            print("Received SUBACK")
            self.last_packet_received = time.time()
            bytes_processed = 4
            return None
        
        elif packet_type & 0xF0 == 0x20: # CONNACK
            connack_payload = self.uart.read(3)
            print("Received CONNACK")
            self.last_packet_received = time.time()
            bytes_processed = 4
            return None

        elif packet_type & 0xF0 == 0x30:  # PUBLISH
            # Decode the remaining length (could be 1 to 4 bytes)
            remaining_length = 0
            multiplier = 1
            while True:
                encoded_byte = self.uart.read(1)
                if encoded_byte == b'':
                    print("Timeout receiving remaining length")
                    return None
                encoded_byte = encoded_byte[0]
                remaining_length += (encoded_byte & 127) * multiplier
                multiplier *= 128
                bytes_processed += 1
                if (encoded_byte & 128) == 0:
                    break

            # Read the rest of the packet based on remaining length
            if remaining_length == 0:
                return None
            
            payload = self.uart.read(remaining_length)
            if len(payload) != remaining_length:
                print("Timeout receiving response from UART")
                return None

            bytes_processed += remaining_length
            self.last_packet_received = time.time()
            topic, message = self._extract_publish_data(payload)

            # Call the callback function if it's set
            if self.callback and topic:
                self.callback(topic, message)
            return None
        else:
            packet_type_name = MQTT_CONTROL_PACKET_TYPES.get(packet_type & 0xF0, "UNKNOWN")
            print(f"Received unknown or unsupported packet type: {packet_type_name} ({packet_type & 0xF0:02x})")
            # Consume remaining length and any data
            remaining_length = 0
            multiplier = 1
            while True:
                if self.uart.any():
                    encoded_byte = ord(self.uart.read(1))
                    remaining_length += (encoded_byte & 127) * multiplier
                    multiplier *= 128
                    if (encoded_byte & 128) == 0:
                        break
                else:
                    time.sleep(0.1)
                    continue
            self.uart.read(remaining_length)
            return None

    def process_incoming_data(self):
        """Processes incoming data from the UART and calls the callback if a message is received."""
        self._recv_packet()

    def ping(self):
        """Sends a PINGREQ message to the broker."""
        print("Sending PINGREQ message (hex): c000")
        self.uart.write(b"\xC0\x00")  # PINGREQ packet

    def send_heartbeat(self):
        """Sends a PINGREQ (heartbeat) message."""
        self.ping()

    def check_heartbeat_response(self):
        """Checks for a PINGRESP message."""
        response = self._read_uart_response(timeout=2)
        if response:
            if response[0] == 0xD0:  # Check for PINGRESP
                print("PINGRESP received")
                return True
        print("No PINGRESP received within timeout.")
        return False
        
    def _extract_publish_data(self, data):
        """
        Extracts the topic and message from a PUBLISH packet's payload.
        Returns a tuple of (topic, message).
        """
        try:
            # Extract topic length (2 bytes)
            topic_length = int.from_bytes(data[:2], 'big')

            # Extract topic
            topic_start = 2
            topic_end = topic_start + topic_length
            topic = data[topic_start:topic_end]

            # Extract message (remaining bytes)
            message_start = topic_end
            message = data[message_start:]

            return topic, message
        except Exception as e:
            print("Error extracting data from PUBLISH packet:", e)
            return None, None