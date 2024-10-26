import paho.mqtt.client as mqtt

# MQTT Broker information
broker = "broker.emqx.io"
port = 1883
topic = "ME35-24/theremin"
client_id = "daniel_laptop"

# Callback when the client connects to the broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"\nConnected to {broker} on port {port}")
        client.subscribe(topic)
    else:
        print(f"\nFailed to connect, return code {rc}")

# Callback when a message is received from the broker
def on_message(client, userdata, msg):
    print(f"\nReceived message '{msg.payload.decode()}' from topic '{msg.topic}'")

# Callback when the client disconnects from the broker
def on_disconnect(client, userdata, rc):
    print("\nDisconnected from the broker")

# Initialize the MQTT client with the correct callback API version
client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)

# Bind callback functions
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

# Connect to the broker
client.connect(broker, port, keepalive=60)

# Start the MQTT client loop
client.loop_start()

try:
    print("Type 'w' for start, 's' for stop, 'd' for surprise, and 'q' to quit.")
    while True:
        command = input("Enter command: ").strip().lower()
        if command == 'w':
            client.publish(topic, "start")
            print("Sent 'start'")
        elif command == 's':
            client.publish(topic, "stop")
            print("Sent 'stop'")
        elif command == 'd':
            client.publish(topic, "surprise")
            print("Sent 'surprise'")
        elif command == 'a':
            client.publish(topic, "none")
            print("Sent 'none'")
        elif command == 'q':
            break
        else:
            print("Invalid input. Please use 'w', 's', 'd', or 'q'.")

except KeyboardInterrupt:
    pass

# Gracefully disconnect on exit
client.disconnect()
client.loop_stop()
print("\nExited gracefully")