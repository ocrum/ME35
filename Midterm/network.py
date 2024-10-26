import network
import mqtt
import secrets
import time

class NetworkManager:
    """
    Manages Wi-Fi connection and MQTT communication.
    Handles subscribing to topics and processing incoming messages.
    """
    def __init__(self, sub='', pub='',client_id=''):
        """
        Initialize the NetworkManager.
        Sets up Wi-Fi, MQTT client
        """
        self.wlan = network.WLAN(network.STA_IF)

        # TODO;daniel; what is going on with my personal broker stuff? Should I just use this lmaoo
        # like i can't transfer the stuff in the example code because i don't know what micropython can even take
        self.using_mqtt = False
        self.using_internet = False
        self.message_used = True
        self.mqtt_broker = 'broker.emqx.io'
        self.user = 'daniel'
        self.password = '<PASSWORD>'
        self.port = 1883
        self.topic_sub = 'ME35-24/theremin'
        self.topic_pub = 'ME35-24/theremin_out'
        self.client_id = 'daniel_pico'
        self.client = mqtt.MQTTClient(self.client_id,self.mqtt_broker, self.port, keepalive=60)

    def connect_to_internet(self):
        """
        Connects to the Wi-Fi using credentials from the secrets file.
        Waits for connection up to 10 seconds, then returns success/failure.
        :return: True if connected, False otherwise
        """
        self.wlan.active(True)
        ret = False
        if self.wlan.isconnected() and True:
            print(f'Network connected, IP {self.wlan.ifconfig()[0]}')
            self.using_internet = True
            ret = True
        else:
            self.wlan.connect(secrets.mysecrets['SSID'], secrets.mysecrets['key'])
            print('Connecting to network...')

            max_wait = 10  # 10-second timeout
            while max_wait > 0 and not self.using_internet:
                # status_message = self.get_status_message()
                # print(status_message)

                if self.wlan.isconnected():
                    print(f'Network connected, IP {self.wlan.ifconfig()[0]}')
                    self.using_internet = True
                    ret = True

                time.sleep(1)
                max_wait -= 1
        if ret is False:
            print('Failed to connect to the network.')
        return ret

    def callback(self, topic, msg):
        """
        Callback function for MQTT.
        :param topic: The topic of the MQTT message
        :param msg: The message payload
        """
        print(topic.decode(), msg.decode())
        msg = msg.decode()

        self.curr_callback = msg
        self.message_used = False

    def connect_to_MQTT(self):
        """
        Connects to the MQTT broker and subscribes to a specified topic.
        Also sets the MQTT callback to the NetworkManager's callback function.
        """
        try:
            self.client.connect()
            print(f'Connected to {self.mqtt_broker}s MQTT')
            self.client.set_callback(self.callback)
            self.client.subscribe(self.topic_sub.encode())
            self.using_mqtt = True
        except Exception as e:
            print("Didn't connect to MQTT")
            time.sleep(1)

    def check_callback(self):
        """
        Checks for incoming MQTT messages and processes the callback.
        """
        if self.using_mqtt:
            try:
                self.client.check_msg()
                if self.message_used is False:
                    self.message_used = True
                    return self.curr_callback
            except Exception as e:
                print('MQTT callback failed')
                self.using_mqtt = False
                self.connect_to_MQTT()
                return None

    def publish_message(self, message):
        """
        Publishes a message to the MQTT broker on the subscribed topic.
        """
        if self.using_mqtt:
            self.client.publish(self.topic_pub.encode(), str(message).encode())
        else:
            print('MQTT is not connected')
            self.using_mqtt = False
            self.connect_to_MQTT()
