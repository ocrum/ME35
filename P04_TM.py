#Code to run on PC for Teachable Machine Joystick
from pyscript.js_modules import teach, mqtt_library
import asyncio


class TM_manager:
    def __init__(self):
        self.model_url = "https://teachablemachine.withgoogle.com/models/hviwwpdXy/"
        self.num_classes = 3

        self.mqtt_topic = "ME35-24/test"
        self.myClient = mqtt_library.myClient

    async def connect_mqtt(self):
        self.myClient.init()
        while not self.myClient.connected:
            await asyncio.sleep(0.1)

    async def run_model(self):
        s = teach.s
        s.URL2 = self.model_url
        await s.init()

    def send(self, message):
        print('sending ', message)
        self.myClient.publish(self.mqtt_topic, str(message))

    def get_predictions(self):
        predictions = []
        for i in range (self.num_classes):
            divElement = document.getElementById('class' + str(i))
            if divElement:
                divValue = divElement.innerHTML
                try:
                    label, value = divValue.split(': ')
                    predictions.append((label.strip(), float(value.strip())))
                except:
                    return ""
        return predictions

    async def run(self):
        threshold = 0.9 # when to be confident in a gesture
        last_sent_message = None
        while True:
            if self.myClient.connected:
                predictions = self.get_predictions()
                if predictions and len(predictions) == self.num_classes:
                    max_prediction = max(predictions, key=lambda x: x[1])
                    gesture_label, confidence = max_prediction

                    if confidence >= threshold:
                        if gesture_label == 'Thumbs Up' and last_sent_message != "start":
                            self.send("start")
                            last_sent_message = "start"
                        elif gesture_label == "Open Palm" and last_sent_message != "stop":
                            self.send("stop")
                            last_sent_message = "stop"
                    else:
                        last_sent_message = None
            await asyncio.sleep(0.5)

tm_manger = TM_manager()
await tm_manger.connect_mqtt()
await tm_manger.run_model()
await tm_manger.run()