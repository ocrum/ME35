# Code to run on PC for Teachable Machine Joystick
from pyscript.js_modules import teach, pose, ble_library, mqtt_library
import asyncio

# Initialize MQTT client and topics
myClient = mqtt_library.myClient("broker.hivemq.com", 8884)
# myClient = mqtt_library.myClient("broker.emqx.io", 1883)
mqtt_connected = False
pub_topic = 'ME35-24/theremin'

async def received_mqtt_msg(message):
    message = myClient.read().split('	')  # Process received messages here

async def run_model(URL2):
    s = teach.s  # or s = pose.s
    s.URL2 = URL2
    await s.init()

async def connect(name):
    global mqtt_connected
    myClient.init()
    while not myClient.connected:
        await asyncio.sleep(2)
    myClient.callback = received_mqtt_msg
    mqtt_connected = True
    print("connected")

async def disconnect():
    print('disconnected')

def send(message):
    print('sending ', message)
    myClient.publish(pub_topic, message)

def get_predictions(num_classes):
    predictions = []
    for i in range(num_classes):
        divElement = document.getElementById('class' + str(i))
        if divElement:
            divValue = divElement.innerHTML
            try:
                label, value = divValue.split(': ')
                predictions.append((label.strip(), float(value.strip())))
            except:
                return ""
    return predictions

# Main loop to process predictions and send messages
async def run():
    threshold = 0.9  # Confidence threshold
    last_sent_message = None
    num_classes = 2  # Adjust for your model
    while True:
        if mqtt_connected:
            predictions = get_predictions(num_classes)
            if predictions and len(predictions) == num_classes:
                max_prediction = max(predictions, key=lambda x: x[1])
                gesture_label, confidence = max_prediction

                if confidence >= threshold:
                    if gesture_label == 'suprise' and last_sent_message != 'surprise':
                        send('surprise')
                        last_sent_message = 'surprise'
                    elif gesture_label == 'none' and last_sent_message != 'none':
                        send('none')
                        last_sent_message = 'none'
                else:
                    last_sent_message = None
        await asyncio.sleep(0.5)

# Run the model and MQTT connection
await run_model("https://teachablemachine.withgoogle.com/models/Qx6FSnij_/")  # Change to your model link
await connect('daniel1')
await run()