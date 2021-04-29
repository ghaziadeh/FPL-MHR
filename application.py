import paho.mqtt.client as mqtt
def connectionStatus(client, userdata, flags, rc):
    mqttClient.subscribe("rpi/gpio")

def messageDecoder(client, userdata, msg):
    message = msg.payload.decode(encoding='UTF-8')
    print(message)
    try:
        msg=int(message)
        print(f'slider value of:{msg}')
        f = open("speed.txt","w+")
        f.write(str(message))
        f.close()
    except ValueError:
        f2 = open("application_control.txt","w+")
        f2.write(str(message))
        f2.close()

clientName = "FPL"
print("testinggg")
#serverAddress = "100.64.13.57"
matthew = "172.20.10.8"
Navid = "172.20.10.6"
serverAddress = "172.20.10.6"
mqttClient = mqtt.Client(clientName)
mqttClient.on_connect = connectionStatus
mqttClient.on_message = messageDecoder
mqttClient.connect(serverAddress)
mqttClient.loop_forever()
