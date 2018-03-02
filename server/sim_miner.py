import paho.mqtt.client as mqtt
import time
import json

client = mqtt.Client()

f = open("sys.cfg")
sys_cfg = json.load(f)
mqtt_server = sys_cfg.get("MQTT_SERVER")
mqtt_port = sys_cfg.get("MQTT_PORT")
mqtt_topic = sys_cfg.get("MQTT_TOPIC")
mqtt_user = sys_cfg.get("MQTT_USER")
mqtt_passwd = sys_cfg.get("MQTT_PASSWD")

if mqtt_server == None:
    mqtt_server = "localhost"

if mqtt_port == None:
    mqtt_port = 1883

if mqtt_topic == None:
    mqtt_topic = "example"

if mqtt_user != None and mqtt_passwd != None:
    client.username_pw_set(mqtt_user, mqtt_passwd)

client.connect(mqtt_server, mqtt_port, 60)

client.loop_start()

while 1:
    client.publish(mqtt_topic, "{\"temp_sensors\":3,\"temperature\":[20.00,19.00,18.00]}")
    time.sleep(10)

client.loop_stop()
client.disconnect()

