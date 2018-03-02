import paho.mqtt.client as mqtt
import json
import sys
import os

#rrdtool create miner_state.rrd --step 10 DS:temp1:GAUGE:20:0:100 DS:temp2:GAUGE:20:0:100 DS:temp3:GAUGE:20:0:100 RRA:AVERAGE:0.5:1:8640

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(mqtt_topic)

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    jmsg = json.loads(msg.payload)
    cmd="rrdtool update miner_state.rrd N:"+str(jmsg["temperature"][0])+":"+str(jmsg["temperature"][1])+":"+str(jmsg["temperature"][2])
    print(cmd)
    os.system(cmd)

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

client.on_connect = on_connect
client.on_message = on_message

client.connect(mqtt_server, mqtt_port, 60)

client.loop_forever()
