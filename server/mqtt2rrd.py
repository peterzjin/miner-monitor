import paho.mqtt.client as mqtt
import json
import sys
import os

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(mqtt_topic)

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    jmsg = json.loads(msg.payload)
    sensors = jmsg["temp_sensors"]
    miners = jmsg["miners"]
    cmd = "rrdtool update miner_state.rrd -t "
    cmd_t = ""
    cmd_N = ""
    for i in range(sensors):
        cmd_t += ("s_temp" + str(i) + ":")
	cmd_N += (str(jmsg["s_temp"][i]) + ":")

    for i in range(miners):
        cmd_t += ("m_temp" + str(i) + ":" + "m_hash" + str(i) + ":")
        cmd_N += (str(jmsg["m_temp"][i]) + ":" + str(jmsg["m_hash"][i]) + ":")

    cmd += cmd_t[0:-1] + " N:" + cmd_N[0:-1]

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
