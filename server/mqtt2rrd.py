import paho.mqtt.client as mqtt
import rrdtool
import json
import sys
import os

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(mqtt_topic)

def on_message(client, userdata, msg):
    try:
        print(msg.topic+" "+str(msg.payload))
    except UnicodeDecodeError, err:
        print err
        return
    jmsg = json.loads(msg.payload)
    sensors = jmsg["temp_sensors"]
    miners = jmsg["miners"]
    pwms = jmsg["pwms"]
    gpus = jmsg["gpus"]
    cmd_ds = "sensors:miners:pwms:gpus:"
    cmd_val = "N:" + str(sensors) + ":" + str(miners) + ":" + str(pwms) + ":" + str(gpus) + ":"
    for i in range(sensors):
        cmd_ds += ("s_temp" + str(i) + ":")
	cmd_val += (str(jmsg["s_temp"][i]) + ":")

    for i in range(pwms):
        cmd_ds += ("pwm" + str(i) + ":")
	cmd_val += (str(jmsg["pwm"][i]) + ":")

    for i in range(miners):
        cmd_ds += ("m_gpus" + str(i) + ":" + "m_type" + str(i) + ":"
                 + "m_temp" + str(i) + ":" + "m_hash" + str(i) + ":"
                 + "m_dhash" + str(i) + ":" + "m_acp_s" + str(i) + ":"
                 + "m_rej_s" + str(i) + ":" + "m_inc_s" + str(i) + ":"
                 + "m_uptime" + str(i) + ":" + "m_offtime" + str(i) + ":")
        cmd_val += (str(jmsg["m_gpus"][i]) + ":" + str(jmsg["m_type"][i]) + ":"
                + str(jmsg["m_temp"][i]) + ":" + str(jmsg["m_hash"][i]) + ":"
                + str(jmsg["m_dhash"][i]) + ":" + str(jmsg["m_acp_s"][i]) + ":"
                + str(jmsg["m_rej_s"][i]) + ":" + str(jmsg["m_inc_s"][i]) + ":"
                + str(jmsg["m_uptime"][i]) + ":"
                + str(jmsg["m_offtime"][i]) + ":")

    for i in range(gpus):
        cmd_ds += ("g_temp" + str(i) + ":" + "g_hash" + str(i) + ":"
                + "g_dhash" + str(i) + ":" + "g_acp_s" + str(i) + ":"
                + "g_rej_s" + str(i) + ":" + "g_inc_s" + str(i) + ":"
                + "g_fan" + str(i) + ":")
        cmd_val += (str(jmsg["g_temp"][i]) + ":" + str(jmsg["g_hash"][i]) + ":"
                + str(jmsg["g_dhash"][i]) + ":" + str(jmsg["g_acp_s"][i]) + ":"
                + str(jmsg["g_rej_s"][i]) + ":" + str(jmsg["g_inc_s"][i]) + ":"
                + str(jmsg["g_fan"][i]) + ":")

    #print(cmd_ds[0:-1])
    #print(cmd_val[0:-1])
    rrdtool.update("miner_state.rrd", "-t", cmd_ds[0:-1], cmd_val[0:-1])

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
