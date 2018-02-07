#include <ESP8266WiFi.h>
/*
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
*/
#include <ESP8266Ping.h>

#include <SPI.h>
#include <U8g2lib.h>
#include <U8x8lib.h>
#include <Adafruit_INA219.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <pcf8574_esp.h>
#include <MQTT.h>
#include <PubSubClient.h>
#include <PubSubClient_JSON.h>
#include <ArduinoJson.h>
#include <Timer.h>

#define HOME_SSID "JDJ_HOME"
#define HOME_PASSWD "120207100425"

#define ONE_WIRE_BUS D1
#define TACHO_PWM_ADDR1 8
#define TACHO_PWM_ADDR2 9
#define TACHO_PWM_NO 2
#define TACHO_PINS_NO 11
#define TEMP_SENSOR_NO 3
#define RPM_MULTI 15

#define TEMP_FONT u8g2_font_crox4t_tn
#define RPM_FONT u8g2_font_ncenR08_tn
#define VOLT_AMP_FONT u8g2_font_timR08_tn

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Adafruit_INA219 INA219;
PCF857x pcf8574(0x27, &Wire);

//U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, D5, D7, D8);
//U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, D5, D7, D8);
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, D8);

typedef void (*timer_cb)();
typedef struct {
  int id;
  int interval;
  timer_cb callback;
} TimerTask;
void update_temp();
void update_tacho_pwm();
void update_disp();
TimerTask tasks[] = {
  {-1, 2000, update_temp},
  {-1, 2000, update_tacho_pwm},
  {-1, 2000, update_disp},
};
#define TASKS_NUM (sizeof(tasks) / sizeof(TimerTask))
Timer task_list;

float temp_val[TEMP_SENSOR_NO];
int tacho_pwm_addr[TACHO_PWM_NO] = {TACHO_PWM_ADDR1, TACHO_PWM_ADDR2};
int pwm_val[TACHO_PWM_NO];
int tacho_val[TACHO_PWM_NO][TACHO_PINS_NO];
bool curr_tacho_pwm = 0;
int volt, amp;

void setup() {
  int i;

  Serial.begin(115200);
  Serial.println("HELLO WORLD!");

  Wire.begin(D3, D2);
  pcf8574.begin();
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  sensors.begin();
  sensors.setWaitForConversion(FALSE);
  INA219.begin();
  u8g2.begin();

  for (i = 0; i < TASKS_NUM; i++) {
    tasks[i].id = task_list.every(tasks[i].interval, tasks[i].callback);
  }
}

void loop() {
  String str;
  int input_pwm_val = 0;
  WiFiClient client;
  DynamicJsonBuffer jsonBuffer1, jsonBuffer2;

/*
  delay(1000);
  Serial.println("TEST");
  return;
  */

/*
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.print(HOME_SSID);
    Serial.println("...");
    WiFi.begin(HOME_SSID, HOME_PASSWD);

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      goto out;

    Serial.println("WiFi connected");
  }

  if (client.connect("192.168.2.216", 1111)) {
    client.println("{\"id\":0,\"jsonrpc\":\"2.0\",\"method\":\"miner_getstat1\"}");
    String line = client.readStringUntil('\r');
    Serial.println(line);
    client.stop();
    JsonObject& root = jsonBuffer1.parseObject(line);
    int uptime = root["result"][1];
    Serial.println(uptime);
    //JsonArray& hashrate = root["result"][3];
    String str = root["result"][3];
    Serial.println(str);
    str.replace(';', ',');
    JsonArray& hashrate = jsonBuffer2.parseArray("["+str+"]");
    Serial.println(hashrate.size());
  } else {
    Serial.println("Cannot connect to peter-m1");
  }
*/

  /*
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.gatewayIP());
  {
    IPAddress ip (192, 168, 2, 1); // The remote ip to ping
    bool ret = Ping.ping(ip);
    int avg_time_ms = Ping.averageTime();
    long rssi;
    Serial.println(ret);
    Serial.println(avg_time_ms);
    ret = Ping.ping("peter-m1");
    avg_time_ms = Ping.averageTime();
    Serial.println(ret);
    Serial.println(avg_time_ms);
    rssi = WiFi.RSSI();
    Serial.println(rssi);
  }
  */

  task_list.update();

  while (Serial.available() > 0) {
    char s_val = 0;

    s_val = Serial.read();
    if (s_val >= 48 && s_val <= 57) {
      input_pwm_val = 10 * input_pwm_val + s_val - 48;
    }
  }

  if (input_pwm_val > 0 && input_pwm_val < 80) {
    Serial.print("I received: ");
    Serial.println(input_pwm_val);

    Wire.beginTransmission(TACHO_PWM_ADDR1);
    Wire.write(input_pwm_val);
    Wire.endTransmission();

    Wire.beginTransmission(TACHO_PWM_ADDR2);
    Wire.write(input_pwm_val);
    Wire.endTransmission();
  }

}

void update_temp() {
  int i;

  sensors.requestTemperatures();
  for (i = 0; i < TEMP_SENSOR_NO; i++) {
    temp_val[i] = sensors.getTempCByIndex(i);
    temp_val[i] = temp_val[i] < 0 ? temp_val[i] * -1 : temp_val[i];
    Serial.print(temp_val[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void update_tacho_pwm() {
  int i, j;
  unsigned char val_h, val_l;

  for (i = 0; i < TACHO_PWM_NO; i++) {
    j = 0;
    Wire.requestFrom(tacho_pwm_addr[i], TACHO_PINS_NO * 2 + 1);
    while (Wire.available()) {
      if (j == TACHO_PINS_NO) {
        pwm_val[i] = Wire.read();
        Serial.println(pwm_val[i]);
      } else {
        val_l = Wire.read();
        val_h = Wire.read();
        tacho_val[i][j] = ((val_h << 8) + val_l) * RPM_MULTI;
        Serial.print(tacho_val[i][j]);
        Serial.print(" ");
        j++;
      }
    }
  }
}

void update_disp() {
  #if 0
  u8g2.firstPage();
  do {
    u8g2.setFont(TEMP_FONT);
    u8g2.drawStr(0, 13, get_str_float(temp_val, 3, 0, 5).c_str());
    u8g2.setFont(RPM_FONT);
    u8g2.drawStr(0, 23, get_str_int(tacho_val[curr_tacho_pwm], 5, 0, 4).c_str());
    u8g2.setFont(RPM_FONT);
    u8g2.drawStr(0, 32, get_str_int(tacho_val[curr_tacho_pwm], 5, 5, 4).c_str());
    u8g2.setFont(VOLT_AMP_FONT);
    u8g2.drawStr(114, 23, get_str_int(&volt, 1, 0, 3).c_str());
    u8g2.setFont(VOLT_AMP_FONT);
    u8g2.drawStr(114, 32, get_str_int(&amp, 1, 0, 3).c_str());
  } while ( u8g2.nextPage() );
  #else
  u8g2.clearBuffer();
  u8g2.setFont(TEMP_FONT);
  get_str_float(temp_val, 3, 0, 5);
  u8g2.drawStr(0, 13, get_str_float(temp_val, 3, 0, 5).c_str());
  u8g2.setFont(RPM_FONT);
  u8g2.drawStr(0, 23, get_str_int(tacho_val[curr_tacho_pwm], 5, 0, 4).c_str());
  u8g2.setFont(RPM_FONT);
  u8g2.drawStr(0, 32, get_str_int(tacho_val[curr_tacho_pwm], 5, 5, 4).c_str());
  u8g2.setFont(VOLT_AMP_FONT);
  u8g2.drawStr(114, 23, get_str_int(&volt, 1, 0, 3).c_str());
  u8g2.setFont(VOLT_AMP_FONT);
  u8g2.drawStr(114, 32, get_str_int(&amp, 1, 0, 3).c_str());
  u8g2.sendBuffer();
  #endif
  curr_tacho_pwm = !curr_tacho_pwm;
}

String get_str_int(int *buf, int sz, int offset, int width) {
  String all_str, str;
  int i, j;

  for (i = 0; i < sz; i++) {
    str = String(buf[offset + i]);

    if (width > str.length()) {
      for (j = 0; j < width - str.length(); j++) {
        all_str += 0;
      }
    }

    all_str += str;

    if (i != sz - 1) {
      all_str += ' ';
    }
  }

  return all_str;
}

String get_str_float(float *buf, int sz, int offset, int width) {
  String all_str, str;
  int i, j;

  for (i = 0; i < sz; i++) {
    str = String(buf[offset + i]);

    if (str.length() > width) {
      str.remove(width, str.length() - width);
    } else {
      for (j = 0; j < width - str.length(); j++) {
        all_str += 0;
      }
    }
    all_str += str;

    if (i != sz - 1) {
      all_str += ' ';
    }
  }

  return all_str;
}
