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
#include <MQTT.h>
#include <PubSubClient.h>
#include <PubSubClient_JSON.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <pcf8574_esp.h>

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

float temp_val[TEMP_SENSOR_NO];
int tacho_pwm_addr[TACHO_PWM_NO] = {TACHO_PWM_ADDR1, TACHO_PWM_ADDR2};
int pwm_val[TACHO_PWM_NO];
int tacho_val[TACHO_PWM_NO][TACHO_PINS_NO];
bool curr_tacho_pwm = 0;
int volt, amp;

void setup() {
  Serial.begin(115200);
  Serial.println("HELLO WORLD!");

  Wire.begin(D3, D2);
  pcf8574.begin();
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  sensors.begin();
  sensors.setWaitForConversion(FALSE);
  INA219.begin();
  u8g2.begin();
}

void loop() {
  int input_pwm_val = 0, i, j;
  unsigned char val_h, val_l;
  unsigned long t1, t2, t3, t4, t5;
  String str;

/*
  delay(1000);
  Serial.println("TEST");
  return;
  */

  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.print(HOME_SSID);
    Serial.println("...");
    WiFi.begin(HOME_SSID, HOME_PASSWD);

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      goto out;

    Serial.println("WiFi connected");
  }

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

  t1 = millis();
  sensors.requestTemperatures();
  for (i = 0; i < TEMP_SENSOR_NO; i++) {
    temp_val[i] = sensors.getTempCByIndex(i);
    temp_val[i] = temp_val[i] < 0 ? temp_val[i] * -1 : temp_val[i];
    Serial.print(temp_val[i]);
    Serial.print(" ");
  }
  Serial.println();

  t2 = millis();
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

  t3 = millis();
/*
  volt = INA219.getBusVoltage_V() * 10;
  amp = abs(INA219.getCurrent_mA()) / 10;
*/

  t4 = millis();
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

  t5 = millis();

#if 0
  str = t1;
  str += " (";
  str += (t2 - t1);
  str += ") ";
  str += t2;
  str += " (";
  str += (t3 - t2);
  str += ") ";
  str += t3;
  str += " (";
  str += (t4 - t3);
  str += ") ";
  str += t4;
  str += " (";
  str += (t5 - t4);
  str += ") ";
  str += t5;
  Serial.println(str);
#endif

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

out:
  delay(2000);
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
