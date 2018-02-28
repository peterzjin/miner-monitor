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
#include <DallasTemperature.h>
#include <Wire.h>
#include <pcf8574_esp.h>
#include <MQTT.h>
#include <PubSubClient.h>
#include <PubSubClient_JSON.h>
#include <ArduinoJson.h>
#include <Timer.h>

#define HOME_SSID "tomato-h118b"
#define HOME_PASSWD "1859132301"

#define ONE_WIRE_BUS D1
#define TACHO_PWM_ADDR1 8
#define TACHO_PWM_ADDR2 9
#define TACHO_PWM_NO 2
#define TACHO_PINS_NO 11
#define TEMP_SENSOR_NO 3
#define RPM_MULTI 15

#define TEMP_FONT u8g2_font_crox4t_tn
#define RPM_FONT u8g2_font_ncenR08_tn
#define MINER_FONT u8g2_font_ncenR08_tr
#define VOLT_AMP_FONT u8g2_font_timR08_tn

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
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
void update_wifi_status();
void update_temp();
void update_tacho_pwm();
void update_disp();
void update_miners();
void memory_state();
TimerTask tasks[] = {
  {-1,  500, update_wifi_status},
  {-1, 2000, update_temp},
  {-1, 2000, update_tacho_pwm},
  {-1, 2000, update_disp},
  {-1, 9999, update_miners},
  {-1, 9999, memory_state},
};
#define TASKS_NUM (sizeof(tasks) / sizeof(TimerTask))
Timer task_list;

#define MAX_GPU_PER_MINER 6
enum miner_type {
  CLAYMORE_ETH_DUAL,
  CLAYMORE_ETH,
  CLAYMORE_ZEC,
  ZM_ZEC,
};
typedef struct {
  char *ip_host;
  int  port;
  enum miner_type type;
} Miner;
Miner miners[] = {
  {.ip_host = "peter-m1", .port = 1111, .type = CLAYMORE_ETH_DUAL},
  {.ip_host = "peter-pc", .port = 2222, .type = CLAYMORE_ZEC},
  {.ip_host = "peter-pc", .port = 4444, .type = ZM_ZEC},
};
#define MINERS_NUM (sizeof(miners) / sizeof(Miner))
typedef struct {
  int gpu_num;
  int uptime;
  int t_hash;
  int t_d_hash;
  int t_accepted_s;
  int t_rejected_s;
  int t_invalid_s;
  int pool_switch;
  int hash[MAX_GPU_PER_MINER];
  int accepted_s[MAX_GPU_PER_MINER];
  int rejected_s[MAX_GPU_PER_MINER];
  int invalid_s[MAX_GPU_PER_MINER];
  int temp[MAX_GPU_PER_MINER];
  int fan[MAX_GPU_PER_MINER];
} Miner_s;
Miner_s miners_s[MINERS_NUM];

unsigned int system_status;

#define SYS_WIFI_CONN_BIT   0
#define SYS_WIFI_CONNECTED  bitRead(system_status, SYS_WIFI_CONN_BIT)

float temp_val[TEMP_SENSOR_NO];
int tacho_pwm_addr[TACHO_PWM_NO] = {TACHO_PWM_ADDR1, TACHO_PWM_ADDR2};
int pwm_val[TACHO_PWM_NO];
int tacho_val[TACHO_PWM_NO][TACHO_PINS_NO];
bool curr_tacho_pwm = 0;
int volt, amp;
int freeheap;

void setup() {
  int i;

  Serial.begin(115200);
  Serial.println("HELLO WORLD!");

  Wire.begin(D3, D2);
  pcf8574.begin();
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  sensors.begin();
  sensors.setWaitForConversion(FALSE);
  u8g2.begin();

  for (i = 0; i < TASKS_NUM; i++) {
    tasks[i].id = task_list.every(tasks[i].interval, tasks[i].callback);
  }
}

void loop() {
  String str;
  int input_pwm_val = 0;
  WiFiClient client;

  /*
  delay(1000);
  Serial.println("TEST");
  return;
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

/* The task to update the wifi status */
void update_wifi_status() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.print(HOME_SSID);
    Serial.println("...");
    WiFi.begin(HOME_SSID, HOME_PASSWD);

    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      bitClear(system_status, SYS_WIFI_CONN_BIT);
      return;
    }

    bitSet(system_status, SYS_WIFI_CONN_BIT);

    Serial.println("WiFi connected");
  } else {
    bitSet(system_status, SYS_WIFI_CONN_BIT);
  }
}

/* The task to get the temperature from sensors */
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

/* The task to get the tacho and pwm values from tachometers */
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

/* The task to draw the display */
void update_disp() {
  String str;
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
  u8g2.drawStr(0, 13, get_str_float(temp_val, 3, 0, 5).c_str());
  u8g2.setFont(MINER_FONT);
  str = String(miners_s[0].t_hash / 1000) + "m "
        + miners_s[0].t_d_hash / 1000 + "m "
        + average(miners_s[0].temp, miners_s[0].gpu_num) + "c "
        + miners_s[0].uptime / 60 + "h " + freeheap;
  u8g2.drawStr(0, 23, str.c_str());
  str = String(miners_s[1].t_hash) + "m "
        + average(miners_s[1].temp, miners_s[1].gpu_num) + "c "
        + miners_s[1].uptime / 60 + "h " + miners_s[2].t_hash + "m "
        + average(miners_s[2].temp, miners_s[2].gpu_num) + "c "
        + miners_s[2].uptime / 60 + "h";
  u8g2.drawStr(0, 32, str.c_str());
  /*
  u8g2.setFont(RPM_FONT);
  u8g2.drawStr(0, 23, get_str_int(tacho_val[curr_tacho_pwm], 5, 0, 4).c_str());
  u8g2.setFont(RPM_FONT);
  u8g2.drawStr(0, 32, get_str_int(tacho_val[curr_tacho_pwm], 5, 5, 4).c_str());
  u8g2.setFont(VOLT_AMP_FONT);
  u8g2.drawStr(114, 23, get_str_int(&volt, 1, 0, 3).c_str());
  u8g2.setFont(VOLT_AMP_FONT);
  u8g2.drawStr(114, 32, get_str_int(&amp, 1, 0, 3).c_str());
  */
  u8g2.sendBuffer();
  #endif
}

/* The task to get the info from miners */
void update_miners() {
  String str;
  WiFiClient client;
  DynamicJsonBuffer json_buf;
  int i, j, tmp_array[MAX_GPU_PER_MINER * 2];

  if (!SYS_WIFI_CONNECTED)
    return;

  for (i = 0; i < MINERS_NUM; i++) {
    if (!client.connect(miners[i].ip_host, miners[i].port)) {
      Serial.print("Cannot connect to ");
      Serial.print(miners[i].ip_host);
      Serial.print(":");
      Serial.println(miners[i].port);
      continue;
    }

    switch (miners[i].type) {
      case CLAYMORE_ETH_DUAL:
      case CLAYMORE_ETH:
      case CLAYMORE_ZEC:
        {
          if (miners[i].type == CLAYMORE_ETH_DUAL
              || miners[i].type == CLAYMORE_ETH)
            client.println(
              "{\"id\":0,\"jsonrpc\":\"2.0\",\"method\":\"miner_getstat2\"}");
          else
            client.println(
              "{\"id\":0,\"jsonrpc\":\"2.0\",\"method\":\"miner_getstat1\"}");
          str = client.readStringUntil('}') + '}';
          JsonArray& result = (json_buf.parseObject(str))["result"];
          miners_s[i].uptime = result[1];
          str2array(result[2], ';', tmp_array, 3);
          miners_s[i].t_hash = tmp_array[0];
          miners_s[i].t_accepted_s = tmp_array[1];
          miners_s[i].t_rejected_s = tmp_array[2];
          str2array(result[4], ';', tmp_array, 3);
          miners_s[i].t_d_hash = tmp_array[0];
          miners_s[i].gpu_num = str2array(result[3], ';',
              miners_s[i].hash, MAX_GPU_PER_MINER);
          str2array(result[6], ';', tmp_array, MAX_GPU_PER_MINER * 2);
          for (j = 0; j < miners_s[i].gpu_num; j++) {
            miners_s[i].temp[j] = tmp_array[j * 2];
            miners_s[i].fan[j] = tmp_array[j * 2 + 1];
          }
          str2array(result[8], ';', tmp_array, MAX_GPU_PER_MINER * 2);
          miners_s[i].t_invalid_s = tmp_array[0];
          miners_s[i].pool_switch = tmp_array[1];
          str2array(result[9], ';', miners_s[i].accepted_s, MAX_GPU_PER_MINER);
          str2array(result[10], ';', miners_s[i].rejected_s, MAX_GPU_PER_MINER);
          str2array(result[11], ';', miners_s[i].invalid_s, MAX_GPU_PER_MINER);
        }
        break;
      case ZM_ZEC:
        {
          client.println("{\"id\":1, \"method\":\"getstat\"}");
          str = client.readStringUntil('}') + '}';
          str += client.readStringUntil('}') + '}';
          //Serial.println(str);
          JsonObject& root = json_buf.parseObject(str);
          miners_s[i].uptime = ((int)root["uptime"]) / 60;
          JsonArray& result = root["result"];
          miners_s[i].gpu_num = result.size();
          miners_s[i].t_hash = 0;
          miners_s[i].t_accepted_s = 0;
          miners_s[i].t_rejected_s = 0;
          for (j = 0; j < miners_s[i].gpu_num; j++) {
            miners_s[i].temp[j] = result[j]["temperature"];
            miners_s[i].hash[j] = result[j]["sol_ps"];
            miners_s[i].t_hash += miners_s[i].hash[j];
            miners_s[i].accepted_s[j] = result[j]["accepted_shares"];
            miners_s[i].t_accepted_s += miners_s[i].accepted_s[j];
            miners_s[i].rejected_s[j] = result[j]["rejected_shares"];
            miners_s[i].t_rejected_s += miners_s[i].rejected_s[j];
          }
        }
        break;
      default:
        break;
    }

    client.flush();
    client.stop();

    str = String(miners[i].ip_host) + ": " + miners_s[i].uptime / 60 + "H"
          + miners_s[i].uptime % 60 + "M "+ miners_s[i].gpu_num + " GPUS "
          + miners_s[i].t_hash + " Mh/s (" + miners_s[i].t_accepted_s + " "
          + miners_s[i].t_rejected_s + " " + miners_s[i].t_invalid_s + ")";
    Serial.println(str);
    for (j = 0; j < miners_s[i].gpu_num; j++) {
      str = String("  ") + j + ": " + miners_s[i].hash[j] + " Mh/s "
            + miners_s[i].temp[j] + "C (" + miners_s[i].accepted_s[j] + " "
            + miners_s[i].rejected_s[j] + " " + miners_s[i].invalid_s[j] + ")";
    Serial.println(str);
    }
  }
}

void memory_state() {
  Serial.print("ESP.getFreeHeap()=");
  Serial.println(freeheap = ESP.getFreeHeap());
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

int str2array(String str, char sep, int *array, int max_size) {
  int p1 = 0, p2 = 0, i = 0, len = str.length();

  while (1) {
    if (-1 == (p2 = str.indexOf(sep, p1))) {
      array[i++] = str.substring(p1).toInt();
      break;
    }

    if (p1 != p2)
      array[i++] = str.substring(p1, p2).toInt();

    if (i >= max_size)
      break;

    p1 = p2 + 1;
  }

  return i;
}

int average(int *buf, int num) {
  int i, sum = 0;

  if (num <= 0)
    return 0;

  for (i = 0; i < num; i++)
    sum += buf[i];

  return (sum / num);
}

