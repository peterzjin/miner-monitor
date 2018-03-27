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
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Timer.h>
#include <PID_v1.h>
#include <mcurses.h>
#include <time.h>
#include <sntp.h>

#include "FS.h"

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
void send_mqtt();
TimerTask tasks[] = {
  {-1,  500, update_wifi_status},
  {-1, 2000, update_temp},
  {-1, 2000, update_tacho_pwm},
  {-1, 2000, update_disp},
  {-1, 9999, update_miners},
  {-1, 9999, memory_state},
  {-1, 9999, send_mqtt},
};
#define TASKS_NUM (sizeof(tasks) / sizeof(TimerTask))
Timer task_list;

#define MAX_GPU_PER_MINER 6
enum miner_type {
  PHOENIX,
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
  {.ip_host = "peter-m1", .port = 1111, .type = PHOENIX},
  {.ip_host = "peter-pc", .port = 2222, .type = CLAYMORE_ZEC},
  {.ip_host = "peter-pc", .port = 4444, .type = ZM_ZEC},
};
#define MINERS_NUM (sizeof(miners) / sizeof(Miner))
#define OFFLINE_TH 30
typedef struct {
  int gpu_num;
  int uptime;
  int t_hash;
  int t_dhash;
  int t_accepted_s;
  int t_rejected_s;
  int t_incorrect_s;
  int pool_switch;
  float t_temp;
  int hash[MAX_GPU_PER_MINER];
  int dhash[MAX_GPU_PER_MINER];
  int accepted_s[MAX_GPU_PER_MINER];
  int rejected_s[MAX_GPU_PER_MINER];
  int incorrect_s[MAX_GPU_PER_MINER];
  int temp[MAX_GPU_PER_MINER];
  int fan[MAX_GPU_PER_MINER];
  int last_offline;
  int offtime;
} Miner_s;
Miner_s miners_s[MINERS_NUM];

#define USE_SCREEN
#ifdef USE_SCREEN
#define dlog scr_log
enum scr_val_type {
  INT,
  DATE,
  FLOAT,
  OK_KO,
  ON_OFF,
  R_CHAR,
  STRING,
  TIME_SPAN,
  SHARE_STATE,
  FLOAT_DELTA,
};
typedef struct {
  enum scr_val_type type;
  byte y;
  byte x;
  void *val1;
  int val2;
  union {
    int val_i;
    float val_f;
    time_t val_t;
  } last;
} Scr_val;
#define MAX_SCR_VAL 120
int cur_last_scr_val, log_y, log_x;
Scr_val scr_val[MAX_SCR_VAL];
#else
#define dlog Serial.print
void setup_screen() { }
void update_scr() { }
#endif

typedef struct {
  String    ssid;
  String    passwd;
  String    mqtt_server;
  uint16_t  mqtt_port;
  String    mqtt_topic;
  String    mqtt_user;
  String    mqtt_passwd;
} Sys_cfg;
Sys_cfg sys_cfg;

#define SYS_CFG_FILE_PATH "/sys.cfg"

unsigned int system_status;

#define SYS_CFG_LOAD_BIT    0
#define SYS_WIFI_CONN_BIT   1
#define SYS_PWM_PID_BIT     2
#define SYS_MQTT_CONN_BIT   3
#define SYS_REDRAW_SCR_BIT  4
#define SYS_WIFI_CONNECTED  bitRead(system_status, SYS_WIFI_CONN_BIT)
#define SYS_CFG_LOADED      bitRead(system_status, SYS_CFG_LOAD_BIT)
#define SYS_PWM_PID_EN      bitRead(system_status, SYS_PWM_PID_BIT)
#define SYS_REDRAW_SCR      bitRead(system_status, SYS_REDRAW_SCR_BIT)

#define MAX_PWM_VAL 69
#define MIN_PWM_VAL 16
double pid_set_point, pid_input, pid_output;
PID pwm_pid(&pid_input, &pid_output, &pid_set_point, 0.8, 0, 5, DIRECT);

WiFiClient espclient;
PubSubClient mqtt_client(espclient);
float temp_val[TEMP_SENSOR_NO];
int tacho_pwm_addr[TACHO_PWM_NO] = {TACHO_PWM_ADDR1, TACHO_PWM_ADDR2};
int pwm_val[TACHO_PWM_NO];
int tacho_val[TACHO_PWM_NO][TACHO_PINS_NO];
int next_pwm_val = 0;
bool curr_tacho_pwm = 0;
int volt, amp;
int freeheap;

void setup() {
  int i;

  Serial.begin(115200);

  setup_screen();
  update_scr();

  Wire.begin(D3, D2);
  pcf8574.begin();
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  sensors.begin();
  sensors.setWaitForConversion(FALSE);
  u8g2.begin();

  for (i = 0; i < TASKS_NUM; i++) {
    tasks[i].id = task_list.every(tasks[i].interval, tasks[i].callback);
  }

  pid_set_point = 15;
  pwm_pid.SetMode(MANUAL);
  pwm_pid.SetSampleTime(16100);
  pwm_pid.SetOutputLimits(-10, 10);

  SPIFFS.begin();

  File f = SPIFFS.open(SYS_CFG_FILE_PATH, "r");
  if (!f) {
    dlog("System CFG File open failed.\r\n");
  } else {
    DynamicJsonBuffer json_buf;
    String str = f.readStringUntil('}') + '}';
    JsonObject& cfg = json_buf.parseObject(str);

    f.close();
    dlog(str + "\r\n");
    sys_cfg.ssid = String(cfg.get<char*>("SSID"));
    sys_cfg.passwd = String(cfg.get<char*>("PASSWD"));
    sys_cfg.mqtt_server = String(cfg.get<char*>("MQTT_SERVER"));
    sys_cfg.mqtt_user = String(cfg.get<char*>("MQTT_USER"));
    sys_cfg.mqtt_passwd = String(cfg.get<char*>("MQTT_PASSWD"));
    sys_cfg.mqtt_port = cfg.get<int>("MQTT_PORT");
    sys_cfg.mqtt_port = sys_cfg.mqtt_port == 0 ? 1883 : sys_cfg.mqtt_port;
    sys_cfg.mqtt_topic = String(cfg.get<char*>("MQTT_TOPIC"));
    if (!sys_cfg.mqtt_topic.length()) {
      sys_cfg.mqtt_topic = String("example");
    }

    if (sys_cfg.ssid.length()) {
      bitSet(system_status, SYS_CFG_LOAD_BIT);
      if (sys_cfg.mqtt_server.length())
        mqtt_client.setServer(sys_cfg.mqtt_server.c_str(), sys_cfg.mqtt_port);
    }
  }

  SPIFFS.end();

  sntp_init();
  sntp_setservername(0, "pool.ntp.org");
  sntp_set_timezone(8);
}

void loop() {
  bool need_set_pwm = false;

  /*
  delay(1000);
  dlog("TEST");
  return;
  */

  /*
  dlog(WiFi.localIP());
  dlog(WiFi.gatewayIP());
  {
    IPAddress ip (192, 168, 2, 1); // The remote ip to ping
    bool ret = Ping.ping(ip);
    int avg_time_ms = Ping.averageTime();
    long rssi;
    dlog(ret);
    dlog(avg_time_ms);
    ret = Ping.ping("peter-m1");
    avg_time_ms = Ping.averageTime();
    dlog(ret);
    dlog(avg_time_ms);
    rssi = WiFi.RSSI();
    dlog(rssi);
  }
  */

  task_list.update();

  while (Serial.available() > 0) {
    char s_val = 0;

    s_val = Serial.read();
    //dlog((int)s_val);
    if (s_val >= 48 && s_val <= 57) {
      next_pwm_val = 10 * next_pwm_val + s_val - 48;
    } else if (s_val == 127 /* Backspace */) {
      next_pwm_val = next_pwm_val / 10;
    } else if (s_val == 13 /* Enter */) {
      need_set_pwm = true;
    } else if (s_val == 's') {
      bitClear(system_status, SYS_PWM_PID_BIT);
      pwm_pid.SetMode(MANUAL);
      dlog("Stop the PWM PID control.\r\n");
    } else if (s_val == 'p') {
      bitSet(system_status, SYS_PWM_PID_BIT);
      pid_output = 0;
      pwm_pid.SetMode(AUTOMATIC);
      dlog("Start the PWM PID control.\r\n");
    } else if (s_val == 'd') {
      bitSet(system_status, SYS_REDRAW_SCR_BIT);
      dlog("Re-Draw the Screen.\r\n");
    } else if (s_val == 'r') {
      ESP.restart();
    }
  }

  if (SYS_PWM_PID_EN) {
    if (pwm_pid.Compute()) {
      dlog(String("Current PID input is ") + pid_input
                + ", PID output is " + pid_output + "\r\n");
      next_pwm_val = (int)(0.5 + pwm_val[0] - pid_output);
      need_set_pwm = true;
    }
  }

  if (need_set_pwm) {
    next_pwm_val = next_pwm_val > MAX_PWM_VAL ? MAX_PWM_VAL :
                  (next_pwm_val < MIN_PWM_VAL ? MIN_PWM_VAL : next_pwm_val);
    dlog(String("Get PWM value: ") + next_pwm_val + "\r\n");

    if (next_pwm_val != pwm_val[0]) {
      Wire.beginTransmission(TACHO_PWM_ADDR1);
      Wire.write(next_pwm_val);
      Wire.endTransmission();

      Wire.beginTransmission(TACHO_PWM_ADDR2);
      Wire.write(next_pwm_val);
      Wire.endTransmission();
    }
    next_pwm_val = 0;
  }
}

/* The task to update the wifi status */
void update_wifi_status() {
  if (!SYS_CFG_LOADED)
    return;

  if (WiFi.status() != WL_CONNECTED) {
    dlog("Connecting to " + sys_cfg.ssid + "...\r\n");
    WiFi.begin(sys_cfg.ssid.c_str(), sys_cfg.passwd.c_str());

    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      bitClear(system_status, SYS_WIFI_CONN_BIT);
      return;
    }

    bitSet(system_status, SYS_WIFI_CONN_BIT);

    dlog("WiFi connected\r\n");
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
    dlog(String(temp_val[i]) + " ");
  }
  pid_input = temp_val[0] - temp_val[2];
  dlog(String("delta: ") + pid_input + "\r\n");
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
        dlog(String(pwm_val[i]) + "\r\n");
      } else {
        val_l = Wire.read();
        val_h = Wire.read();
        tacho_val[i][j] = ((val_h << 8) + val_l) * RPM_MULTI;
        dlog(String(tacho_val[i][j]) + " ");
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
        + miners_s[0].t_dhash / 1000 + "m "
        + (int)miners_s[0].t_temp + "c "
        + (miners_s[0].gpu_num ? miners_s[0].uptime : miners_s[0].offtime) / 60
        + "h " + freeheap;
  u8g2.drawStr(0, 23, str.c_str());
  str = String(miners_s[1].t_hash) + "m "
        + (int)miners_s[1].t_temp + "c "
        + (miners_s[1].gpu_num ? miners_s[1].uptime : miners_s[1].offtime) / 60
        + "h " + miners_s[2].t_hash + "m "
        + (int)miners_s[2].t_temp + "c "
        + (miners_s[2].gpu_num ? miners_s[2].uptime : miners_s[2].offtime) / 60
        + "h";
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

  update_scr();
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
      if (!miners_s[i].last_offline)
        miners_s[i].last_offline = (int)(millis() / 1000);
      miners_s[i].offtime = (int)(millis() / 1000 - miners_s[i].last_offline);
      if (miners_s[i].offtime > OFFLINE_TH && miners_s[i].gpu_num != 0) {
        int offtime = miners_s[i].offtime;
        int last_offline = miners_s[i].last_offline;
        memset(&(miners_s[i]), 0, sizeof(Miner_s));
        miners_s[i].offtime = offtime;
        miners_s[i].last_offline = last_offline;
        dlog(String(miners[i].ip_host) + ":" + miners[i].port + " offline\r\n");
      }
      dlog(String("Can't connect to ") + miners[i].ip_host + ":" + miners[i].port + "\r\n");
      continue;
    }
    miners_s[i].last_offline = 0;

    switch (miners[i].type) {
      case PHOENIX:
      case CLAYMORE_ETH:
      case CLAYMORE_ZEC:
        {
          if (miners[i].type == CLAYMORE_ETH || miners[i].type == PHOENIX)
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
          miners_s[i].t_dhash = tmp_array[0];
          miners_s[i].gpu_num = str2array(result[3], ';',
              miners_s[i].hash, MAX_GPU_PER_MINER);
          str2array(result[5], ';', miners_s[i].dhash, MAX_GPU_PER_MINER);
          str2array(result[6], ';', tmp_array, MAX_GPU_PER_MINER * 2);
          for (j = 0; j < miners_s[i].gpu_num; j++) {
            miners_s[i].temp[j] = tmp_array[j * 2];
            miners_s[i].fan[j] = tmp_array[j * 2 + 1];
          }
          str2array(result[8], ';', tmp_array, MAX_GPU_PER_MINER * 2);
          miners_s[i].t_incorrect_s = tmp_array[0];
          miners_s[i].pool_switch = tmp_array[1];
          str2array(result[9], ';', miners_s[i].accepted_s, MAX_GPU_PER_MINER);
          str2array(result[10], ';', miners_s[i].rejected_s, MAX_GPU_PER_MINER);
          str2array(result[11], ';', miners_s[i].incorrect_s, MAX_GPU_PER_MINER);
        }
        break;
      case ZM_ZEC:
        {
          client.println("{\"id\":1, \"method\":\"getstat\"}");
          str = client.readStringUntil('}') + '}';
          str += client.readStringUntil('}') + '}';
          //dlog(str);
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
    miners_s[i].t_temp = average(miners_s[i].temp, miners_s[i].gpu_num);

    client.flush();
    client.stop();

    str = String(miners[i].ip_host) + ": " + miners_s[i].uptime / 60 + "H"
          + miners_s[i].uptime % 60 + "M "+ miners_s[i].gpu_num + " GPUS "
          + miners_s[i].t_hash + " Mh/s (" + miners_s[i].t_accepted_s + " "
          + miners_s[i].t_rejected_s + " " + miners_s[i].t_incorrect_s + ")";
    dlog(str + "\r\n");
    for (j = 0; j < miners_s[i].gpu_num; j++) {
      str = String("  ") + j + ": " + miners_s[i].hash[j] + " Mh/s "
            + miners_s[i].temp[j] + "C (" + miners_s[i].accepted_s[j] + " "
            + miners_s[i].rejected_s[j] + " " + miners_s[i].incorrect_s[j] + ")";
    dlog(str + "\r\n");
    }
  }
}

/* The task to track the free heap memory */
void memory_state() {
  dlog("ESP.getFreeHeap()=");
  dlog(String(freeheap = ESP.getFreeHeap()) + "\r\n");
}

/* The task to send mqtt data to the server */
void send_mqtt() {
  DynamicJsonBuffer json_buf;
  String str;
  int i, j, gpus = 0;

  if (!SYS_WIFI_CONNECTED)
    return;

  if (!mqtt_client.connected()) {
    if (!mqtt_client.connect(sys_cfg.mqtt_user.c_str(),
         sys_cfg.mqtt_user.c_str(), sys_cfg.mqtt_passwd.c_str())) {
      bitClear(system_status, SYS_MQTT_CONN_BIT);
      dlog(String("Cannot connect to the MQTT Server ")
                 + sys_cfg.mqtt_server + ':' + sys_cfg.mqtt_port + "\r\n");
      return;
    } else {
      dlog(String("Connected to the MQTT Server ")
                 + sys_cfg.mqtt_server + ':' + sys_cfg.mqtt_port + "\r\n");
    }
  }
  bitSet(system_status, SYS_MQTT_CONN_BIT);
  //dlog("MQTT connected.\r\n");

  JsonObject& root = json_buf.createObject();

  root["temp_sensors"] = TEMP_SENSOR_NO;
  JsonArray& s_temp = root.createNestedArray("s_temp");
  for (i = 0; i < TEMP_SENSOR_NO; i++)
    s_temp.add(temp_val[i]);

  root["pwms"] = TACHO_PWM_NO;
  JsonArray& pwm = root.createNestedArray("pwm");
  for (i = 0; i < TACHO_PWM_NO; i++)
    pwm.add(pwm_val[i]);

  root["miners"] = MINERS_NUM;
  JsonArray& m_gpus = root.createNestedArray("m_gpus");
  JsonArray& m_type = root.createNestedArray("m_type");
  JsonArray& m_temp = root.createNestedArray("m_temp");
  JsonArray& m_hash = root.createNestedArray("m_hash");
  JsonArray& m_dhash = root.createNestedArray("m_dhash");
  JsonArray& m_acp_s = root.createNestedArray("m_acp_s");
  JsonArray& m_rej_s = root.createNestedArray("m_rej_s");
  JsonArray& m_inc_s = root.createNestedArray("m_inc_s");
  JsonArray& m_uptime = root.createNestedArray("m_uptime");
  JsonArray& m_offtime = root.createNestedArray("m_offtime");
  root["gpus"] = gpus;
  JsonArray& g_temp = root.createNestedArray("g_temp");
  JsonArray& g_fan = root.createNestedArray("g_fan");
  JsonArray& g_hash = root.createNestedArray("g_hash");
  JsonArray& g_dhash = root.createNestedArray("g_dhash");
  JsonArray& g_acp_s = root.createNestedArray("g_acp_s");
  JsonArray& g_rej_s = root.createNestedArray("g_rej_s");
  JsonArray& g_inc_s = root.createNestedArray("g_inc_s");
  for (i = 0; i < MINERS_NUM; i++) {
    m_gpus.add(miners_s[i].gpu_num);
    m_type.add((int)(miners[i].type));
    m_temp.add(miners_s[i].t_temp);
    m_hash.add((float)(miners_s[i].t_hash > 999 ?
               (float)miners_s[i].t_hash / 1000 : miners_s[i].t_hash));
    m_dhash.add((float)(miners_s[i].t_dhash > 999 ?
                (float)miners_s[i].t_dhash / 1000 : miners_s[i].t_dhash));
    m_acp_s.add(miners_s[i].t_accepted_s);
    m_rej_s.add(miners_s[i].t_rejected_s);
    m_inc_s.add(miners_s[i].t_incorrect_s);
    m_uptime.add(miners_s[i].uptime);
    m_offtime.add(miners_s[i].offtime);
    gpus += miners_s[i].gpu_num;
    for (j = 0; j < miners_s[i].gpu_num; j++) {
      g_temp.add(miners_s[i].temp[j]);
      g_fan.add(miners_s[i].fan[j]);
      g_hash.add((float)(miners_s[i].hash[j] > 999 ?
                 (float)miners_s[i].hash[j] / 1000 : miners_s[i].hash[j]));
      g_dhash.add((float)(miners_s[i].dhash[j] > 999 ?
                  (float)miners_s[i].dhash[j] / 1000 : miners_s[i].dhash[j]));
      g_acp_s.add(miners_s[i].accepted_s[j]);
      g_rej_s.add(miners_s[i].rejected_s[j]);
      g_inc_s.add(miners_s[i].incorrect_s[j]);
    }
  }
  root["gpus"] = gpus;

  root.printTo(str);
  //dlog(str);
  //dlog(str.length());
  mqtt_client.publish(sys_cfg.mqtt_topic.c_str(), str.c_str());
}

#ifdef USE_SCREEN
void Arduino_putchar(uint8_t c) {
  Serial.write(c);
}

void scr_log(const char *str) {
  int orig_x, orig_y;
  char ch;

  if (str == NULL || *str == '\0')
    return;

  getyx(orig_y, orig_x);

  if (log_x == 0) {
    String prefix = String(" [") + millis() + "] ";
    scroll();
    mvaddstr(log_y, log_x, prefix.c_str());
    log_x += prefix.length();
  }

  while (ch = *(str++)) {
    if (ch == '\r') {
      continue;
    } else if (ch == '\n') {
      log_x = 0;
      scr_log(str);
      break;
    } else {
      mvaddch(log_y, log_x++, ch);
    }
  }

  move(orig_y, orig_x);
}

void scr_log(String str) {
  scr_log(str.c_str());
}

byte add_scr_val(enum scr_val_type type, byte y, byte x, void* val1, int val2) {
  scr_val[cur_last_scr_val].type       = type;
  scr_val[cur_last_scr_val].y          = y;
  scr_val[cur_last_scr_val].x          = x;
  scr_val[cur_last_scr_val].val1       = val1;
  scr_val[cur_last_scr_val].val2       = val2;
  scr_val[cur_last_scr_val].last.val_i = -1;
  cur_last_scr_val++;

  switch (type) {
    case INT:
    case FLOAT:
    case R_CHAR:
    case FLOAT_DELTA:
      x += val2;
      break;
    case OK_KO:
      x += 2;  /* OK or KO */
      break;
    case ON_OFF:
      x += 6;  /* ONLINE or OFLINE*/
      break;
    case TIME_SPAN:
      x += 9;  /* 99D23H59M */
      break;
    case SHARE_STATE:
      x += 18; /* 3312/1/0    99.00% */
      break;
    case STRING:
      x += strlen((char*)val1);
      break;
    case DATE:
      x += 24; /*Mon Mar 19 08:09:18 2018*/
      break;
    default:
    break;
  }

  return x;
}

/*
      Miner State  ( Mon Mar 19 08:09:18 2018 --- Mon Mar 19 08:09:18 2018 )
--------------------------------------------------------------------------------
 System | WIFI: OK | CFG: OK | PID: KO  TARGET: 00  PWM: 28/28 | MQTT: OK
 Sensor | Inlet: 18.37c | Water: 31.31c | Outlet: 31.56c | Delta: 12.94c
 Fan0   | 4095 | 4125 | 3855 | 5970 | 5820 | 5775 | 5835 | 1065 | 5745 | 0000
 Miner0 | 6 | ONLINE | 170MH/s | 36.33c | 00D23H13M | A/R/I: 3312/1/0 99.00%
--------------------------------------------------------------------------------
... ...
--------------------------------------------------------------------------------
CMD:
 */
void setup_screen() {
  int col, row = 0, i, j, offset;
  char *c_str;

  setFunction_putchar(Arduino_putchar);
  initscr();
  clear();
  cur_last_scr_val = 0;

  /* Miner State  ( Mon Mar 19 08:09:18 2018 --- Mon Mar 19 08:09:18 2018 )*/
  offset = 6;
  offset = add_scr_val(STRING, row, offset, (void *)"Miner State", 0);
  offset = add_scr_val(STRING, row, offset, (void *)"  ( ", 0);
  offset = add_scr_val(DATE, row, offset, NULL, 0);
  offset = add_scr_val(STRING, row, offset, (void *)" --- ", 0);
  offset = add_scr_val(DATE, row, offset, NULL, 1);
  offset = add_scr_val(STRING, row, offset, (void *)" )", 0);
  row++;
  add_scr_val(R_CHAR, row++, 0, (void *)'-', COLS);

  /* System | WIFI: OK | CFG: OK | PID: KO  TARGET: 00  PWM: 28/28 | MQTT: OK */
  offset = 0;
  offset = add_scr_val(STRING, row, offset, (void *)" System | WIFI: ", 0);
  offset = add_scr_val(OK_KO, row, offset, &system_status, SYS_WIFI_CONN_BIT);
  offset = add_scr_val(STRING, row, offset, (void *)" | CFG: ", 0);
  offset = add_scr_val(OK_KO, row, offset, &system_status, SYS_CFG_LOAD_BIT);
  offset = add_scr_val(STRING, row, offset, (void *)" | PID: ", 0);
  offset = add_scr_val(OK_KO, row, offset, &system_status, SYS_PWM_PID_BIT);
  offset = add_scr_val(STRING, row, offset, (void *)"  TARGET: ", 0);
  offset = add_scr_val(INT, row, offset, &pid_set_point, 2);
  offset = add_scr_val(STRING, row, offset, (void *)"  PWM: ", 0);
  offset = add_scr_val(INT, row, offset, &pwm_val[0], 2);
  offset = add_scr_val(R_CHAR, row, offset, (void *)'/', 1);
  offset = add_scr_val(INT, row, offset, &pwm_val[1], 2);
  offset = add_scr_val(STRING, row, offset, (void *)" | MQTT: ", 0);
  add_scr_val(OK_KO, row++, offset, &system_status, SYS_MQTT_CONN_BIT);

  /* Sensor | Inlet: 18.37c | Water: 31.31c | Outlet: 31.56c | Delta: 12.94c */
  offset = 0;
  offset = add_scr_val(STRING, row, offset, (void *)" Sensor | Inlet: ", 0);
  offset = add_scr_val(FLOAT, row, offset, &temp_val[2], 5);
  offset = add_scr_val(STRING, row, offset, (void *)"c | Water: ", 0);
  offset = add_scr_val(FLOAT, row, offset, &temp_val[0], 5);
  offset = add_scr_val(STRING, row, offset, (void *)"c | Outlet: ", 0);
  offset = add_scr_val(FLOAT, row, offset, &temp_val[1], 5);
  offset = add_scr_val(STRING, row, offset, (void *)"c | Delta: ", 0);
  add_scr_val(FLOAT_DELTA, row++, offset, &temp_val[0], 5);

  /* Fan0   | 4095 | 4125 | 3855 | 5970 | 5820 | 5775 | 5835 | 1065 | 5745 | 0000 */
  for (i = 0; i < TACHO_PWM_NO; i++) {
    sprintf(c_str = strdup(" Fan0  "), " Fan%d  ", i);
    offset = 0;
    offset = add_scr_val(STRING, row, offset, c_str, 0);
    for (j = 0; j < 10; j++, col += 7) {
      offset = add_scr_val(STRING, row, offset, (void *)" | ", 0);
      offset = add_scr_val(INT, row, offset, &tacho_val[i][j], 4);
    }
    row++;
  }

  /* Miner0 | 6 | ONLINE | 170MH/s | 36.33c | 00D23H13M | A/R/I: 3312/1/0 99.00% */
  for (i = 0; i < MINERS_NUM; i++) {
    sprintf(c_str = strdup(" Miner0 | "), " Miner%d | ", i);
    offset = 0;
    offset = add_scr_val(STRING, row, offset, c_str, 0);
    offset = add_scr_val(INT, row, offset, &(miners_s[i].gpu_num), 1);
    offset = add_scr_val(STRING, row, offset, (void *)" | ", 0);
    offset = add_scr_val(ON_OFF, row, offset, &(miners_s[i].gpu_num), 1);
    offset = add_scr_val(STRING, row, offset, (void *)" | ", 0);
    offset = add_scr_val(INT, row, offset, &(miners_s[i].t_hash), 3);
    offset = add_scr_val(STRING, row, offset, (void *)"MH/s | ", 0);
    offset = add_scr_val(FLOAT, row, offset, &(miners_s[i].t_temp), 5);
    offset = add_scr_val(STRING, row, offset, (void *)"c | ", 0);
    offset = add_scr_val(TIME_SPAN, row, offset, &(miners_s[i].uptime), 0);
    offset = add_scr_val(STRING, row, offset, (void *)" | A/R/I: ", 0);
    add_scr_val(SHARE_STATE, row, offset, &(miners_s[i].t_accepted_s), 0);
    row++;
  }

  add_scr_val(R_CHAR, row++, 0, (void *)'-', COLS);
  add_scr_val(R_CHAR, LINES - 2, 0, (void *)'-', COLS);

  setscrreg (row, LINES - 3);
  log_y = LINES - 3;
  log_x = 0;

  offset = add_scr_val(STRING, LINES - 1, 0, (void *)"CMD: ", 0);
  move(LINES - 1, offset);

  bitSet(system_status, SYS_REDRAW_SCR_BIT);
}

void update_scr() {
  int i, orig_x, orig_y, changed, redraw, val_i;
  float val_f;
  time_t val_t;

  getyx(orig_y, orig_x);
  redraw = SYS_REDRAW_SCR;
  bitClear(system_status, SYS_REDRAW_SCR_BIT);
  for (i = 0; i < cur_last_scr_val; i++) {
    String str;
    changed = 0;
    switch (scr_val[i].type) {
      case OK_KO:
        val_i = bitRead(*(int *)(scr_val[i].val1), scr_val[i].val2);
        if (val_i != scr_val[i].last.val_i || redraw) {
          scr_val[i].last.val_i = val_i;
          changed = 1;
          str = val_i ? String("OK") : String("KO");
        }
        break;
      case ON_OFF:
        val_i = *(int *)(scr_val[i].val1);
        if (val_i != scr_val[i].last.val_i || redraw) {
          scr_val[i].last.val_i = val_i;
          changed = 1;
          str = val_i ? String("ONLINE") : String("OFLINE");
        }
        break;
      case FLOAT:
      case FLOAT_DELTA:
        val_f = *(float *)(scr_val[i].val1) - (scr_val[i].type == FLOAT_DELTA ?
                                  *((float *)scr_val[i].val1 + 2) : 0);
        if (val_f != scr_val[i].last.val_f || redraw) {
          scr_val[i].last.val_f = val_f;
          changed = 1;
          str = get_str_float(&val_f, 1, 0, scr_val[i].val2);
        }
        break;
      case INT:
        val_i = *(int *)(scr_val[i].val1);
        if (val_i != scr_val[i].last.val_i || redraw) {
          scr_val[i].last.val_i =  val_i;
          changed = 1;
          str = get_str_int(&val_i, 1, 0, scr_val[i].val2);
        }
        break;
      case TIME_SPAN:
        val_i = *(int *)(scr_val[i].val1);
        if (val_i != scr_val[i].last.val_i || redraw) {
          int m, h, d;
          scr_val[i].last.val_i =  val_i;
          changed = 1;
          m = val_i % 60;
          val_i = val_i / 60;
          h = val_i % 24;
          d = val_i / 24;
          str = get_str_int(&d, 1, 0, 2) + 'D' + get_str_int(&h, 1, 0, 2)
                + 'H' + get_str_int(&m, 1, 0, 2) + 'M';
        }
        break;
      case SHARE_STATE:
        int acp, rej, inc;
        val_i  = acp = *((int *)scr_val[i].val1);
        val_i += rej = *((int *)scr_val[i].val1 + 1);
        val_i += inc = *((int *)scr_val[i].val1 + 2);
        if (val_i != scr_val[i].last.val_i || redraw) {
          scr_val[i].last.val_i = val_i;
          changed = 1;
          str = String(acp) + '/' + String(rej) + '/' + String(inc) + ' ';
          if (val_i) {
            float rate = ((float)acp * 100) / val_i;
            str += (get_str_float(&rate, 1, 0, 5)) + '%';
          } else {
            str += "NA";
          }
        }
        break;
      case STRING:
        if (redraw)
          mvaddstr(scr_val[i].y, scr_val[i].x, (char *)scr_val[i].val1);
        break;
      case R_CHAR:
        if (redraw) {
          int j = (int)scr_val[i].val1;
          char ch = (char)j;
          move(scr_val[i].y, scr_val[i].x);
          for (j = 0; j < scr_val[i].val2; j++)
            addch(ch);
        }
        break;
      case DATE:
        val_t = time(NULL) - (scr_val[i].val2 ? 0 : millis() / 1000);
        if (val_t != scr_val[i].last.val_t || redraw) {
          scr_val[i].last.val_t = val_t;
          str = ctime(&val_t);
          changed = 1;
        }
        break;
      default:
        break;
    }
    if (changed)
      mvaddstr(scr_val[i].y, scr_val[i].x, str.c_str());
  }

  move(orig_y, orig_x);
}
#endif

String get_str_int(int *buf, int sz, int offset, int width) {
  String all_str, str;
  int i, j;

  for (i = 0; i < sz; i++) {
    str = String(buf[offset + i]);

    if (width < str.length()) {
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

float average(int *buf, int num) {
  int i;
  float sum = 0;

  if (num <= 0)
    return 0;

  for (i = 0; i < num; i++)
    sum += buf[i];

  return (sum / num);
}

