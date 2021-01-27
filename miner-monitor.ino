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
#include <ESP8266WebServer.h>
#include <ESPAsyncTCP.h>

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

#define TTP_PINS 11
#define RPM_MULTI 15

#define TEMP_FONT u8g2_font_crox4t_tn
#define RPM_FONT u8g2_font_ncenR08_tn
#define MINER_FONT u8g2_font_ncenR08_tr
#define VOLT_AMP_FONT u8g2_font_timR08_tn

OneWire oneWire;
DallasTemperature sensors;
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
void update_state_json();
void memory_state();
void send_mqtt();
TimerTask tasks[] = {
  {-1,  500, update_wifi_status},
  {-1, 2000, update_temp},
  {-1, 2000, update_tacho_pwm},
  {-1, 2000, update_disp},
  {-1, 9999, update_miners},
  {-1, 9999, update_state_json},
  {-1, 9999, memory_state},
  {-1, 9999, send_mqtt},
};
#define TASKS_NUM (sizeof(tasks) / sizeof(TimerTask))
Timer task_list;

#define USE_ASYNC_TCP
#define MAX_GPU_PER_MINER 9
enum miner_type {
  PHOENIX,
  CLAYMORE_ETH,
  CLAYMORE_ZEC,
  ZM_ZEC,
};
typedef struct {
  char *ip_host;
  uint16_t port;
  short enabled;
  enum miner_type type;
#ifdef USE_ASYNC_TCP
  AsyncClient *asc;
#endif
} Miner;
Miner *miners;
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
Miner_s *miners_s;

#define SCREEN_LINES 60
#define MAX_SCR_VAL 160
enum disp_type {
  SERIAL_SCREEN,
  ST7920_128X64,
  MAX_PANEL_TYPE,
};
enum disp_val_type {
  INT,
  LOG,
  DATE,
  FLOAT,
  OK_KO,
  ON_OFF,
  R_CHAR,
  STRING,
  MACADR,
  IPADDR,
  TIME_SPAN,
  SHARE_STATE,
  FLOAT_DELTA,
};
typedef struct {
  enum disp_val_type type;
  uint16_t y;
  uint16_t x;
  void *val1;
  int val2;
  union {
    int val_i;
    float val_f;
    time_t val_t;
    uint32_t val_u;
  } last;
} Disp_val;
typedef struct _Disp_outp Disp_outp;
typedef void (*Disp_log)(Disp_outp *outp, const char *str);
typedef void (*Disp_render)(Disp_outp *outp);
typedef void (*Disp_init)(Disp_outp *outp);
struct _Disp_outp {
  enum disp_type type;
  int            last_val;
  Disp_val       disp_val[MAX_SCR_VAL];
  Disp_render    render;
  Disp_init      init;
};
void init_scr(Disp_outp *outp);
void init_u8g2(Disp_outp *outp);
void update_scr(Disp_outp *outp);
void update_u8g2(Disp_outp *outp);
Disp_init   d_inits[]   = {init_scr, init_u8g2};
Disp_render d_renders[] = {update_scr, update_u8g2};

#define RESET_PIN 6
#define BOOT_PIN  7

String state_json_str;
typedef struct {
  String    ssid;
  String    passwd;
  String    hostname;
  bool      mqtt_enabled;
  String    mqtt_server;
  uint16_t  mqtt_port;
  String    mqtt_topic;
  String    mqtt_user;
  String    mqtt_passwd;
  bool      sensors_enabled;
  uint8_t   sensors_num;
  uint8_t   *sensors_map;
  uint8_t   sensors_pin;
  bool      ttp_enabled;
  uint8_t   ttp_num;
  uint8_t   *ttp_addr;
  bool      display_enabled;
  int       display_num;
  Disp_outp *display_output;
  bool      ext_gpio_enabled;
  int       miners_num;
} Sys_cfg;
Sys_cfg sys_cfg;

#define SYS_CFG_FILE_PATH "/sys.cfg"

typedef struct {
  uint32_t  flags;
  uint32_t  **rpm;
  uint32_t  *pwm;
  float     *temp;
  int       freeheap;
  uint8_t   bssid[6];
  uint32_t  localip;
  int       rssi;
} Sys_stat;
Sys_stat sys_stat;

#define SYS_CFG_LOAD_BIT    0
#define SYS_WIFI_CONN_BIT   1
#define SYS_PWM_PID_BIT     2
#define SYS_MQTT_CONN_BIT   3
#define SYS_REDRAW_SCR_BIT  4
#define SYS_NEED_RESET_BIT  5
#define SYS_NEED_BOOT_BIT   6
#define SYS_WIFI_CONNECTED  bitRead(sys_stat.flags, SYS_WIFI_CONN_BIT)
#define SYS_CFG_LOADED      bitRead(sys_stat.flags, SYS_CFG_LOAD_BIT)
#define SYS_PWM_PID_EN      bitRead(sys_stat.flags, SYS_PWM_PID_BIT)
#define SYS_REDRAW_SCR      bitRead(sys_stat.flags, SYS_REDRAW_SCR_BIT)
#define SYS_NEED_RESET      bitRead(sys_stat.flags, SYS_NEED_RESET_BIT)
#define SYS_NEED_BOOT       bitRead(sys_stat.flags, SYS_NEED_BOOT_BIT)

#define MAX_PWM_VAL 69
#define MIN_PWM_VAL 16
double pid_set_point, pid_input, pid_output;
PID pwm_pid(&pid_input, &pid_output, &pid_set_point, 0.8, 0, 5, DIRECT);

WiFiClient espclient;
ESP8266WebServer ws(80);
PubSubClient mqtt_client(espclient);
// 0: Water 1: Outlet Air 2: Inlet Air
int next_pwm_val = 0;
//bool curr_tacho_pwm = 0;

void setup() {
  int i;
  String cfg_str;
  DynamicJsonDocument cfg(2048);

  Serial.begin(115200);

  pid_set_point = 15;
  pwm_pid.SetMode(MANUAL);
  pwm_pid.SetSampleTime(16100);
  pwm_pid.SetOutputLimits(-10, 10);

  SPIFFS.begin();

  File f = SPIFFS.open(SYS_CFG_FILE_PATH, "r");
  if (!f) {
    dlog("System CFG File open failed.\r\n");
  } else {
    f.setTimeout(1);
    cfg_str = f.readString();
    deserializeJson(cfg, cfg_str);

    f.close();
    memset(&sys_cfg, 0, sizeof(Sys_cfg));
    sys_cfg.ssid = cfg["SSID"].as<String>();
    sys_cfg.passwd = cfg["PASSWD"].as<String>();
    sys_cfg.hostname = cfg["HOSTNAME"].as<String>();

    if (!sys_cfg.hostname.length())
      sys_cfg.hostname = String("esp8266");

    if (sys_cfg.ssid.length())
      bitSet(sys_stat.flags, SYS_CFG_LOAD_BIT);

    JsonObject json_mqtt = cfg["MQTT"];
    sys_cfg.mqtt_enabled = json_mqtt["ENABLED"].as<bool>();
    sys_cfg.mqtt_server = json_mqtt["SERVER"].as<String>();
    sys_cfg.mqtt_user = json_mqtt["USER"].as<String>();
    sys_cfg.mqtt_passwd = json_mqtt["PASSWD"].as<String>();
    sys_cfg.mqtt_port = json_mqtt["PORT"].as<uint16_t>();
    sys_cfg.mqtt_port = sys_cfg.mqtt_port == 0 ? 1883 : sys_cfg.mqtt_port;
    sys_cfg.mqtt_topic = json_mqtt["TOPIC"].as<String>();

    if (!sys_cfg.mqtt_topic.length())
      sys_cfg.mqtt_topic = String("example");
    if (sys_cfg.mqtt_server.length())
      mqtt_client.setServer(sys_cfg.mqtt_server.c_str(), sys_cfg.mqtt_port);

    JsonObject json_sensors = cfg["SENSORS"];
    sys_cfg.sensors_pin = str2pin(json_sensors["PIN"].as<String>());
    sys_cfg.sensors_enabled = sys_cfg.sensors_pin == 0xFF ?
                              false : json_sensors["ENABLED"].as<bool>();
    JsonArray json_sm = json_sensors["MAP"];
    sys_cfg.sensors_num = json_sm.size();
    if (sys_cfg.sensors_num > 0) {
      sys_cfg.sensors_map = (uint8_t *)malloc(sys_cfg.sensors_num * sizeof(uint8_t));
      sys_stat.temp = (float *)malloc(sys_cfg.sensors_num * sizeof(float));
      for (i = 0; i < sys_cfg.sensors_num; i++) {
        sys_cfg.sensors_map[i] = json_sm[i].as<uint8_t>();
        // invalid mapping, ignore the sensors
        if (sys_cfg.sensors_map[i] >= sys_cfg.sensors_num)
          sys_cfg.sensors_num = 0;
      }
    }

    JsonObject json_ttp = cfg["TTP"];
    sys_cfg.ttp_enabled = json_ttp["ENABLED"].as<bool>();
    JsonArray json_ta = json_ttp["ADDR"];
    sys_cfg.ttp_num = json_ta.size();
    if (sys_cfg.ttp_num > 0) {
      sys_cfg.ttp_addr = (uint8_t *)malloc(sys_cfg.ttp_num * sizeof(uint8_t));
      sys_stat.rpm = (uint32_t **)malloc(sys_cfg.ttp_num * sizeof(uint32_t *));
      sys_stat.pwm = (uint32_t *)malloc(sys_cfg.ttp_num * sizeof(uint32_t));
      for (i = 0; i < sys_cfg.ttp_num; i++) {
        sys_cfg.ttp_addr[i] = json_ta[i].as<uint8_t>();
        sys_stat.rpm[i] = (uint32_t *)malloc((TTP_PINS - 1) * sizeof(uint32_t));
      }
    }

    JsonObject json_display = cfg["DISPLAY"];
    sys_cfg.display_enabled = json_display["ENABLED"].as<bool>();
    JsonArray json_do = json_display["OUTPUT"];
    for (i = 0; i < json_do.size(); i++) {
      if (json_do[i].as<int>() >= MAX_PANEL_TYPE)
        continue;

      sys_cfg.display_num++;
      sys_cfg.display_output = (Disp_outp *)realloc(sys_cfg.display_output,
                                sys_cfg.display_num * sizeof(Disp_outp));
      memset(&sys_cfg.display_output[i], 0, sizeof(Disp_outp));
      sys_cfg.display_output[i].type   = (enum disp_type)json_do[i].as<int>();
      sys_cfg.display_output[i].render = d_renders[sys_cfg.display_output[i].type];
      sys_cfg.display_output[i].init   = d_inits[sys_cfg.display_output[i].type];
    }

    JsonObject json_ext_gpio = cfg["EXT_GPIO"];
    sys_cfg.ext_gpio_enabled = json_ext_gpio["ENABLED"].as<bool>();

    JsonArray json_miners = cfg["MINERS"];
    sys_cfg.miners_num = json_miners.size();
    if (sys_cfg.miners_num > 0) {
      miners = (Miner *)malloc(sys_cfg.miners_num * sizeof(Miner));
      miners_s = (Miner_s *)malloc(sys_cfg.miners_num * sizeof(Miner_s));
      memset(miners, 0, sys_cfg.miners_num * sizeof(Miner));
      memset(miners_s, 0, sys_cfg.miners_num * sizeof(Miner_s));
      for (i = 0; i < sys_cfg.miners_num; i++) {
        JsonObject json_miner = json_miners[i];
        miners[i].ip_host = strdup(json_miner["IP_HOST"].as<const char*>());
        miners[i].port    = json_miner["PORT"].as<uint16_t>();
        miners[i].enabled = json_miner["ENABLED"].as<short>();
        miners[i].type    = (enum miner_type)json_miner["TYPE"].as<int>();
      }
    }
  }

  if (sys_cfg.sensors_enabled) {
    oneWire.begin(sys_cfg.sensors_pin);
    pinMode(sys_cfg.sensors_pin, INPUT_PULLUP);
    sensors.setOneWire(&oneWire);
    sensors.begin();
    sensors.setWaitForConversion(FALSE);
  }

  if (sys_cfg.ttp_enabled)
    Wire.begin(D3, D2);

  if (sys_cfg.ext_gpio_enabled)
    pcf8574.begin();

  if (sys_cfg.display_enabled) {
    init_display();
  }

  dlog(cfg_str + "\r\n");

  WiFi.mode(WIFI_STA);
  WiFi.hostname(sys_cfg.hostname);

  sntp_init();
  sntp_setservername(0, "pool.ntp.org");
  sntp_set_timezone(8);

  ws.onNotFound(ws_default_handler);
  ws.on("/get_state", HTTP_GET, ws_get_state);
  ws.on("/action", HTTP_POST, ws_action);

  for (i = 0; i < TASKS_NUM; i++) {
    tasks[i].id = task_list.every(tasks[i].interval, tasks[i].callback);
    tasks[i].callback();
  }
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

  if (SYS_NEED_RESET || SYS_NEED_BOOT) {
    byte pin = SYS_NEED_RESET ? RESET_PIN : BOOT_PIN;
    bitClear(sys_stat.flags, SYS_NEED_RESET_BIT);
    bitClear(sys_stat.flags, SYS_NEED_BOOT_BIT);
    if (sys_cfg.ext_gpio_enabled) {
      pcf8574.write(pin, LOW);
      delay(200);
      pcf8574.write(pin, HIGH);
    }
  }

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
      bitClear(sys_stat.flags, SYS_PWM_PID_BIT);
      pwm_pid.SetMode(MANUAL);
      dlog("Stop the PWM PID control.\r\n");
    } else if (s_val == 'p') {
      bitSet(sys_stat.flags, SYS_PWM_PID_BIT);
      pid_output = 0;
      pwm_pid.SetMode(AUTOMATIC);
      dlog("Start the PWM PID control.\r\n");
    } else if (s_val == 'd') {
      bitSet(sys_stat.flags, SYS_REDRAW_SCR_BIT);
      dlog("Re-Draw the Screen.\r\n");
    } else if (s_val == 'r') {
      ESP.restart();
    }
  }

  if (SYS_PWM_PID_EN) {
    if (pwm_pid.Compute()) {
      dlog(String("Current PID input is ") + pid_input
                + ", PID output is " + pid_output + "\r\n");
      next_pwm_val = (int)(0.5 + sys_stat.pwm[0] - pid_output);
      need_set_pwm = true;
    }
  }

  if (sys_cfg.ttp_enabled && need_set_pwm) {
    next_pwm_val = next_pwm_val > MAX_PWM_VAL ? MAX_PWM_VAL :
                  (next_pwm_val < MIN_PWM_VAL ? MIN_PWM_VAL : next_pwm_val);
    dlog(String("Get PWM value: ") + next_pwm_val + "\r\n");

    if (next_pwm_val != sys_stat.pwm[0]) {
      int i;
      for (i = 0; i < sys_cfg.ttp_num; i++) {
        Wire.beginTransmission(sys_cfg.ttp_addr[i]);
        Wire.write(next_pwm_val);
        Wire.endTransmission();
      }
    }
    next_pwm_val = 0;
  }
}

/* The task to update the wifi status */
void update_wifi_status() {
  if (!SYS_CFG_LOADED)
    return;

  if (WiFi.status() != WL_CONNECTED) {
    sys_stat.localip = 0xFFFFFFFF;
    sys_stat.bssid[0] = sys_stat.bssid[1] = sys_stat.bssid[2] = sys_stat.bssid[3]
                      = sys_stat.bssid[4] = sys_stat.bssid[5] = 0xFF;
    dlog("Connecting to " + sys_cfg.ssid + "...\r\n");
    WiFi.begin(sys_cfg.ssid.c_str(), sys_cfg.passwd.c_str());

    if (WiFi.waitForConnectResult(5000) != WL_CONNECTED) {
      bitClear(sys_stat.flags, SYS_WIFI_CONN_BIT);
      return;
    }

    bitSet(sys_stat.flags, SYS_WIFI_CONN_BIT);
    ws.begin();

    dlog("WiFi connected @" + WiFi.localIP().toString() + " (" + WiFi.BSSIDstr() + ")\r\n");
  } else {
    memcpy(sys_stat.bssid, WiFi.BSSID(), 6);
    sys_stat.localip = WiFi.localIP();
    sys_stat.rssi = WiFi.RSSI();
    bitSet(sys_stat.flags, SYS_WIFI_CONN_BIT);
    ws.handleClient();
  }
}

/* The task to get the temperature from sensors */
void update_temp() {
  if (!sys_cfg.sensors_enabled)
    return;

  sensors.requestTemperatures();
#if 1
  task_list.after(750, _update_temp);
#else
  int i;
  for (i = 0; i < sys_cfg.sensors_num; i++) {
    sys_stat.temp[i] = sensors.getTempCByIndex(i);
    sys_stat.temp[i] = sys_stat.temp[i] < 0 ? sys_stat.temp[i] * -1 : sys_stat.temp[i];
    dlog(String(sys_stat.temp[i]) + " ");
  }
  pid_input = sys_stat.temp[0] - sys_stat.temp[2];
  dlog(String("delta: ") + pid_input + "\r\n");
#endif
}

/* The task to get the tacho and pwm values from tachometers */
void update_tacho_pwm() {
  int i, j;
  unsigned char val_h, val_l;

  if (!sys_cfg.ttp_enabled)
    return;

  for (i = 0; i < sys_cfg.ttp_num; i++) {
    j = 0;
    Wire.requestFrom(sys_cfg.ttp_addr[i], (uint8_t)(TTP_PINS * 2 + 1));
    while (Wire.available()) {
      if (j == TTP_PINS) {
        sys_stat.pwm[i] = Wire.read();
        //dlog(String(sys_stat.pwm[i]) + "\r\n");
      } else {
        val_l = Wire.read();
        val_h = Wire.read();
        sys_stat.rpm[i][j] = ((val_h << 8) + val_l) * RPM_MULTI;
        //dlog(String(sys_stat.rpm[i][j]) + " ");
        j++;
      }
    }
  }
}

/* The task to draw the display */
void update_disp() {
  if (!sys_cfg.display_enabled)
    return;

  update_display();

  return;
}

/* The task to get the info from miners */
#ifdef USE_ASYNC_TCP
void as_onError(void *arg, AsyncClient *c, int error) {
  int i = (int)arg;
  dlog("Error : " + String((uint32_t)c, HEX) + " " + String(error) + " idx: " + String(i) + "\r\n");
  miners[i].asc = NULL;
  delete c;
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
}
void as_onDisconnect(void *arg, AsyncClient *c) {
  //dlog("DisConnected : " + String((uint32_t)c, HEX) + " idx: " + String((int)arg) + " " + c->state() +"\r\n");
  miners[(int)arg].asc = NULL;
  delete c;
}
void as_onData(void *arg, AsyncClient *c, void *data, size_t len) {
  //dlog("Data : " + String((uint32_t)c, HEX) + " idx: " + String((int)arg) + " " + String(len) +"\r\n");
  parse_miner_state((int)arg, (char *)data);
  dump_miner_state((int)arg);
}
void as_onConnect(void *arg, AsyncClient *c) {
  //dlog("Connected : " + String((uint32_t)c, HEX) + " idx: " + String((int)arg) + "\r\n");
  c->setNoDelay(true);
  c->onError(NULL, NULL);
  c->onDisconnect(as_onDisconnect, arg);
  c->onData(as_onData, arg);
  //dlog(String(query_miner_state((int)arg)) + "\r\n");
  c->write(query_miner_state((int)arg));
}
void update_miners() {
  int i;

  if (!SYS_WIFI_CONNECTED)
    return;

  for (i = 0; i < sys_cfg.miners_num; i++) {
    if (!miners[i].enabled)
      continue;
    if (!miners[i].asc) {
      miners[i].asc = new AsyncClient();
      if (!miners[i].asc) {
        dlog("Cannot alloc AsyncClient.\r\n");
        return;
      }
      //dlog("Alloc aClient : " + String((uint32_t)miners[i].asc, HEX) + "\r\n");
      miners[i].asc->onError(as_onError, (void *)i);
      miners[i].asc->onConnect(as_onConnect, (void *)i);
      //dlog(String((uint32_t)miners[i].asc, HEX) + " state " + miners[i].asc->state() + "\r\n");
      if (!miners[i].asc->connect(miners[i].ip_host, miners[i].port)) {
        dlog("AsyncClient Connect failed.\r\n");
        delete miners[i].asc;
        miners[i].asc = NULL;
      }
    } else {
      if (miners[i].asc->state() == 4)
        miners[i].asc->write(query_miner_state(i));
    }
  }
}
#else
void update_miners() {
  String str;
  WiFiClient client;
  int i, j;

  if (!SYS_WIFI_CONNECTED)
    return;

  for (i = 0; i < sys_cfg.miners_num; i++) {
    if (!miners[i].enabled)
      continue;
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

    str = "";
    switch (miners[i].type) {
      case PHOENIX:
      case CLAYMORE_ETH:
        client.println(
              "{\"id\":0,\"jsonrpc\":\"2.0\",\"method\":\"miner_getstat2\"}");
        break;
      case CLAYMORE_ZEC:
        client.println(
              "{\"id\":0,\"jsonrpc\":\"2.0\",\"method\":\"miner_getstat1\"}");
        break;
      case ZM_ZEC:
        client.println("{\"id\":1, \"method\":\"getstat\"}");
        str = client.readStringUntil('}') + '}';
        break;
      default:
        break;
    }
    str += client.readStringUntil('}') + '}';
    parse_miner_state(i, (char *)str.c_str());

    client.flush();
    client.stop();

    dump_miner_state(i);
  }
}
#endif

/* The task to update the state json string */
void update_state_json() {
  int i, j, gpus = 0;
  DynamicJsonDocument root(2048);

  root["temp_sensors"] = sys_cfg.sensors_num;
  JsonArray s_temp = root.createNestedArray("s_temp");
  for (i = 0; i < sys_cfg.sensors_num; i++)
    s_temp.add(sys_stat.temp[i]);

  root["pwms"] = sys_cfg.ttp_num;
  JsonArray pwm = root.createNestedArray("pwm");
  for (i = 0; i < sys_cfg.ttp_num; i++)
    pwm.add(sys_stat.pwm[i]);

  root["miners"] = sys_cfg.miners_num;
  JsonArray m_gpus = root.createNestedArray("m_gpus");
  JsonArray m_type = root.createNestedArray("m_type");
  JsonArray m_temp = root.createNestedArray("m_temp");
  JsonArray m_hash = root.createNestedArray("m_hash");
  JsonArray m_dhash = root.createNestedArray("m_dhash");
  JsonArray m_acp_s = root.createNestedArray("m_acp_s");
  JsonArray m_rej_s = root.createNestedArray("m_rej_s");
  JsonArray m_inc_s = root.createNestedArray("m_inc_s");
  JsonArray m_uptime = root.createNestedArray("m_uptime");
  JsonArray m_offtime = root.createNestedArray("m_offtime");
  root["gpus"] = gpus;
  JsonArray g_temp = root.createNestedArray("g_temp");
  JsonArray g_fan = root.createNestedArray("g_fan");
  JsonArray g_hash = root.createNestedArray("g_hash");
  JsonArray g_dhash = root.createNestedArray("g_dhash");
  JsonArray g_acp_s = root.createNestedArray("g_acp_s");
  JsonArray g_rej_s = root.createNestedArray("g_rej_s");
  JsonArray g_inc_s = root.createNestedArray("g_inc_s");
  for (i = 0; i < sys_cfg.miners_num; i++) {
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

  state_json_str = String();
  serializeJson(root, state_json_str);
  //dlog(str);
  //dlog(str.length());
}

/* The task to track the free heap memory */
void memory_state() {
  dlog("ESP.getFreeHeap()=");
  dlog(String(sys_stat.freeheap = ESP.getFreeHeap()) + "\r\n");
}

/* The task to send mqtt data to the server */
void send_mqtt() {
  if (!sys_cfg.mqtt_enabled || !SYS_WIFI_CONNECTED)
    return;

  if (!mqtt_client.connected()) {
    if (!mqtt_client.connect(sys_cfg.mqtt_user.c_str(),
         sys_cfg.mqtt_user.c_str(), sys_cfg.mqtt_passwd.c_str())) {
      bitClear(sys_stat.flags, SYS_MQTT_CONN_BIT);
      dlog(String("Cannot connect to the MQTT Server ")
                 + sys_cfg.mqtt_server + ':' + sys_cfg.mqtt_port + "\r\n");
      return;
    } else {
      dlog(String("Connected to the MQTT Server ")
                 + sys_cfg.mqtt_server + ':' + sys_cfg.mqtt_port + "\r\n");
    }
  }
  bitSet(sys_stat.flags, SYS_MQTT_CONN_BIT);
  //dlog("MQTT connected.\r\n");

  mqtt_client.publish(sys_cfg.mqtt_topic.c_str(), state_json_str.c_str());
}

void dlog(const char *str) {
  int i;

  for (i = 0; i < sys_cfg.display_num; i++)
    if (sys_cfg.display_output[i].disp_val[0].type == LOG)
      ((Disp_log)sys_cfg.display_output[i].disp_val[0].val1)(&sys_cfg.display_output[i], str);
}

void dlog(String str) {
  dlog(str.c_str());
}

void _update_temp() {
  int i, n;
  for (i = 0; i < sys_cfg.sensors_num; i++) {
    n = sys_cfg.sensors_map[i];
    sys_stat.temp[n] = sensors.getTempCByIndex(i);
    sys_stat.temp[n] = sys_stat.temp[n] < 0 ? sys_stat.temp[n] * -1 : sys_stat.temp[n];
  }
  pid_input = sys_stat.temp[0] - sys_stat.temp[2];
  dlog(String("W: ") + sys_stat.temp[0] + " O: " + sys_stat.temp[1] + " I: " + sys_stat.temp[2]
           + " D: " + (sys_stat.temp[0] - sys_stat.temp[2]) + "\r\n");
}

byte add_disp_val(Disp_outp *outp, enum disp_val_type type,
                  byte y, byte x, void* val1, int val2) {
  if (outp->last_val >= MAX_SCR_VAL)
    return x;

  outp->disp_val[outp->last_val].type       = type;
  outp->disp_val[outp->last_val].y          = y;
  outp->disp_val[outp->last_val].x          = x;
  outp->disp_val[outp->last_val].val1       = val1;
  outp->disp_val[outp->last_val].last.val_i = -1;

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
      x += (val2 = 9);  /* 99D23H59M */
      break;
    case SHARE_STATE:
      x += (val2 = 18); /* 3312/1/0    99.00% */
      break;
    case STRING:
      x += (val2 = strlen((char*)val1));
      break;
    case DATE:
      x += 24; /*Mon Mar 19 08:09:18 2018*/
      break;
    case MACADR:
      x += (val2 = 17); /* FF:FF:FF:FF:FF:FF */
      break;
    case IPADDR:
      x += (val2 = 15); /* 255.255.255.255 */
      break;
    default:
    break;
  }

  outp->disp_val[outp->last_val].val2       = val2;
  outp->last_val++;

  return x;
}

char* query_miner_state(int i) {
  switch (miners[i].type) {
    case PHOENIX:
    case CLAYMORE_ETH:
      return "{\"id\":0,\"jsonrpc\":\"2.0\",\"method\":\"miner_getstat2\"}\r\n";
    case CLAYMORE_ZEC:
      return "{\"id\":0,\"jsonrpc\":\"2.0\",\"method\":\"miner_getstat1\"}\r\n";
    case ZM_ZEC:
      return "{\"id\":1, \"method\":\"getstat\"}\r\n";
    default:
      return NULL;
  }
}

void dump_miner_state(int i) {
  String str;
  int j;

  str = String(miners[i].ip_host) + ": " + miners_s[i].uptime / 60 + "H"
        + miners_s[i].uptime % 60 + "M "+ miners_s[i].gpu_num + " GPUS "
        + average(miners_s[i].temp, miners_s[i].gpu_num) + " C "
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

void parse_miner_state(int i, char *str) {
  DynamicJsonDocument root(2048);
  int j, tmp_array[MAX_GPU_PER_MINER * 2];

  deserializeJson(root, str);

  switch (miners[i].type) {
    case PHOENIX:
    case CLAYMORE_ETH:
    case CLAYMORE_ZEC:
      {
        JsonArray result = root["result"];
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
        miners_s[i].uptime = ((int)root["uptime"]) / 60;
        JsonArray result = root["result"];
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
}

void Arduino_putchar(uint8_t c) {
  Serial.write(c);
}

void scr_log(Disp_outp *outp, const char *str) {
  Disp_val *vlog = outp->disp_val;
  int orig_x, orig_y;
  char ch;

  if (str == NULL || *str == '\0')
    return;

  getyx(orig_y, orig_x);

  if (vlog->x == 0) {
    String prefix = String(" [") + millis() + "] ";
    scroll();
    mvaddstr(vlog->y, vlog->x, prefix.c_str());
    vlog->x += prefix.length();
  }

  while (ch = *(str++)) {
    if (ch == '\r') {
      continue;
    } else if (ch == '\n') {
      vlog->x = 0;
      scr_log(outp, str);
      break;
    } else {
      mvaddch(vlog->y, vlog->x++, ch);
    }
  }

  move(orig_y, orig_x);
}

/*
      Miner State  ( Mon Mar 19 08:09:18 2018 --- Mon Mar 19 08:09:18 2018 )
--------------------------------------------------------------------------------
 System | WIFI: OK | CFG: OK | PID: KO  TARGET: 00  PWM: 28/28 | MQTT: OK
 WIFI   | SSID: xxxx@xx:xx:xx:xx:xx:xx | IP: 192.168.2.100 | RSSI: -55
 Sensor | Inlet: 18.37c | Water: 31.31c | Outlet: 31.56c | Delta: 12.94c
 Fan0   | 4095 | 4125 | 3855 | 5970 | 5820 | 5775 | 5835 | 1065 | 5745 | 0000
 Miner0 | 6 | ONLINE | 170MH/s | 36.33c | 00D23H13M | A/R/I: 3312/1/0 99.00%
--------------------------------------------------------------------------------
... ...
--------------------------------------------------------------------------------
CMD:
 */
void init_scr(Disp_outp *outp) {
  int col, row = 0, i, j, offset;
  char *c_str;

  if (outp == NULL || outp->type != SERIAL_SCREEN)
    return;

  setFunction_putchar(Arduino_putchar);
  initscr();
  clear();

  outp->last_val = 0;

  add_disp_val(outp, LOG, SCREEN_LINES - 3, 0, (void *)scr_log, 0);

  /* Miner State  ( Mon Mar 19 08:09:18 2018 --- Mon Mar 19 08:09:18 2018 )*/
  offset = 6;
  offset = add_disp_val(outp, STRING, row, offset, (void *)"Miner State", 0);
  offset = add_disp_val(outp, STRING, row, offset, (void *)"  ( ", 0);
  offset = add_disp_val(outp, DATE, row, offset, NULL, 0);
  offset = add_disp_val(outp, STRING, row, offset, (void *)" --- ", 0);
  offset = add_disp_val(outp, DATE, row, offset, NULL, 1);
  offset = add_disp_val(outp, STRING, row++, offset, (void *)" )", 0);
  add_disp_val(outp, R_CHAR, row++, 0, (void *)'-', COLS);

  /* System | WIFI: OK | CFG: OK | PID: KO  TARGET: 00  PWM: 28/28 | MQTT: OK */
  offset = 0;
  offset = add_disp_val(outp, STRING, row, offset, (void *)" System | WIFI: ", 0);
  offset = add_disp_val(outp, OK_KO, row, offset, &sys_stat.flags, SYS_WIFI_CONN_BIT);
  offset = add_disp_val(outp, STRING, row, offset, (void *)" | CFG: ", 0);
  offset = add_disp_val(outp, OK_KO, row, offset, &sys_stat.flags, SYS_CFG_LOAD_BIT);
  offset = add_disp_val(outp, STRING, row, offset, (void *)" | PID: ", 0);
  offset = add_disp_val(outp, OK_KO, row, offset, &sys_stat.flags, SYS_PWM_PID_BIT);
  offset = add_disp_val(outp, STRING, row, offset, (void *)"  TARGET: ", 0);
  offset = add_disp_val(outp, INT, row, offset, &pid_set_point, 2);
  offset = add_disp_val(outp, STRING, row, offset, (void *)"  PWM: ", 0);
  offset = add_disp_val(outp, INT, row, offset, &sys_stat.pwm[0], 2);
  offset = add_disp_val(outp, R_CHAR, row, offset, (void *)'/', 1);
  offset = add_disp_val(outp, INT, row, offset, &sys_stat.pwm[1], 2);
  offset = add_disp_val(outp, STRING, row, offset, (void *)" | MQTT: ", 0);
  add_disp_val(outp, OK_KO, row++, offset, &sys_stat.flags, SYS_MQTT_CONN_BIT);

  /* WIFI   | SSID: xxxx@xx:xx:xx:xx:xx:xx | IP: 192.168.2.100 | RSSI: -55 */
  offset = 0;
  offset = add_disp_val(outp, STRING, row, offset, (void *)" WIFI   | SSID: ", 0);
  offset = add_disp_val(outp, STRING, row, offset, (void *)sys_cfg.ssid.c_str(), 0);
  offset = add_disp_val(outp, R_CHAR, row, offset, (void *)'@', 1);
  offset = add_disp_val(outp, MACADR, row, offset, (void *)sys_stat.bssid, 0);
  offset = add_disp_val(outp, STRING, row, offset, (void *)" | IP: ", 0);
  offset = add_disp_val(outp, IPADDR, row, offset, (void *)&sys_stat.localip, 0);
  offset = add_disp_val(outp, STRING, row, offset, (void *)" | RSSI: ", 0);
  add_disp_val(outp, INT, row++, offset, &sys_stat.rssi, 3);

  /* Sensor | Inlet: 18.37c | Water: 31.31c | Outlet: 31.56c | Delta: 12.94c */
  if (sys_cfg.sensors_enabled) {
    offset = 0;
    offset = add_disp_val(outp, STRING, row, offset, (void *)" Sensor | Inlet: ", 0);
    offset = add_disp_val(outp, FLOAT, row, offset, &sys_stat.temp[2], 5);
    offset = add_disp_val(outp, STRING, row, offset, (void *)"c | Water: ", 0);
    offset = add_disp_val(outp, FLOAT, row, offset, &sys_stat.temp[0], 5);
    offset = add_disp_val(outp, STRING, row, offset, (void *)"c | Outlet: ", 0);
    offset = add_disp_val(outp, FLOAT, row, offset, &sys_stat.temp[1], 5);
    offset = add_disp_val(outp, STRING, row, offset, (void *)"c | Delta: ", 0);
    add_disp_val(outp, FLOAT_DELTA, row++, offset, &sys_stat.temp[0], 5);
  }

  /* Fan0   | 4095 | 4125 | 3855 | 5970 | 5820 | 5775 | 5835 | 1065 | 5745 | 0000 */
  if (sys_cfg.ttp_enabled) {
    for (i = 0; i < sys_cfg.ttp_num; i++) {
      sprintf(c_str = strdup(" Fan0  "), " Fan%d  ", i);
      offset = 0;
      offset = add_disp_val(outp, STRING, row, offset, c_str, 0);
      for (j = 0; j < TTP_PINS - 1; j++, col += 7) {
        offset = add_disp_val(outp, STRING, row, offset, (void *)" | ", 0);
        offset = add_disp_val(outp, INT, row, offset, &sys_stat.rpm[i][j], 4);
      }
      row++;
    }
  }

  /* Miner0 | 6 | ONLINE | 170MH/s | 36.33c | 00D23H13M | A/R/I: 3312/1/0 99.00% */
  for (i = 0; i < sys_cfg.miners_num; i++) {
    sprintf(c_str = strdup(" Miner0 | "), " Miner%d | ", i);
    offset = 0;
    offset = add_disp_val(outp, STRING, row, offset, c_str, 0);
    offset = add_disp_val(outp, INT, row, offset, &(miners_s[i].gpu_num), 1);
    offset = add_disp_val(outp, STRING, row, offset, (void *)" | ", 0);
    offset = add_disp_val(outp, ON_OFF, row, offset, &(miners_s[i].gpu_num), 1);
    offset = add_disp_val(outp, STRING, row, offset, (void *)" | ", 0);
    offset = add_disp_val(outp, INT, row, offset, &(miners_s[i].t_hash), 3);
    offset = add_disp_val(outp, STRING, row, offset, (void *)"MH/s | ", 0);
    offset = add_disp_val(outp, FLOAT, row, offset, &(miners_s[i].t_temp), 5);
    offset = add_disp_val(outp, STRING, row, offset, (void *)"c | ", 0);
    offset = add_disp_val(outp, TIME_SPAN, row, offset, &(miners_s[i].uptime), 0);
    offset = add_disp_val(outp, STRING, row, offset, (void *)" | A/R/I: ", 0);
    add_disp_val(outp, SHARE_STATE, row, offset, &(miners_s[i].t_accepted_s), 0);
    row++;
  }

  add_disp_val(outp, R_CHAR, row++, 0, (void *)'-', COLS);
  add_disp_val(outp, R_CHAR, SCREEN_LINES - 2, 0, (void *)'-', COLS);

  setscrreg (row, SCREEN_LINES - 3);

  offset = add_disp_val(outp, STRING, SCREEN_LINES - 1, 0, (void *)"CMD: ", 0);
  move(SCREEN_LINES - 1, offset);

  bitSet(sys_stat.flags, SYS_REDRAW_SCR_BIT);
}

void update_scr(Disp_outp *outp) {
  int i, n, orig_x, orig_y, changed, redraw, val_i;
  float val_f;
  time_t val_t;
  uint32_t val_u;
  Disp_val *disp_val = outp->disp_val;

  getyx(orig_y, orig_x);
  redraw = SYS_REDRAW_SCR;
  bitClear(sys_stat.flags, SYS_REDRAW_SCR_BIT);
  for (i = 0; i < outp->last_val; i++) {
    String str;
    changed = 0;
    switch (disp_val[i].type) {
      case OK_KO:
        val_i = bitRead(*(int *)(disp_val[i].val1), disp_val[i].val2);
        if (val_i != disp_val[i].last.val_i || redraw) {
          disp_val[i].last.val_i = val_i;
          changed = 2;
          str = val_i ? String("OK") : String("KO");
        }
        break;
      case ON_OFF:
        val_i = *(int *)(disp_val[i].val1);
        if (val_i != disp_val[i].last.val_i || redraw) {
          disp_val[i].last.val_i = val_i;
          changed = 2;
          str = val_i ? String("ONLINE") : String("OFLINE");
        }
        break;
      case FLOAT:
      case FLOAT_DELTA:
        val_f = *(float *)(disp_val[i].val1) - (disp_val[i].type == FLOAT_DELTA ?
                                  *((float *)disp_val[i].val1 + 2) : 0);
        if (val_f != disp_val[i].last.val_f || redraw) {
          disp_val[i].last.val_f = val_f;
          changed = 1;
          str = get_str_float(&val_f, 1, 0, disp_val[i].val2);
        }
        break;
      case INT:
        val_i = *(int *)(disp_val[i].val1);
        if (val_i != disp_val[i].last.val_i || redraw) {
          disp_val[i].last.val_i =  val_i;
          changed = 1;
          str = get_str_int(&val_i, 1, 0, disp_val[i].val2);
        }
        break;
      case TIME_SPAN:
        val_i = *(int *)(disp_val[i].val1);
        if (val_i != disp_val[i].last.val_i || redraw) {
          int m, h, d;
          disp_val[i].last.val_i =  val_i;
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
        val_i  = acp = *((int *)disp_val[i].val1);
        val_i += rej = *((int *)disp_val[i].val1 + 1);
        val_i += inc = *((int *)disp_val[i].val1 + 2);
        if (val_i != disp_val[i].last.val_i || redraw) {
          disp_val[i].last.val_i = val_i;
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
          mvaddstr(disp_val[i].y, disp_val[i].x, (char *)disp_val[i].val1);
        break;
      case R_CHAR:
        if (redraw) {
          int j = (int)disp_val[i].val1;
          char ch = (char)j;
          move(disp_val[i].y, disp_val[i].x);
          for (j = 0; j < disp_val[i].val2; j++)
            addch(ch);
        }
        break;
      case DATE:
        val_t = time(NULL) - (disp_val[i].val2 ? 0 : millis() / 1000);
        if (val_t != disp_val[i].last.val_t || redraw) {
          disp_val[i].last.val_t = val_t;
          str = ctime(&val_t);
          changed = 2;
        }
        break;
      case MACADR:
        val_u = BKDRHash((uint8_t *)disp_val[i].val1, 6);
        if (val_u != disp_val[i].last.val_u || redraw) {
          uint8_t *bssid = (uint8_t *)disp_val[i].val1;
          disp_val[i].last.val_u = val_u;
          changed = 1;
          str = String(String(bssid[0], HEX) + ":" + String(bssid[1], HEX) + ":"
                     + String(bssid[2], HEX) + ":" + String(bssid[3], HEX) + ":"
                     + String(bssid[4], HEX) + ":" + String(bssid[5], HEX));
        }
        break;
      case IPADDR:
        val_u = *(uint32_t *)(disp_val[i].val1);
        if (val_u != disp_val[i].last.val_u || redraw) {
          disp_val[i].last.val_u = val_u;
          changed = 1;
          str = IPAddress(val_u).toString();
        }
        break;
      default:
        break;
    }
    if (changed) {
      mvaddstr(disp_val[i].y, disp_val[i].x, str.c_str());
      if (disp_val[i].val2 > str.length() && changed == 1) {
        for (n = 0; n < disp_val[i].val2 - str.length(); n++)
          mvaddch(disp_val[i].y, disp_val[i].x + str.length() + n, ' ');
      }
    }
  }

  move(orig_y, orig_x);
}

void init_u8g2(Disp_outp *outp) {
  if (outp == NULL || outp->type != ST7920_128X64)
    return;

  u8g2.setBusClock(500000);
  u8g2.begin();
}

void update_u8g2(Disp_outp *outp) {
  #if 0
  u8g2.firstPage();
  do {
    u8g2.setFont(TEMP_FONT);
    u8g2.drawStr(0, 13, get_str_float(sys_stat.temp, 3, 0, 5).c_str());
    u8g2.setFont(RPM_FONT);
    u8g2.drawStr(0, 23, get_str_int(sys_stat.rpm[curr_tacho_pwm], 5, 0, 4).c_str());
    u8g2.setFont(RPM_FONT);
    u8g2.drawStr(0, 32, get_str_int(sys_stat.rpm[curr_tacho_pwm], 5, 5, 4).c_str());
    u8g2.setFont(VOLT_AMP_FONT);
    u8g2.drawStr(114, 23, get_str_int(&volt, 1, 0, 3).c_str());
    u8g2.setFont(VOLT_AMP_FONT);
    u8g2.drawStr(114, 32, get_str_int(&amp, 1, 0, 3).c_str());
  } while ( u8g2.nextPage() );
  #else
  int i;
  String str;

  u8g2.clearBuffer();
  u8g2.setFont(TEMP_FONT);
  u8g2.drawStr(0, 13, get_str_float(sys_stat.temp, 3, 0, 5).c_str());
  u8g2.setFont(MINER_FONT);
  str = String(miners_s[0].t_hash / 1000) + "m "
        + miners_s[0].t_dhash / 1000 + "m "
        + (int)miners_s[0].t_temp + "c "
        + (miners_s[0].gpu_num ? miners_s[0].uptime : miners_s[0].offtime) / 60
        + "h " + sys_stat.freeheap + " " + sys_stat.rpm[0][4];
  u8g2.drawStr(0, 23, str.c_str());
  str = String();
  for (i = 1; i < sys_cfg.miners_num; i++)
    if (miners[i].enabled)
      str += String(miners_s[i].t_hash) + "m " + (int)miners_s[i].t_temp + "c "
          + (miners_s[i].gpu_num ? miners_s[i].uptime : miners_s[i].offtime) / 60
          + "h ";
  u8g2.drawStr(0, 32, str.c_str());
  /*
  u8g2.setFont(RPM_FONT);
  u8g2.drawStr(0, 23, get_str_int(sys_stat.rpm[curr_tacho_pwm], 5, 0, 4).c_str());
  u8g2.setFont(RPM_FONT);
  u8g2.drawStr(0, 32, get_str_int(sys_stat.rpm[curr_tacho_pwm], 5, 5, 4).c_str());
  u8g2.setFont(VOLT_AMP_FONT);
  u8g2.drawStr(114, 23, get_str_int(&volt, 1, 0, 3).c_str());
  u8g2.setFont(VOLT_AMP_FONT);
  u8g2.drawStr(114, 32, get_str_int(&amp, 1, 0, 3).c_str());
  */
  u8g2.sendBuffer();
  #endif
}

void init_display() {
  int i;

  for (i = 0; i < sys_cfg.display_num; i++)
    sys_cfg.display_output[i].init(&sys_cfg.display_output[i]);
}

void update_display() {
  int i;

  for (i = 0; i < sys_cfg.display_num; i++)
    sys_cfg.display_output[i].render(&sys_cfg.display_output[i]);
}

String getContentType(String filename){
  if(ws.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

void ws_default_handler() {
  String path = ws.uri();
  dlog("Get uri " + path + "\r\n");
  if(path.endsWith("/")) path += "index.html";
  String contentType = getContentType(path);
  if(SPIFFS.exists(path)){
    File file = SPIFFS.open(path, "r");
    size_t sent = ws.streamFile(file, contentType);
    file.close();
    return;
  }
  ws.send(404, "text/plain", "FileNotFound");
}

void ws_get_state() {
  ws.send(200, "text/json", state_json_str);
}

void ws_action() {
  String action = ws.arg(0);
  if (action == "reset")
    bitSet(sys_stat.flags, SYS_NEED_RESET_BIT);
  else if (action == "boot")
    bitSet(sys_stat.flags, SYS_NEED_BOOT_BIT);
  ws.send(200, "text/plain", "");
}

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

static uint8_t arduino_pins[] = {D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10};
uint8_t str2pin(String str) {
  if (str.charAt(0) != 'D')
    return 0xFF;

  if (str.length() == 2 && str.charAt(1) >= '0' && str.charAt(1) <= '9')
    return arduino_pins[str.charAt(1) - '0'];

  if (str.length() == 3 && str.charAt(1) == '1'
                        && str.charAt(2) >= '0' && str.charAt(2) <= '9')
    return arduino_pins[10 + str.charAt(2) - '0'];

  return 0xFF;
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

uint32_t BKDRHash(uint8_t *buf, int num)
{
  uint32_t hash = 0;
  int i;

  for (i = 0; i < num; i++)
    hash = (hash << 5) - hash + (*buf++);

  return (hash & 0x7FFFFFFF);
}
