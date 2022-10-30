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
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold12pt7b.h>

#include "FS.h"

#define TTP_PWM_NUM 2
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
U8G2 u8g2;
U8G2_ST7920_128X64_F_HW_SPI u8g2_st7920(U8G2_R0, D8);
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2_ssd1322(U8G2_R2, D8, D4);
GxEPD2_BW<GxEPD2_213, GxEPD2_213::HEIGHT> epd(GxEPD2_213(/*CS*/D8, /*DC*/D2, /*RST*/D1, /*BUSY*/D0));

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
  NBMINER,
};
typedef struct {
  char *ip_host;
  uint16_t port;
  short enabled;
  enum miner_type type;
#ifdef USE_ASYNC_TCP
  AsyncClient *asc;
#endif
#define MAX_MINER_STATE_BUFF 4096
  char *buff;
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
#define SCREEN_MAX_LINE_LEN 80
#define MAX_SCR_VAL 160
#define EPD_PRE_Y 2
#define EPD_INT_X 4
#define EPD_INT_Y 4
#define EPD_PST_Y 2
#define EPD_PRE_X 4
#define EPD_IMG_W  32
#define EPD_IMG_H  32
#define BMP_HDR_SZ 62
#define BMP_FIL_SZ (BMP_HDR_SZ + EPD_IMG_W * EPD_IMG_H / 8)
enum disp_type {
  SERIAL_SCREEN,
  ST7920_128X64,
  SSD1322_256X64,
  GxEPD2_213,
  MAX_PANEL_TYPE,
};
enum disp_val_type {
  INT,
  LOG,
  IMG,
  DATE,
  FUNC,
  UINT8,
  FLOAT,
  OK_KO,
  UINT16,
  ON_OFF,
  R_CHAR,
  STRING,
  MACADR,
  IPADDR,
  TIME_SPAN,
  SHARE_STATE,
  FLOAT_DELTA,
};
typedef String Conv_f(uint16_t val);
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
  void           *priv;
};
void init_scr(Disp_outp *outp);
void init_u8g2(Disp_outp *outp);
void init_epd(Disp_outp *outp);
void update_scr(Disp_outp *outp);
void update_u8g2(Disp_outp *outp);
void update_epd(Disp_outp *outp);
Disp_init   d_inits[]   = {init_scr, init_u8g2, init_u8g2, init_epd};
Disp_render d_renders[] = {update_scr, update_u8g2, update_u8g2, update_epd};

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
  uint8_t   ttp_pins;
  float     ttp_vcc;
  uint16_t  ttp_0a_val;
  uint8_t   *ttp_addr;
  bool      display_enabled;
  int       display_num;
  Disp_outp *display_output;
  bool      ext_gpio_enabled;
  int       miners_num;
} Sys_cfg;
Sys_cfg sys_cfg;

#define SYS_CFG_FILE_PATH "/sys.cfg"

typedef struct __attribute__((packed)) {
  uint8_t pwm_val[TTP_PWM_NUM];
  uint16_t acs712_val;
  uint16_t interval;
} Ttp_stat;

typedef struct {
  uint32_t  flags;
  Ttp_stat  *ttp_stat;
  uint16_t  **rpm;
  float     *temp;
  int       freeheap;
  uint8_t   bssid[6];
  uint32_t  localip;
  int       rssi;
} Sys_stat;
Sys_stat sys_stat;

#define SYS_CFG_LOAD_BIT     0
#define SYS_WIFI_CONN_BIT    1
#define SYS_WIFI_CONI_BIT    2
#define SYS_PWM_PID_BIT      3
#define SYS_MQTT_CONN_BIT    4
#define SYS_REDRAW_SCR_BIT   5
#define SYS_REDRAW_EPD_BIT   6
#define SYS_NEED_RESET_BIT   7
#define SYS_NEED_BOOT_BIT    8
#define SYS_WIFI_CONNECTED   bitRead(sys_stat.flags, SYS_WIFI_CONN_BIT)
#define SYS_WIFI_CONNECTING  bitRead(sys_stat.flags, SYS_WIFI_CONI_BIT)
#define SYS_CFG_LOADED       bitRead(sys_stat.flags, SYS_CFG_LOAD_BIT)
#define SYS_PWM_PID_EN       bitRead(sys_stat.flags, SYS_PWM_PID_BIT)
#define SYS_REDRAW_SCR       bitRead(sys_stat.flags, SYS_REDRAW_SCR_BIT)
#define SYS_REDRAW_EPD       bitRead(sys_stat.flags, SYS_REDRAW_EPD_BIT)
#define SYS_NEED_RESET       bitRead(sys_stat.flags, SYS_NEED_RESET_BIT)
#define SYS_NEED_BOOT        bitRead(sys_stat.flags, SYS_NEED_BOOT_BIT)

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
    if (sys_cfg.ttp_enabled && sys_cfg.ttp_num > 0) {
      sys_cfg.ttp_pins = json_ttp["PINS"].as<uint8_t>();
      sys_cfg.ttp_vcc = json_ttp["VCC"].as<float>();
      sys_cfg.ttp_0a_val = json_ttp["0A_VAL"].as<uint16_t>();
      sys_cfg.ttp_addr = (uint8_t *)malloc(sys_cfg.ttp_num * sizeof(uint8_t));
      sys_stat.rpm = (uint16_t **)malloc((sys_cfg.ttp_num + 1) * sizeof(uint16_t *));
      for (i = 0; i < sys_cfg.ttp_num; i++) {
        sys_cfg.ttp_addr[i] = json_ta[i].as<uint8_t>();
        sys_stat.rpm[i] = (uint16_t *)malloc((sys_cfg.ttp_pins) * sizeof(uint16_t));
      }
      sys_stat.rpm[i] = (uint16_t *)malloc((sys_cfg.ttp_pins) * sizeof(uint16_t));
      sys_stat.ttp_stat = (Ttp_stat *)malloc(sys_cfg.ttp_num * sizeof(Ttp_stat));
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
        if (miners[i].type == NBMINER) {
          miners[i].buff = (char *)malloc(MAX_MINER_STATE_BUFF);
          miners[i].buff[0] = 0;
        } else {
          miners[i].buff = NULL;
        }
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
    //dlog(String("Input Char Value: ") + (int)s_val + "\r\n");
    if (s_val >= 48 && s_val <= 57) {
      next_pwm_val = 10 * next_pwm_val + s_val - 48;
    } else if (s_val == 8 /* Backspace */) {
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
      next_pwm_val = (int)(0.5 + sys_stat.ttp_stat[0].pwm_val[0] - pid_output);
      need_set_pwm = true;
    }
  }

  if (sys_cfg.ttp_enabled && need_set_pwm) {
    pwm2ttp(next_pwm_val);
    next_pwm_val = 0;
  }
}

/* The task to update the wifi status */
void update_wifi_status() {
  if (!SYS_CFG_LOADED)
    return;

  int wifi_status = SYS_WIFI_CONNECTING ? WiFi.waitForConnectResult(5000) : WiFi.status();

  if (SYS_WIFI_CONNECTED) {
    if (wifi_status != WL_CONNECTED) {
      bitClear(sys_stat.flags, SYS_WIFI_CONN_BIT);
      dlog("WiFi " + sys_cfg.ssid + " disconnected ...\r\n");
    } else {
      ws.handleClient();
    }
  } else if (wifi_status == WL_CONNECTED){
    ws.begin();
    sys_stat.rssi = WiFi.RSSI();
    sys_stat.localip = WiFi.localIP();
    memcpy(sys_stat.bssid, WiFi.BSSID(), 6);
    bitSet(sys_stat.flags, SYS_WIFI_CONN_BIT);
    bitClear(sys_stat.flags, SYS_WIFI_CONI_BIT);
    dlog("WiFi connected @" + WiFi.localIP().toString() + " (" + WiFi.BSSIDstr() + ")\r\n");
  } else {
    sys_stat.localip = 0xFFFFFFFF;
    sys_stat.bssid[0] = sys_stat.bssid[1] = sys_stat.bssid[2] = sys_stat.bssid[3]
                      = sys_stat.bssid[4] = sys_stat.bssid[5] = 0xFF;
    bitSet(sys_stat.flags, SYS_WIFI_CONI_BIT);
    WiFi.begin(sys_cfg.ssid.c_str(), sys_cfg.passwd.c_str());
    dlog("Connecting to " + sys_cfg.ssid + "...\r\n");
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
  int i, j, ret, req;

  if (!sys_cfg.ttp_enabled)
    return;

  req = sizeof(Ttp_stat) + sys_cfg.ttp_pins * sizeof(uint16_t);
  for (i = 0; i < sys_cfg.ttp_num; i++) {
    ret = Wire.requestFrom(sys_cfg.ttp_addr[i], (uint8_t)req);
    if (ret != req) {
      dlog(String("TTP@") + sys_cfg.ttp_addr[i] + ": response size(" + ret
                  + ") not match " + req + "\r\n");
      return;
    }

    Wire.readBytes((uint8_t *)sys_stat.rpm[sys_cfg.ttp_num],
                    sys_cfg.ttp_pins * sizeof(uint16_t));
    Wire.readBytes((uint8_t *)&sys_stat.ttp_stat[i], sizeof(Ttp_stat));
    for (j = 0; j < sys_cfg.ttp_pins; j++) {
      sys_stat.rpm[i][j] = (uint32_t)sys_stat.rpm[sys_cfg.ttp_num][j]
                            * RPM_MULTI * 1000 / sys_stat.ttp_stat[i].interval;
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
  if (parse_miner_state((int)arg, (char *)data, len))
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

  if (sys_cfg.sensors_enabled) {
    root["temp_sensors"] = sys_cfg.sensors_num;
    JsonArray s_temp = root.createNestedArray("s_temp");
    for (i = 0; i < sys_cfg.sensors_num; i++)
      s_temp.add(sys_stat.temp[i]);
  }

  if (sys_cfg.ttp_enabled) {
    root["pwms"] = sys_cfg.ttp_num;
    JsonArray pwm = root.createNestedArray("pwm");
    for (i = 0; i < sys_cfg.ttp_num; i++)
      pwm.add(sys_stat.ttp_stat[i].pwm_val[0]);
  }

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
      mqtt_client.setBufferSize(1024);
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
  float water;
  for (i = 0; i < sys_cfg.sensors_num; i++) {
    n = sys_cfg.sensors_map[i];
    sys_stat.temp[n] = sensors.getTempCByIndex(i);
    sys_stat.temp[n] = sys_stat.temp[n] < 0 ? sys_stat.temp[n] * -1 : sys_stat.temp[n];
  }
  water = (sys_stat.temp[2] + sys_stat.temp[3]) / 2;
  pid_input = water - sys_stat.temp[0];
  dlog(String("W: ") + water + " AI: " + sys_stat.temp[0] + " AO: " + sys_stat.temp[1]
           + " D: " + pid_input + "\r\n");
}

void pwm2ttp(int next_pwm) {
/*
  32      :   all pwm set to 32;
  n32     :   pwm[n - 1] of all ttp set to 32;
  m032    :   all pwm of ttp[m - 1] set to 32;
  mn32    :   pwm[n - 1] of ttp[m - 1] set to 32;
*/
  uint8_t pwm, m, n, i, j;
  Ttp_stat ttp_stat;

  m = next_pwm / 1000;
  n = (next_pwm - m * 1000) / 100;
  pwm = next_pwm - (m * 1000 + n * 100);

  //dlog(String("Parse PWM: ") + next_pwm + " --> " + m + " " + n + " " + pwm + "\r\n");

  if (pwm < MIN_PWM_VAL || pwm > MAX_PWM_VAL || m > sys_cfg.ttp_num || n > TTP_PWM_NUM) {
    dlog(String("Invalid PWM value: ") + next_pwm + "\r\n");
    return;
  }

  for (i = 0; i < sys_cfg.ttp_num; i++) {
    if (m == 0 || m == (i + 1)) {
      memcpy(&ttp_stat, &sys_stat.ttp_stat[i], sizeof(Ttp_stat));
      for (j = 0; j < TTP_PWM_NUM; j++) {
        if (n == 0 || n == (j + 1)) {
          ttp_stat.pwm_val[j] = pwm;
        }
      }
      Wire.beginTransmission(sys_cfg.ttp_addr[i]);
      Wire.write((byte *)&ttp_stat, sizeof(Ttp_stat));
      Wire.endTransmission();
    }
  }

  /*
  dlog(String("New PWM: ") + sys_stat.ttp_stat[0].pwm_val[0] + " "
              + sys_stat.ttp_stat[0].pwm_val[1] + " "
              + sys_stat.ttp_stat[1].pwm_val[0] + " "
              + sys_stat.ttp_stat[1].pwm_val[1] + "\r\n");
  */
}

String conv_flow(uint16_t val) {
  float flow = (float)val * 2 / 450;
  String str = get_str_float(&flow, 1, 0, 4);
  return str + "L";
}

String conv_acs712(uint16_t val) {
  float vcc = sys_cfg.ttp_vcc < 0.1 ? 5 : sys_cfg.ttp_vcc;
  uint16_t z_val = sys_cfg.ttp_0a_val == 0 ? 512 : sys_cfg.ttp_0a_val;
  float amp = vcc * abs(val - z_val) * 1000 / (val * 2 * 185);
  String str = get_str_float(&amp, 1, 0, 3);
  //dlog(String("ACS712: ") + vcc + " " + z_val + " " + val + " " + amp + "\r\n");
  return str + "A";
}

byte add_disp_val(Disp_outp *outp, enum disp_val_type type,
                  byte y, byte x, void* val1, int val2) {
  uint8_t width = 1;

  if (outp->last_val >= MAX_SCR_VAL)
    return x;

  outp->disp_val[outp->last_val].type       = type;
  outp->disp_val[outp->last_val].y          = y;
  outp->disp_val[outp->last_val].x          = x;
  outp->disp_val[outp->last_val].val1       = val1;
  outp->disp_val[outp->last_val].last.val_i = -1;

  if (outp->type == SSD1322_256X64)
    width = u8g2.getMaxCharWidth();

  switch (type) {
    case INT:
    case UINT8:
    case FLOAT:
    case UINT16:
    case R_CHAR:
    case FLOAT_DELTA:
      x += val2 * width;
      break;
    case OK_KO:
      x += 2 * width;  /* OK or KO */
      break;
    case ON_OFF:
      x += 6 * width;  /* ONLINE or OFLINE*/
      break;
    case TIME_SPAN:
      x += (val2 = 9) * width;  /* 99D23H59M */
      break;
    case SHARE_STATE:
      x += (val2 = 18) * width; /* 3312/1/0    99.00% */
      break;
    case STRING:
      x += (val2 = strlen((char*)val1)) * width;
      break;
    case DATE:
      x += 24 * width; /*Mon Mar 19 08:09:18 2018*/
      break;
    case MACADR:
      x += (val2 = 17) * width; /* FF:FF:FF:FF:FF:FF */
      break;
    case IPADDR:
      x += (val2 = 15) * width; /* 255.255.255.255 */
      break;
    case FUNC:
      x += ((Conv_f *)val2)(1).length() * width;
      break;
    default:
      break;
  }

  outp->disp_val[outp->last_val].val2       = val2;
  outp->last_val++;

  return x;
}

int check_disp_val(Disp_val *disp_val, int redraw, String &str)
{
  float val_f;
  time_t val_t;
  uint32_t val_u;
  int changed = 0, val_i;

  str = "";
  switch (disp_val->type) {
    case OK_KO:
      val_i = bitRead(*(int *)(disp_val->val1), disp_val->val2);
      if (val_i != disp_val->last.val_i || redraw) {
        disp_val->last.val_i = val_i;
        changed = 2;
        str = val_i ? String("OK") : String("KO");
      }
      break;
    case ON_OFF:
      val_i = *(int *)(disp_val->val1);
      if (val_i != disp_val->last.val_i || redraw) {
        disp_val->last.val_i = val_i;
        changed = 2;
        str = val_i ? String("ONLINE") : String("OFLINE");
      }
      break;
    case FLOAT:
    case FLOAT_DELTA:
      val_f = *(float *)(disp_val->val1) - (disp_val->type == FLOAT_DELTA ?
                                *((float *)disp_val->val1 + 2) : 0);
      if (val_f != disp_val->last.val_f || redraw) {
        disp_val->last.val_f = val_f;
        changed = 1;
        str = get_str_float(&val_f, 1, 0, disp_val->val2);
      }
      break;
    case INT:
    case UINT8:
    case UINT16:
      if (disp_val->type == INT)
        val_i = *(int *)(disp_val->val1);
      else if (disp_val->type == UINT8)
        val_i = *(uint8_t *)(disp_val->val1);
      else
        val_i = *(uint16_t *)(disp_val->val1);
      if (val_i != disp_val->last.val_i || redraw) {
        disp_val->last.val_i =  val_i;
        changed = 1;
        str = get_str_int(&val_i, 1, 0, disp_val->val2);
      }
      break;
    case TIME_SPAN:
      val_i = *(int *)(disp_val->val1);
      if (val_i != disp_val->last.val_i || redraw) {
        int m, h, d;
        disp_val->last.val_i =  val_i;
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
      val_i  = acp = *((int *)disp_val->val1);
      val_i += rej = *((int *)disp_val->val1 + 1);
      val_i += inc = *((int *)disp_val->val1 + 2);
      if (val_i != disp_val->last.val_i || redraw) {
        disp_val->last.val_i = val_i;
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
      if (redraw) {
        changed = 1;
        str = String((char *)disp_val->val1);
      }
      break;
    case R_CHAR:
      if (redraw) {
        int ch = (int)disp_val->val1, j;
        changed = 1;
        for (j = 0; j < disp_val->val2; j++)
          str += (char)ch;
      }
      break;
    case DATE:
      val_t = time(NULL) - (disp_val->val2 ? 0 : millis() / 1000);
      if (val_t != disp_val->last.val_t || redraw) {
        disp_val->last.val_t = val_t;
        str = ctime(&val_t);
        changed = 2;
      }
      break;
    case MACADR:
      val_u = BKDRHash((uint8_t *)disp_val->val1, 6);
      if (val_u != disp_val->last.val_u || redraw) {
        uint8_t *bssid = (uint8_t *)disp_val->val1;
        disp_val->last.val_u = val_u;
        changed = 1;
        str = String(String(bssid[0], HEX) + ":" + String(bssid[1], HEX) + ":"
                   + String(bssid[2], HEX) + ":" + String(bssid[3], HEX) + ":"
                   + String(bssid[4], HEX) + ":" + String(bssid[5], HEX));
      }
      break;
    case IPADDR:
      val_u = *(uint32_t *)(disp_val->val1);
      if (val_u != disp_val->last.val_u || redraw) {
        disp_val->last.val_u = val_u;
        changed = 1;
        str = IPAddress(val_u).toString();
      }
      break;
    case FUNC:
      val_i = *(uint16_t *)(disp_val->val1);
      if (val_i != disp_val->last.val_i || redraw) {
        disp_val->last.val_i = val_i;
        changed = 2;
        str = ((Conv_f *)disp_val->val2)(val_i);
      }
      break;
    default:
      break;
  }

  return changed;
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
    case NBMINER:
      return "GET /api/v1/status HTTP/1.0\r\n\r\n";
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

bool parse_miner_state(int i, char *data, size_t len) {
  DynamicJsonDocument root(4096);
  int j, tmp_array[MAX_GPU_PER_MINER * 2];
  char *str = data;

  // special handle for NBMINER, as the miner stat data are long
  // and include a HTTP header.
  if (miners[i].type == NBMINER) {
    if (strlen(miners[i].buff) + len >= MAX_MINER_STATE_BUFF)
      miners[i].buff[0] = 0;
    strncat(miners[i].buff, data, len);
    str = strchr(miners[i].buff, '{');
    if (str == NULL)
      return false;
  }

  // use (const char*) here to make sure deserializeJson will not write to str.
  DeserializationError err = deserializeJson(root, (const char*)str);
  //dlog(String("Parsing miner") + i + ": " + err.c_str() + "\r\n");
  if (err)
    return false;

  //data[len] = 0;
  //dlog(String(str) + "\r\n");
  if (miners[i].type == NBMINER)
      miners[i].buff[0] = 0;

  switch (miners[i].type) {
    case PHOENIX:
    case CLAYMORE_ETH:
    case CLAYMORE_ZEC:
      {
        JsonArray result = root["result"];
        miners_s[i].uptime = result[1];
        str2array(result[2], ';', tmp_array, 3);
        miners_s[i].t_hash = tmp_array[0] / 1000;
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
    case NBMINER:
      {
        miners_s[i].uptime = (time(NULL) - (int)root["start_time"]) / 60;
        JsonArray result = root["miner"]["devices"];
        miners_s[i].gpu_num = result.size();
        miners_s[i].t_hash = (int)root["miner"]["total_hashrate_raw"] / 1000000;
        miners_s[i].t_accepted_s = root["stratum"]["accepted_shares"];
        miners_s[i].t_rejected_s = root["stratum"]["rejected_shares"];
        miners_s[i].t_incorrect_s = root["stratum"]["invalid_shares"];
        for (j = 0; j < miners_s[i].gpu_num; j++) {
          miners_s[i].hash[j] = (int)result[j]["hashrate_raw"] / 1000000;
          miners_s[i].temp[j] = result[j]["temperature"];
          miners_s[i].fan[j] = result[j]["fan"];
          miners_s[i].accepted_s[j] = result[j]["accepted_shares"];
          miners_s[i].rejected_s[j] = result[j]["rejected_shares"];
          miners_s[i].incorrect_s[j] = result[j]["invalid_shares"];
        }
      }
      break;
    default:
      break;
  }
  miners_s[i].t_temp = average(miners_s[i].temp, miners_s[i].gpu_num);

  return true;
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

  while (ch = *(str++)) {
    if (vlog->x == 0) {
      String prefix = String(" [") + millis() + "] ";
      scroll();
      mvaddstr(vlog->y, vlog->x, prefix.c_str());
      vlog->x = prefix.length();
    }
    if (ch == '\r') {
      continue;
    } else if (ch == '\n') {
      vlog->x = 0;
    } else {
      mvaddch(vlog->y, vlog->x++, ch);
      vlog->x %= SCREEN_MAX_LINE_LEN;
    }
  }

  move(orig_y, orig_x);
}

/*
      Miner State  ( Mon Mar 19 08:09:18 2018 --- Mon Mar 19 08:09:18 2018 )
--------------------------------------------------------------------------------
 System | WIFI: OK | CFG: OK | PID: KO/00/28/28 | MQTT: OK | FLR: 2.80LPM
 WIFI   | SSID: xxxx@xx:xx:xx:xx:xx:xx | IP: 192.168.2.100 | RSSI: -55
 Sensor | AI: 18.37c | AO: 31.31c | WI: 31.56c | WO: 32.99 | Delta: 12.94c
 TTP0   | 26/38 3.10A |1072|1065|1050|1035|0795|1005|0000|0000|0000|0000|0000|
 Miner0 | 6 | ONLINE | 170MH/s | 36.33c | 00D23H13M | A/R/I: 3312/1/0 99.00%
--------------------------------------------------------------------------------
... ...
--------------------------------------------------------------------------------
CMD:
 */
void init_scr(Disp_outp *outp) {
  int row = 0, i, j, offset;
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

  /* System | WIFI: OK | CFG: OK | PID: KO/00/28/28 | MQTT: OK | FLR: 2.80LPM */
  offset = 0;
  offset = add_disp_val(outp, STRING, row, offset, (void *)" System | WIFI: ", 0);
  offset = add_disp_val(outp, OK_KO, row, offset, &sys_stat.flags, SYS_WIFI_CONN_BIT);
  offset = add_disp_val(outp, STRING, row, offset, (void *)" | CFG: ", 0);
  offset = add_disp_val(outp, OK_KO, row, offset, &sys_stat.flags, SYS_CFG_LOAD_BIT);
  offset = add_disp_val(outp, STRING, row, offset, (void *)" | PID: ", 0);
  offset = add_disp_val(outp, OK_KO, row, offset, &sys_stat.flags, SYS_PWM_PID_BIT);
  offset = add_disp_val(outp, R_CHAR, row, offset, (void *)'/', 1);
  offset = add_disp_val(outp, INT, row, offset, &pid_set_point, 2);
  if (sys_cfg.ttp_enabled) {
    for (i = 0; i < sys_cfg.ttp_num; i++) {
      offset = add_disp_val(outp, R_CHAR, row, offset, (void *)'/', 1);
      offset = add_disp_val(outp, UINT8, row, offset, &sys_stat.ttp_stat[i].pwm_val[0], 2);
    }
  } else {
    offset = add_disp_val(outp, STRING, row, offset, (void *)"/NA", 0);
  }
  offset = add_disp_val(outp, STRING, row, offset, (void *)" | MQTT: ", 0);
  offset = add_disp_val(outp, OK_KO, row, offset, &sys_stat.flags, SYS_MQTT_CONN_BIT);
  offset = add_disp_val(outp, STRING, row, offset, (void *)" | FLR: ", 0);
  if (sys_cfg.ttp_enabled)
    offset = add_disp_val(outp, FUNC, row, offset, (void *)&sys_stat.rpm[1][9], (int)conv_flow);
  else
    offset = add_disp_val(outp, STRING, row, offset, (void *)"NAL", 0);
  add_disp_val(outp, STRING, row++, offset, (void *)"PM", 0);

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

  /* Sensor | AI: 18.37c | AO: 31.31c | WI: 31.56c | WO: 32.99 | Delta: 12.94c */
  if (sys_cfg.sensors_enabled) {
    offset = 0;
    offset = add_disp_val(outp, STRING, row, offset, (void *)" Sensor | AI: ", 0);
    offset = add_disp_val(outp, FLOAT, row, offset, &sys_stat.temp[0], 5);
    offset = add_disp_val(outp, STRING, row, offset, (void *)"c | AO: ", 0);
    offset = add_disp_val(outp, FLOAT, row, offset, &sys_stat.temp[1], 5);
    offset = add_disp_val(outp, STRING, row, offset, (void *)"c | WI: ", 0);
    offset = add_disp_val(outp, FLOAT, row, offset, &sys_stat.temp[2], 5);
    offset = add_disp_val(outp, STRING, row, offset, (void *)"c | WO: ", 0);
    offset = add_disp_val(outp, FLOAT, row, offset, &sys_stat.temp[3], 5);
    offset = add_disp_val(outp, STRING, row, offset, (void *)"c | Delta: ", 0);
    add_disp_val(outp, FLOAT_DELTA, row++, offset, &sys_stat.temp[0], 5);
  }

  /* TTP0   | 26/38 3.10A |1072|1065|1050|1035|0795|1005|0000|0000|0000|0000|0000| */
  if (sys_cfg.ttp_enabled) {
    for (i = 0; i < sys_cfg.ttp_num; i++) {
      sprintf(c_str = strdup(" TTP0  "), " TTP%d  ", i);
      offset = 0;
      offset = add_disp_val(outp, STRING, row, offset, c_str, 0);
      offset = add_disp_val(outp, STRING, row, offset, (void *)" | ", 0);
      offset = add_disp_val(outp, UINT8, row, offset,
                            &sys_stat.ttp_stat[i].pwm_val[0], 2);
      offset = add_disp_val(outp, R_CHAR, row, offset, (void *)'/', 1);
      offset = add_disp_val(outp, UINT8, row, offset,
                            &sys_stat.ttp_stat[i].pwm_val[1], 2);
      offset = add_disp_val(outp, R_CHAR, row, offset, (void *)' ', 1);
      offset = add_disp_val(outp, FUNC, row, offset,
                            &sys_stat.ttp_stat[i].acs712_val, (int)conv_acs712);
      offset = add_disp_val(outp, STRING, row, offset, (void *)" |", 0);
      for (j = 0; j < sys_cfg.ttp_pins - 1; j++) {
        offset = add_disp_val(outp, UINT16, row, offset, &sys_stat.rpm[i][j], 4);
        offset = add_disp_val(outp, STRING, row, offset, (void *)"|", 0);
      }
      row++;
    }
  }

  /* Miner0 | 6 | ONLINE | 170MH/s | 36.33c | 00D23H13M | A/R/I: 3312/1/0 99.00% */
  for (i = 0; i < sys_cfg.miners_num; i++) {
    if (!miners[i].enabled)
      continue;
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
  int i, n, orig_x, orig_y, changed, redraw;
  Disp_val *disp_val = outp->disp_val;
  String str;

  getyx(orig_y, orig_x);

  redraw = SYS_REDRAW_SCR;
  bitClear(sys_stat.flags, SYS_REDRAW_SCR_BIT);
  for (i = 0; i < outp->last_val; i++) {
    changed = check_disp_val(&disp_val[i], redraw, str);
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
  if (outp == NULL)
    return;

  if (outp->type == ST7920_128X64)
    u8g2 = u8g2_st7920;
  else if (outp->type == SSD1322_256X64)
    u8g2 = u8g2_ssd1322;
  else
    return;

  u8g2.setBusClock(500000);
  u8g2.begin();

  if (outp->type == SSD1322_256X64) {

/*
 S | AI:18.37|AO:31.31|WI:31.56|WO:33.56
 F | 28/38|4095|4125|3855|5970|5820|5775
 F | 28/38|4095|4125|3855|5970|5820|5775
 B | 2.22L|4095|4125|3855|5970|5820|3.1A
 M | 70M|36.3|00D23H13M|3312/10/00 99.00
 M | 70M|36.3|00D23H13M|3312/10/00 99.00
 */
#define SSD1322_256X64_MAX_MINERS 2

    int x, y, i, j;
    Miner_s *d_miners;

    u8g2.setContrast(0);
    u8g2.setFont(u8g2_font_t0_12_mr);

    y = 10;
    x = add_disp_val(outp, STRING, y, 0, (void *)"S | AI:", 0);
    x = add_disp_val(outp, FLOAT, y, x, &sys_stat.temp[0], 5);
    x = add_disp_val(outp, STRING, y, x, (void *)"|AO:", 0);
    x = add_disp_val(outp, FLOAT, y, x, &sys_stat.temp[1], 5);
    x = add_disp_val(outp, STRING, y, x, (void *)"|WI:", 0);
    x = add_disp_val(outp, FLOAT, y, x, &sys_stat.temp[2], 5);
    x = add_disp_val(outp, STRING, y, x, (void *)"|WO:", 0);
    x = add_disp_val(outp, FLOAT, y, x, &sys_stat.temp[3], 5);

    if (sys_cfg.ttp_enabled) {
      for (i = 0; i < sys_cfg.ttp_num; i++) {
        y += 10;
        x = add_disp_val(outp, STRING, y, 0, (void *)"F | ", 0);
        x = add_disp_val(outp, UINT8, y, x, &sys_stat.ttp_stat[i].pwm_val[0], 2);
        x = add_disp_val(outp, R_CHAR, y, x, (void *)'/', 1);
        x = add_disp_val(outp, UINT8, y, x, &sys_stat.ttp_stat[i].pwm_val[1], 2);
        for (j = 0; j < 6; j++) {
          x = add_disp_val(outp, STRING, y, x, (void *)"|", 0);
          x = add_disp_val(outp, UINT16, y, x, &sys_stat.rpm[i][j], 4);
        }
      }

      y += 10;
      x = add_disp_val(outp, STRING, y, 0, (void *)"B | ", 0);
      x = add_disp_val(outp, FUNC, y, x, (void *)&sys_stat.rpm[1][9], (int)conv_flow);
      x = add_disp_val(outp, STRING, y, x, (void *)"|", 0);
      x = add_disp_val(outp, UINT16, y, x, &sys_stat.rpm[1][10], 4);
      for (j = 6; j < 10; j++) {
        x = add_disp_val(outp, STRING, y, x, (void *)"|", 0);
        x = add_disp_val(outp, UINT16, y, x, &sys_stat.rpm[0][j], 4);
      }
      x = add_disp_val(outp, STRING, y, x, (void *)"|", 0);
      x = add_disp_val(outp, FUNC, y, x,
                            &sys_stat.ttp_stat[1].acs712_val, (int)conv_acs712);
    }

    d_miners = (Miner_s *)malloc(sizeof(Miner_s) * SSD1322_256X64_MAX_MINERS);
    for (i = 0; i < SSD1322_256X64_MAX_MINERS; i++) {
      y += 10;
      x = add_disp_val(outp, STRING, y, 0, (void *)"M | ", 0);
      x = add_disp_val(outp, INT, y, x, &(d_miners[i].t_hash), 3);
      x = add_disp_val(outp, STRING, y, x, (void *)"M|", 0);
      x = add_disp_val(outp, FLOAT, y, x, &(d_miners[i].t_temp), 4);
      x = add_disp_val(outp, STRING, y, x, (void *)"|", 0);
      x = add_disp_val(outp, TIME_SPAN, y, x, &(d_miners[i].uptime), 0);
      x = add_disp_val(outp, STRING, y, x, (void *)"|", 0);
      add_disp_val(outp, SHARE_STATE, y, x, &(d_miners[i].t_accepted_s), 0);
    }
    outp->priv = d_miners;
  }
}

void update_u8g2(Disp_outp *outp) {
  if (outp->type == SSD1322_256X64) {
    int i, j, changed, redraw = 1;//SYS_REDRAW_SCR;
    Disp_val *disp_val = outp->disp_val;
    Miner_s *d_miners = (Miner_s *)outp->priv;
    String str;

    memset (outp->priv, 0, sizeof(Miner_s) * SSD1322_256X64_MAX_MINERS);
    for (i = 0, j = 0; i < sys_cfg.miners_num && j < SSD1322_256X64_MAX_MINERS; i++) {
      if (!miners[i].enabled || miners_s[i].gpu_num == 0)
        continue;
      memcpy(&d_miners[j++], &miners_s[i], sizeof(Miner_s));
    }

    u8g2.clearBuffer();
    for (i = 0; i < outp->last_val; i++) {
      changed = check_disp_val(&disp_val[i], redraw, str);
      if (changed) {
        //dlog(str + " " + disp_val[i].y + " " + disp_val[i].x + "\r\n");
        u8g2.drawStr(disp_val[i].x, disp_val[i].y, str.c_str());
      }
    }
    u8g2.sendBuffer();

    return;
  }

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
  for (i = 0; i < sys_cfg.miners_num; i++)
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

void memcpy_mirror_x(uint8_t dest[], uint8_t src[], int16_t w, int16_t h)
{
  int16_t i, byte_w = (w + 7) / 8;

  for (i = 0; i < h; i++)
    memcpy(&dest[i * byte_w], &src[(h - i - 1) * byte_w], byte_w);
}

#define B_RT 8   // epd.width() / EPD_IMG_W
#define B_RT_IMG "/btc%d.bmp"
uint8_t *b_rt_buf[B_RT], b_rt_cnt;

void init_epd(Disp_outp *outp) {
  File f;
  size_t ret;
  void *val1;
  char *fname;
  enum disp_val_type type;
  int i, j, n, x, y, val2_base, val2;
  const GFXfont *font = &FreeMonoBold12pt7b;
  uint8_t f_buf[BMP_FIL_SZ], *img_buf, fw, fh;
  void *disp[][5] = {
    {(void*)"/mhash.bmp", (void*)INT,   &(miners_s[0].t_hash), (void*)3, (void*)"M"},
    {(void*)"/mtemp.bmp", (void*)FLOAT, &(miners_s[0].t_temp), (void*)4, (void*)"C"},
    {(void*)"/mtime.bmp", (void*)INT,   &(miners_s[0].uptime), (void*)3, (void*)"H"} };

  if (outp == NULL || outp->type != GxEPD2_213)
    return;

  if ('0' < font->first || '0' > font->last)
    font = &FreeMonoBold12pt7b;

  fw = font->glyph['0' - font->first].xAdvance;
  fh = font->glyph['0' - font->first].height;
  val2_base = (fw << 16) | (fh << 8);

  n = sizeof(disp) / sizeof(disp[0]);
  y = EPD_PRE_Y;
  for (i = 0; i < sys_cfg.miners_num; i++) {
    for (j = 0; j < n; j++) {
      f = SPIFFS.open((const char *)disp[j][0], "r");
      if (!f)
        continue;
      ret = f.readBytes((char *)f_buf, sizeof(f_buf));
      f.close();
      if (ret != sizeof(f_buf))
        continue;

      img_buf = (uint8_t *)malloc((EPD_IMG_H * EPD_IMG_W) >> 3);
      if (!img_buf)
        continue;

      memcpy_mirror_x(img_buf, &f_buf[BMP_HDR_SZ], EPD_IMG_W, EPD_IMG_H);
      type = (enum disp_val_type)(int)disp[j][1];
      val1 = (void *)(disp[j][2] + i * sizeof(Miner_s));
      val2 = val2_base | ((int)disp[j][3] & 0xFF);

      x = EPD_PRE_X;
      add_disp_val(outp, IMG, y, x, (void *)img_buf, 0);
      x += (EPD_IMG_W + EPD_INT_X);
      add_disp_val(outp, type, y + ((EPD_IMG_H + fh) >> 1), x, val1, val2);
      add_disp_val(outp, STRING, y + ((EPD_IMG_H + fh) >> 1),
                   epd.width() - fw - EPD_PRE_X - 7, disp[j][4], val2_base);
      y += (EPD_IMG_H + EPD_INT_Y);
    }
  }

  fname = strdup(B_RT_IMG);
  b_rt_cnt = 0;
  for (i = 0; i < B_RT; i++) {
    sprintf(fname, B_RT_IMG, i);
    f = SPIFFS.open((const char *)fname, "r");
    if (!f)
      continue;
    ret = f.readBytes((char *)f_buf, sizeof(f_buf));
    f.close();
    if (ret != sizeof(f_buf))
      continue;

    b_rt_buf[i] = (uint8_t *)malloc((EPD_IMG_H * EPD_IMG_W) >> 3);
    if (!b_rt_buf[i])
      continue;

    memcpy_mirror_x(b_rt_buf[i], &f_buf[BMP_HDR_SZ], EPD_IMG_W, EPD_IMG_H);
  }

  epd.init();
  epd.fillScreen(GxEPD_WHITE);
  epd.setTextColor(GxEPD_BLACK);
  epd.setFont(font);
  bitSet(sys_stat.flags, SYS_REDRAW_EPD_BIT);
}

void update_epd(Disp_outp *outp) {
  int i, changed, redraw, fw, fh, orig_val2;
  Disp_val *dval;
  String str;

  redraw = SYS_REDRAW_EPD;
  bitClear(sys_stat.flags, SYS_REDRAW_EPD_BIT);
  for (i = 0; i < outp->last_val; i++) {
    dval = &outp->disp_val[i];
    if (dval->type == IMG && redraw) {
      epd.drawInvertedBitmap(dval->x, dval->y, (uint8_t *)dval->val1,
                             EPD_IMG_W, EPD_IMG_H, GxEPD_BLACK);
      continue;
    }
    orig_val2 = dval->val2;
    dval->val2 &= 0xFF;
    changed = check_disp_val(dval, redraw, str);
    dval->val2 = orig_val2;
    if (changed) {
      fw = (orig_val2 >> 16) & 0xFF;
      fh = (orig_val2 >> 8) & 0xFF;
      epd.fillRect(dval->x, dval->y - fh, str.length() * fw + 1,
                   fh + 1, GxEPD_WHITE);
      epd.setCursor(dval->x, dval->y);
      epd.print(str);
    }
  }

  epd.fillRect(0, epd.height() - EPD_IMG_H, epd.width(), EPD_IMG_H, GxEPD_WHITE);
  epd.drawInvertedBitmap((b_rt_cnt & 7) * 16, epd.height() - EPD_IMG_H,
                b_rt_buf[(b_rt_cnt & 7)], EPD_IMG_W, EPD_IMG_H, GxEPD_BLACK);
  b_rt_cnt++;

  epd.display(true);
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
