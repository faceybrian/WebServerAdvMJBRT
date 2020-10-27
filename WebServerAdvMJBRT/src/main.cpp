/*  
 * With inspiration from:
 * 
 * https://shawnhymel.com/1882/how-to-create-a-web-server-with-websockets-using-an-esp32-in-arduino/
 *
 * Author: J. Sanggaard 
 * Date: 10. september 2020
 *  
 */
#include <global.h>
#include <ESP32Encoder.h>
#include "pid.h"
#include <WiFi.h>
//#include <WiFiClientSecure.h>
//#include <FS.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <hbridge.h>

TaskHandle_t PidTaskHandle;
TaskHandle_t MotionTaskHandle;
TaskHandle_t parm_update_taskHandle;
TaskHandle_t KinTaskHandle;

ESP32Encoder encoder;
Pid pid_vel(DT_S, MIN_CTRL_VALUE, MAX_CTRL_VALUE);
Pid pid_pos(DT_S, MIN_CTRL_VALUE / 5, MAX_CTRL_VALUE / 5);
H_Bridge hbridge;

const double integration_threshold = 200;

volatile double req_pos;
volatile double req_vel;
volatile int64_t current_pos;
volatile double current_vel;

double ctrl_pos;
double ctrl_vel;

bool mode_pos = true;

/* uncomment for router connection */
#define SOFT_AP

/* Constants */
#ifdef SOFT_AP
const char *ssid = "Le_Rat";
const char *password = "ratruler";
#else
const char *ssid = "MakitaNG";
const char *password = "...";
#endif

const char *cmd_toggle = "toggle";
const char *cmd_led_state = "led_state";
const char *cmd_sli = "sli";
const char *cmd_pid = "pid_";

const int32_t wifi_channel = 6; // alle grupper skal have hver sin kanal
const int32_t dns_port = 53;
const int32_t http_port = 80;
const int32_t ws_port = 1337;
const int32_t led_pin = 17;

// Globals
AsyncWebServer Server(http_port);
WebSocketsServer WebSocket = WebSocketsServer(ws_port);

char MsgBuf[32];
int32_t LedState = 0;
int32_t SliderVal = 0;
double KpVal = 0;
double KiVal = 0;
double KdVal = 0;
double PosVal = 0;
int32_t M1Val = 0;
int32_t M2Val = 0;


/***********************************************************
 * Functions
 */
bool str_to_double(char *str, double *d)
{
  errno = 0;
  char *e;
  double result = strtod(str, &e);
  if (*e == '\0' && 0 == errno) // no error
  {
    *d = result;
    return true;
  }
  return false;
}

bool str_to_int(char *str, int32_t *d)
{
  errno = 0;
  char *e;
  int32_t result = strtol(str, &e, 10);
  if (*e == '\0' && 0 == errno) // no error
  {
    *d = result;
    return true;
  }
  return false;
}

void web_socket_send(const char *buffer, uint8_t client_num)
{
  log_d("Sending to [%u]: %s", client_num, buffer);
  WebSocket.broadcastTXT(buffer, strlen(buffer)); // all clients (opdaterer værdier på alle clients)
  //webSocket.sendTXT(client_num, msg_buf); // only one client
}

void parm_update_task(void *arg)
{
  while (1)
  {
    pid_pos.set_kp(KpVal);
    pid_pos.set_ki(KiVal);
    pid_pos.set_kd(KdVal);
    vTaskDelay(1000);

    //*Serial.printf(
    // "Kp: %.2f  Ki: %.2f  Kd: %.2f  \n\r",
    // KpVal, KiVal, KdVal);

    //Serial.printf(
    //"req_pos: %.2f  curr_pos: %.2f  ctrl_pos: %.2f  set_vel: %.2f  curr_vel: %.2f ctrl_vel: %.2f\n\r",
    //req_pos, (double)current_pos, ctrl_pos, req_vel, current_vel, ctrl_vel);
  }
}

void pid_task(void *arg)
{

  int64_t prev_pos = current_pos;

  TickType_t xTimeIncrement = configTICK_RATE_HZ * pid_pos.get_dt();
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  { // loop tager mindre end 18us * 2
    digitalWrite(PIN_PID_LOOP, HIGH);

    current_pos = encoder.getCount();
    current_vel = (current_pos - prev_pos) / DT_S;

    if (mode_pos)
    {
      pid_pos.update(req_pos, current_pos, &ctrl_pos, integration_threshold);

      req_vel = ctrl_pos;
    }

    pid_vel.update(req_vel, current_vel, &ctrl_vel, 100000);

    hbridge.set_pwm(ctrl_vel);

    prev_pos = current_pos;
    digitalWrite(PIN_PID_LOOP, LOW);
    vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);
  }
}

void wait_move()
{
  while (abs(req_pos - current_pos) > 10)
  {
    vTaskDelay(1);
  }
}

void set_pos(int32_t pos)
{
  req_pos = pos;
}

void motion_task(void *arg)
{
  //home();

  vTaskDelay(1000);
  int32_t n = 1;
  int32_t steps_pr_deg = 1920;

  //TickType_t xTimeIncrement = configTICK_RATE_HZ/10;
  //TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    log_v("motion_task...");

    set_pos(steps_pr_deg * n);
    wait_move();
    vTaskDelay(5000);
    n++;
    //vTaskDelayUntil( &xLastWakeTime, xTimeIncrement);
  }
}

void kin_task(void *arg) // Log task placeholder 
{
  while(1) 
  {
    vTaskDelay(1000);
    log_d("PosVal: %4.2f, M1Val: %4.2d, M2Val: %4.2d", PosVal, M1Val, M2Val);
  }
}

void handle_toggle()
{
  LedState = LedState ? 0 : 1;
  log_i("Toggling LED to %u", LedState);
  digitalWrite(led_pin, LedState);
}

void handle_led_state(uint8_t client_num)
{
  sprintf(MsgBuf, "%d", LedState);
  web_socket_send(MsgBuf, client_num);
}

void handle_slider(char *command, uint8_t client_num)
{
  char *value = strstr(command, ":");

  if (value == NULL || *value != ':')
  {
    log_e("[%u]: Bad command %s", client_num, command);
    return;
  }

  if (*(value + 1) == '?')
  {
    sprintf(MsgBuf, "%s:%d", cmd_sli, SliderVal);
    web_socket_send(MsgBuf, client_num);
  }
  else
  {
    errno = 0;
    char *e;
    int32_t result = strtol(value + 1, &e, 10);
    if (*e == '\0' && 0 == errno) // no error
    {
      SliderVal = result;
      log_d("[%u]: Slidervalue received %d", client_num, SliderVal);
    }
    else
    {
      log_e("[%u]: illegal Slidervalue received: %s", client_num, value + 1);
    }
  }
}

void handle_kx(char *command, uint8_t client_num)
{
  char *value = strstr(command, ":");

  if (value == NULL || *value != ':')
  {
    log_e("[%u]: Bad command %s", client_num, command);
    return;
  }

  char subtype = *(value - 1);

  double *parm_value;

  switch (subtype)
  {
  case 'p':
    parm_value = &KpVal;
    break;
  case 'i':
    parm_value = &KiVal;
    break;
  case 'd':
    parm_value = &KdVal;
    break;
  default:
    log_e("[%u]: Bad command %s", client_num, command);
    return;
    break;
  }

  if (*(value + 1) == '?')
  {
    sprintf(MsgBuf, "%sk%c:%f", cmd_pid, subtype, *parm_value);
    web_socket_send(MsgBuf, client_num);
  }
  else
  {
    errno = 0;
    char *e;
    double result = strtod(value + 1, &e);
    if (*e == '\0' && 0 == errno) // no error
    {
      *parm_value = result;
      log_d("[%u]: k%c value received %f", client_num, subtype, *parm_value);
    }
    else
    {
      log_e("[%u]: illegal format of k%c value received: %s", client_num, subtype, value + 1);
    }
  }
}

void handle_pos(char *command, uint8_t client_num)
{
  char *value = strstr(command, ":");

  if (value == NULL || *value != ':')
  {
    log_e("[%u]: Bad command %s", client_num, command);
    return;
  }
  
  if (*(value + 1) == '?') // hvis value er en request (har '?'), send value til 
  {
    sprintf(MsgBuf, "pos:%.2f", PosVal);
    web_socket_send(MsgBuf, client_num);
  }
  else
  {
    if(!str_to_double(value + 1, &PosVal)) // Hvis vi ikke kan konvertere string til double, gælder dette
    {
      log_e("Error in PosVal, illegal value!: %s", value+1);
    }
  }
}

void handle_m1(char *command, uint8_t client_num)
{
  char *value = strstr(command, ":");

  if (value == NULL || *value != ':')
  {
    log_e("[%u]: Bad command %s", client_num, command);
    return;
  }

  if (*(value + 1) == '?')
  {
    sprintf(MsgBuf, "m1:%.2d", M1Val);  
    web_socket_send(MsgBuf, client_num);
  }
  else 
  {
    if(!str_to_int(value + 1, &M1Val)) // Hvis vi ikke kan konvertere string til double, gælder dette
    {
      log_e("Error in M1Val, illegal value!: %s", value+1);
    }
  }
}

void handle_m2(char *command, uint8_t client_num)
{
  char *value = strstr(command, ":");

  if (value == NULL || *value != ':')
  {
    log_e("[%u]: Bad command %s", client_num, command);
    return;
  }

  if (*(value + 1) == '?')
  {
    sprintf(MsgBuf, "m2:%.2d", M2Val);
    web_socket_send(MsgBuf, client_num);
  }
  else 
  {
    if(!str_to_int(value + 1, &M2Val))
    {
      log_e("Error in M2Val, illegal value!: %s", value+1);
    }
  }
}

void handle_command(uint8_t client_num, uint8_t *payload, size_t length)
{
  char *command = (char *)payload;

  log_d("[%u] Received text: %s", client_num, command);

  if (strcmp(command, cmd_toggle) == 0)
    handle_toggle(); // Toggle LED
  else if (strncmp(command, cmd_led_state, strlen(cmd_led_state)) == 0)
    handle_led_state(client_num); // Report the state of the LED
  else if (strncmp(command, cmd_sli, strlen(cmd_sli)) == 0)
    handle_slider(command, client_num); // slider
  else if (strncmp(command, cmd_pid, strlen(cmd_pid)) == 0)
    handle_kx(command, client_num); // pid params
  // pos, m1 & m2
  else if (strncmp(command, "pos", 3) == 0)
    handle_pos(command, client_num); // position
  else if (strncmp(command, "m1", 2) == 0)
    handle_m1(command, client_num); // m1
  else if (strncmp(command, "m2", 2) == 0)
    handle_m2(command, client_num); // m2
  else
    log_e("[%u] Message not recognized", client_num);

  WebSocket.connectedClients();
}

// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num,
                      WStype_t type,
                      uint8_t *payload,
                      size_t length)
{
  // Figure out the type of WebSocket event
  switch (type)
  {

  // Client has disconnected
  case WStype_DISCONNECTED:
    log_i("[%u] Disconnected!", client_num);
    break;

  // New client has connected
  case WStype_CONNECTED:
  {
    IPAddress ip = WebSocket.remoteIP(client_num);
    log_i("[%u] Connection from ", client_num);
    log_i("IP: %s", ip.toString().c_str());
  }
  break;

  // Handle text messages from client
  case WStype_TEXT:
    handle_command(client_num, payload, length);
    break;

  // For everything else: do nothing
  case WStype_BIN:
  case WStype_ERROR:
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
  default:
    break;
  }
}

void handleRequest(AsyncWebServerRequest *request,
                   const char *file_name,
                   const char *content_type)
{
  IPAddress remote_ip = request->client()->remoteIP();
  log_i("HTTP GET request of %s from %s", request->url().c_str(), remote_ip.toString().c_str());
  request->send(SPIFFS, file_name, content_type);
}

// Callback: send homepage
void onIndexRequest(AsyncWebServerRequest *request)
{
  handleRequest(request, "/index.html", "text/html");
}

// Callback: send style sheet
void onCSSRequest(AsyncWebServerRequest *request)
{
  handleRequest(request, "/robot.css", "text/css");
}

// Callback: send style sheet
void onCSSRequestW3(AsyncWebServerRequest *request)
{
  handleRequest(request, "/w3.css", "text/css");
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request)
{
  IPAddress remote_ip = request->client()->remoteIP();
  log_i("HTTP GET request of %s from %s", request->url().c_str(), remote_ip.toString().c_str());
  request->send(404, "text/plain", "Not found");
}

void setup_spiffs()
{
  // Make sure we can read the file system
  if (!SPIFFS.begin())
  {
    log_e("Error mounting SPIFFS");
    while (1)
      ;
  }
}

void setup_network()
{
#ifdef SOFT_AP
  // Start access point
  WiFi.softAP(ssid, password, wifi_channel); // (alle grupper skal bruge en unik kanal)
  // Print our IP address

  log_i("AP running");
  log_i("My IP address: ");
  log_i("IP: %s", WiFi.softAPIP().toString().c_str());

#else
  // connect to local network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    log_i("Establishing connection to WiFi..");
  }
  log_i("Connected to network");
  log_i("IP: %s", WiFi.localIP().toString().c_str());
#endif

  // On HTTP request for root, provide index.html file
  Server.on("/", HTTP_GET, onIndexRequest);

  // On HTTP request for style sheet, provide robot.css
  Server.on("/robot.css", HTTP_GET, onCSSRequest);
  Server.on("/w3.css", HTTP_GET, onCSSRequestW3);

  // Handle requests for pages that do not exist
  Server.onNotFound(onPageNotFound);

  // Start web server
  Server.begin();

  // Start WebSocket server and assign callback
  WebSocket.begin();
  WebSocket.onEvent(onWebSocketEvent);
}

void setup_tasks()
{

  log_v("starting pid task");
  xTaskCreatePinnedToCore(
      pid_task,       /* Function to implement the task */
      "pid_task",     /* Name of the task */
      10000,          /* Stack size in words */
      NULL,           /* Task input parameter */
      3,              /* Priority of the task from 0 to 25, higher number = higher priority */
      &PidTaskHandle, /* Task handle. */
      1);             /* Core where the task should run */

  log_v("starting motion task");
  xTaskCreatePinnedToCore(
      motion_task,
      "motion_task",
      10000, /* Stack size in words */
      NULL,  /* Task input parameter */
      2,     /* Priority of the task from 0 to 25, higher number = higher priority */
      &MotionTaskHandle,
      0); /* Core where the task should run */

  log_v("starting pid task");
  xTaskCreatePinnedToCore(
      parm_update_task,        /* Function to implement the task */
      "parm_update_task",      /* Name of the task */
      10000,                   /* Stack size in words */
      NULL,                    /* Task input parameter */
      2,                       /* Priority of the task from 0 to 25, higher number = higher priority */
      &parm_update_taskHandle, /* Task handle. */
      1);                      /* Core where the task should run */
  
  log_v("starting kin task");
  xTaskCreatePinnedToCore(
      kin_task,       /* Function to implement the task */
      "kin_task",     /* Name of the task */
      10000,          /* Stack size in words */
      NULL,           /* Task input parameter */
      3,              /* Priority of the task from 0 to 25, higher number = higher priority */
      &KinTaskHandle, /* Task handle. */
      1);             /* Core where the task should run */
}

void setup()
{
  // Init LED and turn off
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);

  // Start Serial port
  Serial.begin(115200);
  pinMode(PIN_PID_LOOP, OUTPUT);
  pinMode(PIN_LIMIT_SW, INPUT);

  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachFullQuad(PIN_ENC_A, PIN_ENC_B);    // Attache pins for use as encoder pins
  encoder.clearCount();

  hbridge.begin(PIN_MOTOR_CTRL, PIN_MOTOR_INA, PIN_MOTOR_INB, PWM_FREQ_HZ, PWM_RES_BITS, PWM_CH);

  pid_pos.set_kp(10.0); //12
  pid_pos.set_ki(0.0);
  pid_pos.set_kd(0.000);

  pid_vel.set_kp(0.0);
  pid_vel.set_ki(20);
  pid_vel.set_kd(0.000);

  setup_spiffs();
  setup_network();
  setup_tasks();
}

void loop()
{
  // Look for and handle WebSocket data
  WebSocket.loop();
}