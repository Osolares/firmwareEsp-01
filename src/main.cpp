// VERY IMPORTANT
// to see colors in terminals add this line at the end of platformio.ini
// monitor_flags = --raw
#include <Arduino.h>
#include "Colors.h"
#include "IoTicosSplitter.h"
#include <ArduinoJson.h>

// librerias AP-WS
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <ESP8266HTTPClient.h>

#include <Servo.h>
// #include <Scheduler.h>

// Configuración del AP
const char *ap_ssid = "OIOT";
const char *ap_password = "12342468";
IPAddress local_IP(192, 168, 1, 5);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// Preferences preferences;
AsyncWebServer server(80);

// Variables globales
String local_wifi_ssid, local_wifi_password;
String local_print_stats, local_dId, local_webhook_pass, local_webhook_ep, local_mqtt_server;
String dId, webhook_pass, webhook_ep;
const char *mqtt_server, *wifi_ssid, *wifi_password;
bool print_monit = false;

// Tamaño de la EEPROM
#define EEPROM_SIZE 512

String readFromEEPROM(int address)
{
  String value = "";
  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < EEPROM_SIZE; i++)
  {
    char c = EEPROM.read(address + i);
    if (c == 0)
      break; // Terminar si encontramos un carácter nulo
    value += c;
  }
  EEPROM.end();
  return value;
}

// Función para escribir un String en EEPROM
void writeStringToEEPROM(int address, const String &data)
{
  EEPROM.begin(EEPROM_SIZE);
  int len = data.length();
  for (int i = 0; i < len; i++)
  {
    EEPROM.write(address + i, data[i]);
  }
  EEPROM.write(address + len, '\0'); // Añade terminador nulo
  EEPROM.commit();                   // Guarda los cambios en la EEPROM
}

// Función para leer un String desde EEPROM
String readStringFromEEPROM(int address)
{
  EEPROM.begin(EEPROM_SIZE);
  String value = "";
  char k;
  for (int i = 0; i < 100; i++) // Limita a 100 caracteres máximo
  {
    k = EEPROM.read(address + i);
    if (k == '\0')
      break;
    value += k;
  }
  return value;
}

// Función para leer o escribir un valor por defecto si no existe
String readOrWriteEEPROM(int address, const String &defaultValue)
{
  String value = readStringFromEEPROM(address);
  if (value.length() == 0)
  {
    writeStringToEEPROM(address, defaultValue);
    return defaultValue;
  }
  return value;
}

void loadPreferences()
{
  EEPROM.begin(EEPROM_SIZE);

  local_wifi_ssid = readOrWriteEEPROM(0, "INTER_MOTORES");
  local_wifi_password = readOrWriteEEPROM(50, "!nt3rm0t0r35");
  local_dId = readOrWriteEEPROM(100, "123456");
  local_print_stats = readOrWriteEEPROM(125, "false");
  local_webhook_pass = readOrWriteEEPROM(150, "N9vhTTS95u");
  local_webhook_ep = readOrWriteEEPROM(200, "http://3.12.220.35:3001/api/getdevicecredentials");
  local_mqtt_server = readOrWriteEEPROM(300, "3.12.220.35");

  dId = local_dId;
  webhook_pass = local_webhook_pass;
  webhook_ep = local_webhook_ep;

  mqtt_server = local_mqtt_server.c_str();
  wifi_ssid = local_wifi_ssid.c_str();
  wifi_password = local_wifi_password.c_str();
  print_monit = (local_print_stats == "true"); // Convertir "on" a true, "off" a false
}

// Función para imprimir datos en el Monitor Serie
void printStoredData()
{
  Serial.println("\n📌 Datos Cargados desde EEPROM:");
  Serial.println("SSID: " + local_wifi_ssid);
  Serial.println("Password: " + local_wifi_password);
  Serial.println("Device ID: " + local_dId);
  Serial.println("Webhook Pass: " + local_webhook_pass);
  Serial.println("Webhook Endpoint: " + local_webhook_ep);
  Serial.println("MQTT Server: " + local_mqtt_server);
  Serial.println("Monitor: " + local_print_stats);

  Serial.println("\n📌 Variables Globales:");
  Serial.println("dId: " + dId);
  Serial.println("webhook_pass: " + webhook_pass);
  Serial.println("webhook_ep: " + webhook_ep);
  Serial.println("Monitor: " + print_monit);

  Serial.println("\n📌 Valores para conexión:");
  Serial.println("WiFi SSID: " + String(wifi_ssid));
  Serial.println("WiFi Password: " + String(wifi_password));
}

// PINS
#define PIN_LED 1
#define PIN_AP_TRIGGER 3
#define PIN_RELE 0
#define PIN_SERVO 2

// Functions definitions
void loadPreferences();
bool get_mqtt_credentials();
void check_mqtt_connection();
bool reconnect();
void process_sensors();
void process_actuators();
void send_data_to_broker();
void callback(char *topic, byte *payload, unsigned int length);
void process_incoming_msg(String topic, String incoming);
void print_stats();
void clear();
void softAPConfig();
void scanNetworks();
void printStoredData();

void handleSetTime(float time);
void handleServoOn();
void handleServoOff();

// Global Vars
WiFiClient espclient;
PubSubClient client(espclient);
IoTicosSplitter splitter;
long lastReconnectAttemp = 0;
long varsLastSend[20];
long closeMonitAttemp = 0;
String last_received_msg = "";
String last_received_topic = "";
int prev_temp = 0;
int prev_hum = 0;
bool prev_status_servo = false;
bool prev_status_rele = false;

bool apModeActive = false;      // Variable global para controlar el estado de AP
unsigned long lastScanTime = 0; // Variable para manejar el tiempo de escaneo
bool scanning = false;          // Estado del escaneo de redes

DynamicJsonDocument mqtt_data_doc(2048);

// Variables globales para guardar las redes encontradas
String networksList = "";

void listFiles(File dir, int numTabs);

Servo servo;
// Variables para controlar el servo
bool isServoOn = false;                 // Estado del servo (encendido/apagado)
int targetTurns = 0;                    // Número de vueltas completas
bool isTimedMode = false;               // Modo de tiempo activado
int targetTime = 0;                     // Tiempo en minutos
unsigned long startTime = 0;            // Tiempo de inicio
unsigned long irrigationTime = 0;       // Tiempo de inicio
unsigned long irrigationTargetTime = 1; // horas tiempo de tiempo de riego
static unsigned long modoAPTime = 0;    // Variable estática para mantener el tiempo inicial
bool triggerActivated = false;          // Bandera para detectar el primer cambio a LOW

void setup()
{

  Serial.begin(115200);
  pinMode(PIN_AP_TRIGGER, INPUT); // Configurar el pin como entrada con resistencia interna
  pinMode(PIN_RELE, OUTPUT);
  pinMode(PIN_SERVO, OUTPUT);
  // pinMode(PIN_LED, OUTPUT);

  digitalWrite(PIN_LED, HIGH);
  digitalWrite(PIN_RELE, HIGH); // Desactiva el relé (HIGH)

  // Iniciar el servo
  servo.attach(PIN_SERVO);
  servo.write(90); // Detener el servo (posición neutra)

  clear();
  LittleFS.begin();
  EEPROM.begin(EEPROM_SIZE);

  loadPreferences();
  printStoredData();
  // Listar archivos en LittleFS
  Serial.println("Archivos en LittleFS:");
  File root = LittleFS.open("/", "r");
  listFiles(root, 0); // Llamar a la función para listar archivos

  if (digitalRead(PIN_AP_TRIGGER) == LOW)
  {
    if (!triggerActivated)
    {
      // Primera vez que se detecta LOW
      modoAPTime = millis();   // Guardar el tiempo inicial
      triggerActivated = true; // Marcar que el trigger se activó
    }

    // Verificar si han pasado 3 segundos desde que se activó el trigger
    if (millis() - modoAPTime >= 1000)
    {
      softAPConfig();
      triggerActivated = false; // Reiniciar la bandera después de ejecutar la función
    }
  }
  else
  {
    // Si el pin no está en LOW, reiniciar la bandera
    triggerActivated = false;
  }

  if (!apModeActive)
  {

    Serial.print(underlinePurple + "\n\n\nWiFi Connection in Progress" + fontReset + Purple);

    int counter = 0;
    Serial.print("📡 Intentando conectar a WiFi con: ");
    Serial.print("SSID: ");
    Serial.println(wifi_ssid);
    Serial.print("Password: ");
    Serial.println(wifi_password);

    // WiFi.mode(WIFI_STA); // Modo estación
    WiFi.disconnect(); // Asegurar una conexión limpia
    delay(1000);

    WiFi.mode(WIFI_STA); // Modo estación

    WiFi.begin(wifi_ssid, wifi_password);

    while (WiFi.status() != WL_CONNECTED)
    {

      if (digitalRead(PIN_AP_TRIGGER) == LOW)
      {
        if (!triggerActivated)
        {
          // Primera vez que se detecta LOW
          modoAPTime = millis();   // Guardar el tiempo inicial
          triggerActivated = true; // Marcar que el trigger se activó
        }

        // Verificar si han pasado 3 segundos desde que se activó el trigger
        if (millis() - modoAPTime >= 1000)
        {
          softAPConfig();
          triggerActivated = false; // Reiniciar la bandera después de ejecutar la función
        }
      }
      else
      {
        // Si el pin no está en LOW, reiniciar la bandera
        triggerActivated = false;
      }

      delay(500);
      Serial.print(".");
      counter++;

      if (counter > 30)
      {
        Serial.print("  ⤵" + fontReset);
        Serial.print(Red + "\n\n         Ups WiFi Connection Failed :( ");
        Serial.println(" -> Restarting..." + fontReset);
        digitalWrite(PIN_LED, HIGH);
        delay(2000);
        ESP.restart();
      }
    }

    Serial.print("  ⤵" + fontReset);

    // Printing local ip
    Serial.println(boldGreen + "\n\n         WiFi Connection -> SUCCESS :)" + fontReset);
    Serial.print("\n         Local IP -> ");
    Serial.print(boldBlue);
    Serial.print(WiFi.localIP());
    Serial.println(fontReset);

    client.setCallback(callback);
    digitalWrite(PIN_LED, LOW);
  }
}

// Función para listar los archivos en SPIFFS
void listFiles(File dir, int numTabs)
{
  while (true)
  {
    File entry = dir.openNextFile();
    if (!entry)
    {
      // No hay más archivos
      break;
    }
    // Imprimir nombre de archivo
    for (int i = 0; i < numTabs; i++)
    {
      Serial.print("\t");
    }
    Serial.print(entry.name());
    if (entry.isDirectory())
    {
      Serial.println("/");
      listFiles(entry, numTabs + 1); // Llamar recursivamente si es un directorio
    }
    else
    {
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC); // Tamaño del archivo
    }
    entry.close();
  }
}

void loop()
{

  // Controlar el servo en modo de tiempo
  if (isTimedMode)
  {
    if (millis() - startTime >= targetTime * 60000)
    {                      // Convertir minutos a milisegundos
      isServoOn = false;   // Apagar el servo después del tiempo
      isTimedMode = false; // Desactivar el modo de tiempo
      servo.write(90);     // Volver a la posición de descanso (cerrado)
      varsLastSend[9] = 0;
    }
  }

  if (apModeActive)
  {
    digitalWrite(PIN_LED, LOW);
    delay(2000); // Reducir el consumo de CPU
    digitalWrite(PIN_LED, HIGH);
    delay(2000); // Reducir el consumo de CPU
    // Mientras estemos en modo AP, no hacer nada más
    return; // Terminar la ejecución de loop() temporalmente
  }

  if (digitalRead(PIN_AP_TRIGGER) == LOW)
  {
    if (!triggerActivated)
    {
      // Primera vez que se detecta LOW
      modoAPTime = millis();   // Guardar el tiempo inicial
      triggerActivated = true; // Marcar que el trigger se activó
    }

    // Verificar si han pasado 3 segundos desde que se activó el trigger
    if (millis() - modoAPTime >= 1000)
    {
      softAPConfig();
      triggerActivated = false; // Reiniciar la bandera después de ejecutar la función
    }
  }
  else
  {
    // Si el pin no está en LOW, reiniciar la bandera
    triggerActivated = false;
  }

  check_mqtt_connection();
}

// USER FUNTIONS ⤵
void process_sensors()
{

  // Controlar el servo en modo de tiempo
  if (isServoOn)
  {
    if (millis() - irrigationTime >= irrigationTargetTime * 60 * 60000)
    {

      mqtt_data_doc["variables"][0]["last"]["value"] = 100;
      mqtt_data_doc["variables"][0]["last"]["save"] = 1;
      isServoOn = false;   // Apagar el servo después del tiempo
      isTimedMode = false; // Desactivar el modo de tiempo
      irrigationTime = millis();
      varsLastSend[0] = 0;
      Serial.println("Servo desactivado por estar mucho tiempo activo");
    }
  }

  // get temp simulation
  int temp = random(1, 100);

  // mqtt_data_doc["variables"][0]["last"]["value"] = temp;

  // save temp?
  int dif = temp - prev_temp;
  if (dif < 0)
  {
    dif *= -1;
  }

  if (dif >= 40)
  {
    // mqtt_data_doc["variables"][0]["last"]["save"] = 1;
  }
  else
  {
    // mqtt_data_doc["variables"][0]["last"]["save"] = 0;
  }

  // get PIN_LED status
  if (true == isServoOn)
  {
    mqtt_data_doc["variables"][0]["last"]["value"] = 50;
    // Serial.println("Servo on");

    irrigationTime = millis();
  }
  else
  {
    mqtt_data_doc["variables"][0]["last"]["value"] = 0;
    // Serial.println("Servo off");
  }

  // get PIN_LED status
  if (prev_status_servo != isServoOn)
  {
    mqtt_data_doc["variables"][0]["last"]["save"] = 1;
    // Serial.println("Servo save on");
  }
  else
  {
    mqtt_data_doc["variables"][0]["last"]["save"] = 0;
    // Serial.println("Servo save off");
  }

  // prev_temp = temp;
  prev_status_servo = isServoOn;

  // get humidity simulation
  int hum = random(1, 50);
  // mqtt_data_doc["variables"][1]["last"]["value"] = hum;

  // save hum?
  dif = hum - prev_hum;
  if (dif < 0)
  {
    dif *= -1;
  }

  if (dif >= 20)
  {
    // mqtt_data_doc["variables"][1]["last"]["save"] = 1;
  }
  else
  {
    // mqtt_data_doc["variables"][1]["last"]["save"] = 0;
  }

  // int dif = hum;
  bool status_rele = (digitalRead(PIN_RELE) == LOW);

  // get PIN_LED status
  if (true == status_rele)
  {
    mqtt_data_doc["variables"][1]["last"]["value"] = 50;
    // Serial.println("Servo on");

    irrigationTime = millis();
  }
  else
  {
    mqtt_data_doc["variables"][1]["last"]["value"] = 0;
    // Serial.println("Servo off");
  }

  // get PIN_LED status
  if (prev_status_rele != status_rele)
  {
    mqtt_data_doc["variables"][1]["last"]["save"] = 1;
    // Serial.println("Servo save on");
  }
  else
  {
    mqtt_data_doc["variables"][1]["last"]["save"] = 0;
    // Serial.println("Servo save off");
  }

  // prev_temp = temp;
  prev_status_rele = status_rele;

  mqtt_data_doc["variables"][3]["last"]["value"] = (true == isServoOn);
  mqtt_data_doc["variables"][6]["last"]["value"] = (LOW == digitalRead(PIN_RELE));
}

void process_actuators()
{
  // Imprimir el objeto JSON en el Monitor Serial

  serializeJson(mqtt_data_doc, Serial); // Imprime el JSON en la consola
  Serial.println();                     // Agrega un salto de línea
  if (mqtt_data_doc["variables"][2]["last"]["value"] == "on")
  {
    digitalWrite(PIN_LED, HIGH);
    // digitalWrite(PIN_SERVO, HIGH);
    handleServoOn();
    Serial.println("Process Actuators [2]: on ");
    mqtt_data_doc["variables"][2]["last"]["value"] = "";
    varsLastSend[3] = 0;
    varsLastSend[0] = 0;
  }
  else if (mqtt_data_doc["variables"][4]["last"]["value"] == "off")
  {
    digitalWrite(PIN_LED, LOW);
    // digitalWrite(PIN_SERVO, LOW);
    handleServoOff();
    Serial.println("Process Actuators [4]: off ");

    mqtt_data_doc["variables"][4]["last"]["value"] = "";
    varsLastSend[3] = 0;
    varsLastSend[0] = 0;
  }

  else if (mqtt_data_doc["variables"][5]["last"]["value"] == "on")
  {
    digitalWrite(PIN_RELE, LOW);
    Serial.println("Process Actuators [5]: on ");

    mqtt_data_doc["variables"][5]["last"]["value"] = "";
    varsLastSend[6] = 0;
    varsLastSend[1] = 0;
  }
  else if (mqtt_data_doc["variables"][7]["last"]["value"] == "off")
  {
    digitalWrite(PIN_RELE, HIGH);
    Serial.println("Process Actuators [7]: off ");

    mqtt_data_doc["variables"][7]["last"]["value"] = "";
    varsLastSend[6] = 0;
    varsLastSend[1] = 0;
  }

  else if (mqtt_data_doc["variables"][8]["last"]["value"] > 0)
  {
    float time = mqtt_data_doc["variables"][8]["last"]["value"]; // Extraer el valor
    handleSetTime(time);                                         // Llamar a la función con el valor    // digitalWrite(PIN_RELE, HIGH);
    mqtt_data_doc["variables"][8]["last"]["value"] = "";

    Serial.println("Process Actuators [8]:  Exito ");
    serializeJson(mqtt_data_doc["variables"][8]["last"]["value"], Serial); // Imprime el JSON en la consola
    varsLastSend[9] = 0;
  }
}

// TEMPLATE ⤵
void process_incoming_msg(String topic, String incoming)
{

  last_received_topic = topic;
  last_received_msg = incoming;

  String variable = splitter.split(topic, '/', 2);

  for (int i = 0; i < mqtt_data_doc["variables"].size(); i++)
  {

    if (mqtt_data_doc["variables"][i]["variable"] == variable)
    {

      DynamicJsonDocument doc(256);
      deserializeJson(doc, incoming);
      mqtt_data_doc["variables"][i]["last"] = doc;

      long counter = mqtt_data_doc["variables"][i]["counter"];
      counter++;
      mqtt_data_doc["variables"][i]["counter"] = counter;
    }
  }

  process_actuators();
}

void callback(char *topic, byte *payload, unsigned int length)
{

  String incoming = "";

  for (int i = 0; i < length; i++)
  {
    incoming += (char)payload[i];
  }

  incoming.trim();

  process_incoming_msg(String(topic), incoming);
}

void send_data_to_broker()
{

  long now = millis();

  for (int i = 0; i < mqtt_data_doc["variables"].size(); i++)
  {

    if (mqtt_data_doc["variables"][i]["variableType"] == "output")
    {
      continue;
    }

    int freq = mqtt_data_doc["variables"][i]["variableSendFreq"];

    if (now - varsLastSend[i] > freq * 1000)
    {
      varsLastSend[i] = millis();

      String str_root_topic = mqtt_data_doc["topic"];
      String str_variable = mqtt_data_doc["variables"][i]["variable"];
      String topic = str_root_topic + str_variable + "/sdata";

      String toSend = "";

      serializeJson(mqtt_data_doc["variables"][i]["last"], toSend);

      client.publish(topic.c_str(), toSend.c_str());

      // STATS
      long counter = mqtt_data_doc["variables"][i]["counter"];
      counter++;
      mqtt_data_doc["variables"][i]["counter"] = counter;
    }
  }
}

bool reconnect()
{

  if (!get_mqtt_credentials())
  {
    Serial.println(boldRed + "\n\n      Error getting mqtt credentials :( \n\n RESTARTING IN 10 SECONDS");
    Serial.println(fontReset);
    digitalWrite(PIN_LED, HIGH);
    delay(5000); // Reducir el consumo de CPU
    delay(10000);
    ESP.restart();
  }

  // Setting up Mqtt Server
  client.setServer(mqtt_server, 1883);

  Serial.print(underlinePurple + "\n\n\nTrying MQTT Connection" + fontReset + Purple + "  ⤵");

  String str_client_id = "device_" + dId + "_" + random(1, 9999);
  const char *username = mqtt_data_doc["username"];
  const char *password = mqtt_data_doc["password"];
  String str_topic = mqtt_data_doc["topic"];

  const char *variables2 = mqtt_data_doc["variables"];

  Serial.print(boldGreen + "\n\n        Probando Username :) " + username + fontReset);
  Serial.print(boldGreen + "\n\n        Probando Pass :) " + password + fontReset);
  Serial.print(boldGreen + "\n\n        Probando topic :) " + str_topic + fontReset);
  Serial.print(boldGreen + "\n\n        Probando Variables :) " + variables2 + fontReset);

  if (client.connect(str_client_id.c_str(), username, password))
  {
    Serial.print(boldGreen + "\n\n         Mqtt Client Connected :) " + fontReset);
    digitalWrite(PIN_LED, LOW);
    delay(2000);

    client.subscribe((str_topic + "+/actdata").c_str());
    return true;
  }
  else
  {
    Serial.print(boldRed + "\n\n         Mqtt Client Connection Failed :( " + fontReset);
    return false;
  }
}

void check_mqtt_connection()
{

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(Red + "\n\n         Ups WiFi Connection Failed :( ");
    Serial.println(" -> Restarting..." + fontReset);
    delay(15000);
    ESP.restart();
  }

  if (!client.connected())
  {

    long now = millis();

    if (now - lastReconnectAttemp > 5000)
    {
      lastReconnectAttemp = millis();
      if (reconnect())
      {
        lastReconnectAttemp = 0;
      }
    }
  }
  else
  {
    client.loop();
    process_sensors();
    send_data_to_broker();
    if (print_monit)
    {
      print_stats();
    }
    digitalWrite(PIN_LED, LOW);
  }
}

bool get_mqtt_credentials()
{

  Serial.print(underlinePurple + "\n\n\nGetting MQTT Credentials from WebHook" + fontReset + Purple + "  ⤵");
  delay(1000);

  String toSend = "dId=" + dId + "&password=" + webhook_pass;

  WiFiClient client;
  HTTPClient http;
  http.begin(client, webhook_ep); // ✅ Forma correcta en ESP8266
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  int response_code = http.POST(toSend);

  if (response_code < 0)
  {
    Serial.print(boldRed + "\n\n         Error Sending Post Request :( " + fontReset);
    http.end();
    return false;
  }

  if (response_code != 200)
  {
    Serial.print(boldRed + "\n\n         Error in response :(   e-> " + fontReset + " " + response_code);
    http.end();
    return false;
  }

  if (response_code == 200)
  {
    String responseBody = http.getString();

    Serial.print(boldGreen + "\n\n         Mqtt Credentials Obtained Successfully :) " + fontReset);

    deserializeJson(mqtt_data_doc, responseBody);
    http.end();
    delay(1000);
  }

  return true;
}

void clear()
{
  Serial.write(27);    // ESC command
  Serial.print("[2J"); // clear screen command
  Serial.write(27);
  Serial.print("[H"); // cursor to home command
}

long lastStats = 0;

void print_stats()
{
  long now = millis();

  if (now - lastStats > 2000)
  {
    lastStats = millis();
    clear();

    Serial.print("\n");
    Serial.print(Purple + "\n╔══════════════════════════╗" + fontReset);
    Serial.print(Purple + "\n║       SYSTEM STATS       ║" + fontReset);
    Serial.print(Purple + "\n╚══════════════════════════╝" + fontReset);
    Serial.print("\n\n");
    Serial.print("\n\n");

    Serial.print(boldCyan + "#" + " \t Name" + " \t\t Var" + " \t\t Type" + " \t\t Count" + " \t\t Last V" + fontReset + "\n\n");

    for (int i = 0; i < mqtt_data_doc["variables"].size(); i++)
    {

      String variableFullName = mqtt_data_doc["variables"][i]["variableFullName"];
      String variable = mqtt_data_doc["variables"][i]["variable"];
      String variableType = mqtt_data_doc["variables"][i]["variableType"];
      String lastMsg = mqtt_data_doc["variables"][i]["last"];
      long counter = mqtt_data_doc["variables"][i]["counter"];

      Serial.println(String(i) + " \t " + variableFullName.substring(0, 5) + " \t\t " + variable.substring(0, 10) + " \t " + variableType.substring(0, 5) + " \t\t " + String(counter).substring(0, 10) + " \t\t " + lastMsg);
    }

    Serial.print(boldGreen + "\n\n Free RAM -> " + fontReset + ESP.getFreeHeap() + " Bytes");

    Serial.print(boldGreen + "\n\n Last Incomming Msg -> " + fontReset + last_received_msg);
  }
}

void softAPConfig()
{
  // Configurar y arrancar el AP con IP fija
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ap_ssid, ap_password);

  Serial.print("\n\n⚡🚀AP iniciado. Conéctate a: ");
  Serial.println(ap_ssid);
  Serial.print("🌐 Accede a la web en: http://");
  Serial.println(local_IP);

  digitalWrite(PIN_LED, HIGH);

  // Servir la página web principal
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html", "text/html"); });

  // Servir la página de configuracion
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/config.html", "text/html"); });

  // Servir la página de configuracion del dispositivo
  server.on("/device_config", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/device_config.html", "text/html"); });

  // Servir la página de Dashboard
  server.on("/darshboard", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/dashboard.html", "text/html"); });

  // Enviar SSID y contraseña al sitio web
  server.on("/getWifiData", HTTP_GET, [](AsyncWebServerRequest *request)
            {
  String ssid = readStringFromEEPROM(0);        // Leer SSID desde la EEPROM
  String password = readStringFromEEPROM(50);  // Leer contraseña desde la EEPROM

  String json = "{\"ssid\":\"" + ssid + "\",";
  json += "\"password\":\"" + password + "\"}";
  request->send(200, "application/json", json); });

  // Enviar Credenciales al sitio web
  server.on("/getCredentialsData", HTTP_GET, [](AsyncWebServerRequest *request)
            {
  String dId = readStringFromEEPROM(100);
  String print_stats = readStringFromEEPROM(125);
  String webhook_pass = readStringFromEEPROM(150);
  String webhook_ep = readStringFromEEPROM(200);
  String mqtt_server = readStringFromEEPROM(300);

  String json = "{\"dId\":\"" + dId + "\",";
  json += "\"webhook_pass\":\"" + webhook_pass + "\",";
  json += "\"webhook_ep\":\"" + webhook_ep + "\",";
  json += "\"mqtt_server\":\"" + mqtt_server + "\",";
  json += "\"print_stats\":\"" + print_stats + "\"}";

  
  request->send(200, "application/json", json); });

  // Guardar nuevos valores de SSID y contraseña
  server.on("/saveWifi", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    String ssid, password;
    if (request->hasParam("ssid", true) && request->hasParam("password", true)) 
    {
        ssid = request->getParam("ssid", true)->value();
        password = request->getParam("password", true)->value();

        // Guardar en EEPROM solo si los valores cambiaron
        if (ssid != readStringFromEEPROM(0)) {
            writeStringToEEPROM(0, ssid);
        }
        if (password != readStringFromEEPROM(50)) {
            writeStringToEEPROM(50, password);
        }

        Serial.println("✅ ¡Configuración WiFi guardada!");
        request->send(200, "text/plain", "¡Configuración guardada! Reinicie manualmente.");
        loadPreferences();
        printStoredData();
    }
    else 
    {
        Serial.println("❌ Datos WiFi no válidos.");
        request->send(400, "text/plain", "Datos no válidos.");
    } });

  // Guardar nuevos valores de Credenciales
  server.on("/saveCredentials", HTTP_POST, [](AsyncWebServerRequest *request)
            {
  String dId, webhook_pass, webhook_ep, mqtt_server, print_stats;

  if (request->hasParam("dId", true) && request->hasParam("webhook_pass", true) && 
      request->hasParam("webhook_ep", true) && request->hasParam("mqtt_server", true)) 
  {
      dId = request->getParam("dId", true)->value();
      webhook_pass = request->getParam("webhook_pass", true)->value();
      webhook_ep = request->getParam("webhook_ep", true)->value();
      mqtt_server = request->getParam("mqtt_server", true)->value();
      print_stats = request->getParam("print_stats", true)->value();

      // Guardar valores nuevos en EEPROM
      EEPROM.begin(EEPROM_SIZE);
      writeStringToEEPROM(100, dId);
      writeStringToEEPROM(125, print_stats);
      writeStringToEEPROM(150, webhook_pass);
      writeStringToEEPROM(200, webhook_ep);
      writeStringToEEPROM(300, mqtt_server);
      EEPROM.commit();

      Serial.println("✅ ¡Configuración de credenciales guardada!");
      request->send(200, "text/plain", "¡Configuración guardada! Reinicie manualmente.");
      loadPreferences();
      printStoredData();
  } 
  else 
  {
      Serial.println("❌ Datos de Credenciales no válidos.");
      request->send(400, "text/plain", "Datos de Credenciales no válidos.");
  } });

  // Ruta para reiniciar manualmente el ESP32
  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request)
            {
        ESP.restart();
        request->send(200, "text/plain", "Reiniciando Dispositivo no se podrá conectar por un momento..."); });

  // Escanear redes WiFi
  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request)
            {
        scanNetworks();  // Realiza el escaneo
        request->send(200, "text/html", networksList); });

  // Iniciar el servidor
  server.begin();
  Serial.println("✅ Servidor web iniciado.");

  // Activar el modo AP y detener otras funciones
  apModeActive = true; // Cambiar el estado de la variable para indicar que estamos en AP

  // ❗ Mantener el ESP32 en modo AP ❗
  while (true)
  {
    digitalWrite(PIN_LED, LOW);
    delay(2000); // Reducir el consumo de CPU
    digitalWrite(PIN_LED, HIGH);
    delay(2000); // Reducir el consumo de CPU
  }
}

// Función para escanear redes WiFi
void scanNetworks()
{
  if (millis() - lastScanTime > 5000 && !scanning)
  {
    scanning = true;
    Serial.println("Iniciando escaneo de redes...");

    WiFi.scanNetworksAsync([](int n)
                           {
                             networksList = ""; // Reiniciamos la lista de redes
                             Serial.println("Escaneo terminado.");

                             if (n == 0)
                             {
                               networksList = "<p>No se encontraron redes disponibles.</p>";
                             }
                             else
                             {
                               for (int i = 0; i < n; ++i)
                               {
                                 networksList += "<button class=' btn btn-success ' onclick=\"selectSSID('" + WiFi.SSID(i) + "')\">" +
                                                 WiFi.SSID(i) + " (Señal: " + String(WiFi.RSSI(i)) + ")</button><br>";
                               }
                             }
                             lastScanTime = millis();
                             scanning = false; // Terminamos el escaneo
                           });
  }
}

// Encender el servo
void handleServoOn()
{
  isServoOn = true;
  isTimedMode = false; // Desactivar el modo de tiempo
  servo.write(180);    // Mover el servo a 180 grados (abierto)
  // server.send(200, "text/plain", "Servo encendido");
}

// Apagar el servo
void handleServoOff()
{
  isServoOn = false;
  isTimedMode = false; // Desactivar el modo de tiempo
  servo.write(90);     // Volver a la posición de descanso (cerrado)
  // server.send(200, "text/plain", "Servo apagado");
}

// Establecer el tiempo en minutos
void handleSetTime(float time)
{
  if (time > 0)
  {
    targetTime = time;    // Establecer el tiempo en minutos
    isServoOn = true;     // Encender el servo
    isTimedMode = true;   // Activar el modo de tiempo
    startTime = millis(); // Registrar el tiempo de inicio
    servo.write(180);     // Mover el servo a 180 grados (abierto)
    Serial.println("Tiempo establecido: " + String(targetTime) + " minutos");
  }
  else
  {
    Serial.println("Error: El tiempo debe ser mayor que 0");
  }
}