#include <ESP8266WiFi.h>
#include <ESP8266TimerInterrupt.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

// Pins
#define WIND_SPEED_PIN 14
#define MUX_ADDR_A_PIN 5
#define MUX_ADDR_B_PIN 4

// Sample Rates
#define SAMPLE_PERIOD_SEC 5
#define SAMPLE_COUNT 24 // This is the averaging time of the sensor - Two minutes -  24 * 5 second samples

// Interrupte timer precision
#define USING_TIME_DIV1 true

#define VERSION 0.2

ESP8266Timer ITimer;
WiFiClient espClient;
PubSubClient client(espClient);

const char* ssid     = "bitfu2";
const char* password = "J&TTroyer";

// MQTT Broker
const char *mqtt_broker = "192.168.1.10";
const char *topic = "weather/test";
const int mqtt_port = 1883;

// Voltages
unsigned int batteryVoltage;
unsigned int solarVoltage;

// Wind
uint16_t windDirection;
float speed_average;
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine
volatile float speed_samples[SAMPLE_COUNT];
volatile uint16_t direction_samples[SAMPLE_COUNT];
volatile byte sample_index;

// Data
volatile bool upload_data_flag;
volatile bool ota_flag;

void setup()
{
  Serial.begin(74880);
  Serial.println("Setup Begin...");

  // start in sleep
  wifi_sleep();

  // Setup MUX pin modes
  pinMode(MUX_ADDR_A_PIN, OUTPUT);
  pinMode(MUX_ADDR_B_PIN, OUTPUT);

  // Setup wind speed pin mode
  pinMode(WIND_SPEED_PIN, INPUT);

  // disable interrupts
  cli();

  // wind speed interrupt increments counter on every anemometer rotation
  attachInterrupt(digitalPinToInterrupt(WIND_SPEED_PIN), isr_rotation, FALLING);

  // Sample period interrupt.  This drives the timing of the sensor, ensures accurate sampling intervals
  ITimer.attachInterruptInterval(SAMPLE_PERIOD_SEC * 1000 * 1000, isr_speed_sample_ready);

  // enable interrupts
  sei();

  ArduinoOTA.onStart([]() {
    ITimer.detachInterrupt();
    ota_flag = true;

    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();

  Serial.println("Setup Complete...");
}

void loop()
{
  ArduinoOTA.handle();

  if (upload_data_flag)
  {
    calculate_readings();
    upload_data();
  }
}

void upload_data()
{
  Serial.println("Uploading Data...");

  connect_wifi();
  connect_mqtt();

  // Wind Speed
  char speed_cstr[16];
  sprintf(speed_cstr, "%f", speed_average);
  client.publish("weather/wind_speed", speed_cstr);

  // Wind Direction
  char dir_cstr[16];
  sprintf(dir_cstr, "%d", direction_samples[sample_index]);
  client.publish("weather/wind_direction", dir_cstr);

  // Batt Voltage
  getBatteryVoltage();
  char bat_cstr[3];
  sprintf(bat_cstr, "%d", batteryVoltage);
  client.publish("weather/battery_voltage", bat_cstr);

  // Solar Voltage
  getSolarVoltage();
  char solar_cstr[16];
  sprintf(solar_cstr, "%d", solarVoltage);
  client.publish("weather/solar_voltage", solar_cstr);

  client.publish("weather/version", String(VERSION).c_str());

  upload_data_flag = false;
  disconnect_mqtt();

  //wifi_sleep();
  Serial.println("Uploading Data Complete!");
}

void calculate_readings()
{
  // Calculate the average speed
  float speed_sum = 0;
  for (unsigned int i = 0; i < SAMPLE_COUNT; ++i)
  {
    speed_sum += speed_samples[i];
  }
  speed_average = speed_sum / SAMPLE_COUNT;
}

void connect_wifi()
{
  Serial.println("Connecting wifi");

  WiFi.forceSleepWake();
  delay(1);

  WiFi.begin("bitfu2", "J&TTroyer");

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
}

void connect_mqtt()
{
  Serial.println("Connecting to Mqtt: ");
  client.setServer(mqtt_broker, mqtt_port);
  while (!client.connected())
  {
    String client_id = "weather-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
    if (!client.connect(client_id.c_str(), "", ""))
    {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void disconnect_mqtt()
{
  client.disconnect();
}

void wifi_sleep()
{
  if (!ota_flag)
  {
    Serial.println("Disconnecting wifi");
    WiFi.disconnect();
    WiFi.forceSleepBegin();
    delay(1); //For some reason the modem won't go to sleep unless you do a delay
  }
}

void getWindSpeed()
{
  // Wind Speed Translation Formula
  // 1600 rev/hr = 1 mph
  // V = P(2.25/T) (V = speed in mph, P = no. of pulses per sample period T = sample period in seconds)
  // Knots = mph * 0.868976
  speed_samples[sample_index] = Rotations * 2.25 / SAMPLE_PERIOD_SEC * 0.868976;

  //  Serial.print("speed ");
  //  Serial.print(Rotations);
  //  Serial.print(" ");
  //  Serial.print(sample_index);
  //  Serial.print(" ");
  //  Serial.println(speed_samples[sample_index]);
  Rotations = 0;
}

void getWindDirection()
{
  setMuxAddress(2);
  int rawValue = analogRead(A0);

  // Direction is a 20k pot fed with vcc and voltage divided to provide 0 - 1v
  // Voltage divider set to 1v = 3.3v
  direction_samples[sample_index] = map(rawValue, 0, 1024, 0, 359);

  //  Serial.print("direction ");
  //  Serial.print(rawValue);
  //  Serial.print(" ");
  //  Serial.println(direction_samples[sample_index]);
}

void getBatteryVoltage()
{
  setMuxAddress(0);
  int rawValue = analogRead(A0);

  // Voltage divider set to 1v = 4.2v
  batteryVoltage = map(rawValue, 0, 1024, 0, 420);
}

void getSolarVoltage()
{
  setMuxAddress(1);
  int rawValue = analogRead(A0);

  // Voltage divider set to 1v = 6v
  solarVoltage = map(rawValue, 0, 1024, 0, 600);
}

void setMuxAddress(byte addr)
{
  digitalWrite(MUX_ADDR_A_PIN, addr & 0x01);
  digitalWrite(MUX_ADDR_B_PIN, (addr >> 1) & 0x01);
}

// This is the function that the interrupt calls to increment the rotation count
ICACHE_RAM_ATTR 
void isr_rotation()
{
  Rotations++;
}

ICACHE_RAM_ATTR 
void isr_speed_sample_ready()
{
  getWindSpeed();
  getWindDirection();

  sample_index++;
  if (sample_index >= SAMPLE_COUNT)
  {
    sample_index = 0;
    upload_data_flag = true;
  }

  Serial.println("");
  Serial.println("#");
}
