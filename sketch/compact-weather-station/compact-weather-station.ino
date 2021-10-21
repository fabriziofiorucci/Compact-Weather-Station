/*
Compact Weather station

Wemos D1:
A0 = Capacitive soil moisture sensor v1.2
D1 = SCL
D2 = SDA
D3 = Rain tipping bucket (tipping bucket wires connected to D3 and GND, D3 set as INPUT_PULLUP)
D6 = to PMS5003 RX (pin 4)
D7 = to PMS5003 TX (pin 5)

I2C ids:
0x23 - GY-30 BH1750
0x48 - ADS1115
0x76 - BME280

ADS1115 gain: 2x, input range=+/-2.048v
ADS1115 addr pin = GND (0x48)
A0 - Free
A1 - Free
A2 - Free
A3 - Anemometer range 0.4-2v, vcc=9v connected to ADC through R=1.8k, max current A0=5mA - 0.4V (0 m/s wind) up to 2.0V (for 32.4m/s wind speed)

=> BME280
VCC - Brown
GND - Red
SCL - White
SDA - Blue

=> GY-30 BH1750
VCC - 3.3v
ADD->GND (I2C 0x23)

=> PMS5003
pin 4 (RX) - to Wemos D6
pin 5 (TX) - to Wemod D7 

*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1X15.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <BH1750FVI.h>
#include <SoftwareSerial.h>
#include "PMS.h"
#include <ArduinoOTA.h>

// Particulate matter sensor
#define   PMS_STATE_ASLEEP        0   // Low power mode, laser and fan off
#define   PMS_STATE_WAKING_UP     1   // Laser and fan on, not ready yet
#define   PMS_STATE_READY         2   // Warmed up, ready to give data
uint8_t   g_pms_state           = PMS_STATE_WAKING_UP;
uint32_t  g_pms_state_start     = 0;  // Timestamp when PMS state last changed
uint8_t   g_pms_ae_readings_taken  = false;  // true/false: whether any readings have been taken
uint8_t   g_pms_ppd_readings_taken = false;  // true/false: whether PPD readings have been taken

uint16_t  g_pm1p0_sp_value      = 0;  // Standard Particle calibration pm1.0 reading
uint16_t  g_pm2p5_sp_value      = 0;  // Standard Particle calibration pm2.5 reading
uint16_t  g_pm10p0_sp_value     = 0;  // Standard Particle calibration pm10.0 reading

uint16_t  g_pm1p0_ae_value      = 0;  // Atmospheric Environment pm1.0 reading
uint16_t  g_pm2p5_ae_value      = 0;  // Atmospheric Environment pm2.5 reading
uint16_t  g_pm10p0_ae_value     = 0;  // Atmospheric Environment pm10.0 reading

uint32_t  g_pm0p3_ppd_value     = 0;  // Particles Per Deciliter pm0.3 reading
uint32_t  g_pm0p5_ppd_value     = 0;  // Particles Per Deciliter pm0.5 reading
uint32_t  g_pm1p0_ppd_value     = 0;  // Particles Per Deciliter pm1.0 reading
uint32_t  g_pm2p5_ppd_value     = 0;  // Particles Per Deciliter pm2.5 readingpm25
uint32_t  g_pm5p0_ppd_value     = 0;  // Particles Per Deciliter pm5.0 reading
uint32_t  g_pm10p0_ppd_value    = 0;  // Particles Per Deciliter pm10.0 reading

uint32_t    g_pms_warmup_period   =  30;             // Seconds to warm up PMS before reading
uint32_t    g_pms_report_period   = 120;             // Seconds between reports

const char* topicPMS5003aepm1p0="weatherstation/pms5003/ae/pm1p0";
const char* topicPMS5003aepm2p5="weatherstation/pms5003/ae/pm2p5";
const char* topicPMS5003aepm10p0="weatherstation/pms5003/ae/pm10p0";
const char* topicPMS5003ppd0p3="weatherstation/pms5003/ppd/0p3";
const char* topicPMS5003ppd0p5="weatherstation/pms5003/ppd/0p5";
const char* topicPMS5003ppd1p0="weatherstation/pms5003/ppd/1p0";
const char* topicPMS5003ppd2p5="weatherstation/pms5003/ppd/2p5";
const char* topicPMS5003ppd5p0="weatherstation/pms5003/ppd/5p0";
const char* topicPMS5003ppd10p0="weatherstation/pms5003/ppd/10p0";

char s[256];

const char* controlTopic="weatherstation/control";

// PMS5003
SoftwareSerial pmsSerial(D6, D7); // RX pin, TX pin
PMS pms(pmsSerial);                      // Use the software serial port for the PMS
PMS::DATA g_data;

#define SEALEVELPRESSURE_HPA (1013.25)

// --------------- CUSTOMIZE HERE MQTT PARAMETERS
const char* mqttServer = "mqtt-server-ip";
const char* mqttClientId="weatherstation";
const char* mqttClientPassword="mymqttpassword";

// --------------- CUSTOMIZE HERE WIFI PARAMETERS
const char* ssid = "MySSID";
const char* password = "MyWifiPassword";
IPAddress ip(192, 168, 1, 199);
IPAddress gateway(192, 168, 1, 5);
IPAddress subnet(255, 255, 255, 0);

Adafruit_BME280 bme;

// https://community.blynk.cc/t/ads1115-with-wemos-mini-d1/12215
Adafruit_ADS1115 ads;
float Voltage0 = 0.0;
float Voltage1 = 0.0;
float Voltage2 = 0.0;
float Voltage3 = 0.0;

float windSpeed = 0.0;
int capacitiveRain = 0;

// GY-30 - BH1750
BH1750FVI luxSensor(0x23);
float luxCorrectionFactor=1.2;

// NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "it.pool.ntp.org", 3600, 60000);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Tipping bucket rain sensor
int TIPPING_BUCKET_PIN=D3;
long tippingBucketPulseCount=0;
unsigned long msIRQdebouncingTime = 1;
volatile unsigned long lastIRQmicros;


float readValue;
long lastMsg = 0;

long pollInterval=5000;



void setup_wifi() {
  delay(10);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);

  //while (WiFi.status() != WL_CONNECTED) {
  for (int i = 0; i < 500 && WiFi.status() != WL_CONNECTED; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(25);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(25);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Restarting ESP due to loss of Wi-Fi");
    ESP.restart();
  }

  randomSeed(micros());

  Serial.println("");
  Serial.print("WiFi connected to ");
  Serial.println(ssid);
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Got MQTT [");
  Serial.print(topic);
  Serial.print("] = [");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("]");

  if (strcmp(topic,controlTopic)==0) {
    if ((char)payload[0]=='R') {
      Serial.println("Received MQTT reset, restarting ESP");
      ESP.restart();
    }
  }
}

void ICACHE_RAM_ATTR onTippingBucketPulse()
{
  if((long)(micros() - lastIRQmicros) >= msIRQdebouncingTime * 1000) {
    tippingBucketPulseCount++;
    lastIRQmicros = micros();
  } 
}

void setup() {
  Serial.begin(115200, SERIAL_8N1);
  Serial.println("-----------------------------------------");
  Serial.printf("Chip ID     : %08X\n", ESP.getChipId());
  Serial.printf("Core version: %s\n", ESP.getCoreVersion().c_str());
  Serial.printf("SDK version : %s\n", ESP.getSdkVersion());
  Serial.printf("CPU speed   : %dMhz\n", ESP.getCpuFreqMHz());
  Serial.printf("Free Heap   : %d\n", ESP.getFreeHeap());
  Serial.printf("Reset reason: %s\n", ESP.getResetReason().c_str());
  Serial.println("-----------------------------------------");

  setup_wifi();
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output

  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(callback);

  bool status;
    
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("BME280 not found");
    //ESP.restart();
  }

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);           // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  
  ads.begin(0x48);

  luxSensor.powerOn();
  luxSensor.setContHighRes();
  luxSensor.setCorrectionFactor(luxCorrectionFactor);

  // PMS5003
  pmsSerial.begin(9600);   // Connection for PMS5003
  pms.passiveMode();                // Tell PMS to stop sending data automatically
  delay(100);
  Serial.println("PMS5003: wakeup");
  pms.wakeUp();                     // Tell PMS to wake up (turn on fan and laser)

  // Rain tipping bucket
  pinMode(TIPPING_BUCKET_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TIPPING_BUCKET_PIN), onTippingBucketPulse, FALLING);
  
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("weatherstation");
  ArduinoOTA.setPassword((const char *)"OTApassword");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

void reconnectMqtt() {
  while (!mqttClient.connected()) {
    String clientId = "weatherstation";

    Serial.print("Attempting MQTT connection...");
    
    if (mqttClient.connect(clientId.c_str(),mqttClientId,mqttClientPassword)) {
      Serial.println("connected to MQTT");
      mqttClient.publish("weatherstation/debug", "connected to MQTT");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void pms5003Read()
{
  uint32_t time_now = millis();

  // Check if we've been in the sleep state for long enough
  if (PMS_STATE_ASLEEP == g_pms_state)
  {
    if (time_now - g_pms_state_start
        >= ((g_pms_report_period * 1000) - (g_pms_warmup_period * 1000)))
    {
      // It's time to wake up the sensor
      Serial.println("PMS5003: wakeup");
      pms.wakeUp();
      g_pms_state_start = time_now;
      g_pms_state = PMS_STATE_WAKING_UP;
    }
  }

  // Check if we've been in the waking up state for long enough
  if (PMS_STATE_WAKING_UP == g_pms_state)
  {
    //Serial.println("Checking if wakeup state is long enough");
    if (time_now - g_pms_state_start
        >= (g_pms_warmup_period * 1000))
    {
      g_pms_state_start = time_now;
      g_pms_state = PMS_STATE_READY;
    }
  }

  // Put the most recent values into globals for reference elsewhere
  if (PMS_STATE_READY == g_pms_state)
  {
    pms.requestRead();
    Serial.println("PMS5003: ready");
    if (pms.readUntil(g_data))  // Use a blocking road to make sure we get values
    {
      g_pm1p0_sp_value   = g_data.PM_SP_UG_1_0;
      g_pm2p5_sp_value   = g_data.PM_SP_UG_2_5;
      g_pm10p0_sp_value  = g_data.PM_SP_UG_10_0;

      g_pm1p0_ae_value   = g_data.PM_AE_UG_1_0;
      g_pm2p5_ae_value   = g_data.PM_AE_UG_2_5;
      g_pm10p0_ae_value  = g_data.PM_AE_UG_10_0;

      sprintf(s,"PMS5003: sp+ae [%u %u %u %u %u %u]",g_pm1p0_sp_value,g_pm2p5_sp_value,g_pm10p0_sp_value,g_pm1p0_ae_value,g_pm2p5_ae_value,g_pm10p0_ae_value);
      Serial.println(s);

      g_pms_ae_readings_taken = true;

      // This condition below should NOT be required, but currently I get all
      // 0 values for the PPD results every second time. This check only updates
      // the global values if there is a non-zero result for any of the values:
      if (g_data.PM_TOTALPARTICLES_0_3 + g_data.PM_TOTALPARTICLES_0_5
          + g_data.PM_TOTALPARTICLES_1_0 + g_data.PM_TOTALPARTICLES_2_5
          + g_data.PM_TOTALPARTICLES_5_0 + g_data.PM_TOTALPARTICLES_10_0
          != 0)
      {
        g_pm0p3_ppd_value  = g_data.PM_TOTALPARTICLES_0_3;
        g_pm0p5_ppd_value  = g_data.PM_TOTALPARTICLES_0_5;
        g_pm1p0_ppd_value  = g_data.PM_TOTALPARTICLES_1_0;
        g_pm2p5_ppd_value  = g_data.PM_TOTALPARTICLES_2_5;
        g_pm5p0_ppd_value  = g_data.PM_TOTALPARTICLES_5_0;
        g_pm10p0_ppd_value = g_data.PM_TOTALPARTICLES_10_0;
        g_pms_ppd_readings_taken = true;

        sprintf(s,"PMS5003: ppd [%u %u %u %u %u %u]",g_pm0p3_ppd_value,g_pm0p5_ppd_value,g_pm1p0_ppd_value,g_pm2p5_ppd_value,g_pm5p0_ppd_value,g_pm10p0_ppd_value);
        Serial.println(s);
      } else {
        Serial.println("PMS5003: ppd readings 0");
      }

      Serial.println("PMS5003: sleep");
      pms.sleep();

      g_pms_state_start = time_now;
      g_pms_state = PMS_STATE_ASLEEP;
    }
  }
}

void reportPMS5003ToMqtt()
{
  if (true == g_pms_ae_readings_taken)
  {
    /* Report PM1.0 AE value */
    mqttClient.publish(topicPMS5003aepm1p0, (char *) String(g_pm1p0_ae_value).c_str());

    /* Report PM2.5 AE value */
    mqttClient.publish(topicPMS5003aepm2p5, (char *) String(g_pm2p5_ae_value).c_str());

    /* Report PM10.0 AE value */
    mqttClient.publish(topicPMS5003aepm10p0, (char *) String(g_pm10p0_ae_value).c_str());

    g_pms_ae_readings_taken=false;
  }

  if (true == g_pms_ppd_readings_taken)
  {
    /* Report PM0.3 PPD value */
    mqttClient.publish(topicPMS5003ppd0p3,(char *) String(g_pm0p3_ppd_value).c_str());

    /* Report PM0.5 PPD value */
    mqttClient.publish(topicPMS5003ppd0p5,(char *) String(g_pm0p5_ppd_value).c_str());

    /* Report PM1.0 PPD value */
    mqttClient.publish(topicPMS5003ppd1p0,(char *) String(g_pm1p0_ppd_value).c_str());

    /* Report PM2.5 PPD value */
    mqttClient.publish(topicPMS5003ppd2p5,(char *) String(g_pm2p5_ppd_value).c_str());

    /* Report PM5.0 PPD value */
    mqttClient.publish(topicPMS5003ppd5p0,(char *) String(g_pm5p0_ppd_value).c_str());

    /* Report PM10.0 PPD value */
    mqttClient.publish(topicPMS5003ppd10p0,(char *) String(g_pm10p0_ppd_value).c_str());
    
    g_pms_ppd_readings_taken=false;
  }
}

void readADS1115()
{
  int16_t adc0, adc1, adc2, adc3;

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

  Serial.print("ADS1115 A0 : ");
  Serial.println(adc0);
  Serial.print("ADS1115 A1 : ");
  Serial.println(adc1);
  Serial.print("ADS1115 A2 : ");
  Serial.println(adc2);
  Serial.print("ADS1115 A3 : ");
  Serial.println(adc3);
  
  Voltage0 = (adc0 * 0.0625)/1000.0;
  Voltage1 = (adc1 * 0.0625)/1000.0; 
  Voltage2 = (adc2 * 0.0625)/1000.0;
  Voltage3 = (adc3 * 0.0625)/1000.0;
}

void loop() { 
  ArduinoOTA.handle();

  long now = millis();
  if (now - lastMsg > pollInterval) {
    lastMsg = now;
  
  if (!mqttClient.connected()) {
    reconnectMqtt();
  }

  mqttClient.loop();
  pms5003Read();

  Serial.println("-----------");

  digitalWrite(LED_BUILTIN, LOW);

  readValue=bme.readTemperature();
  mqttClient.publish("weatherstation/temperature", (char *) String(readValue).c_str());
  Serial.print("Temperature:   ");
  Serial.print(readValue, 2);
  Serial.println(" °C");

  readValue=bme.readPressure() / 100.0F;
  mqttClient.publish("weatherstation/pressure", (char *) String(readValue).c_str());
  Serial.print("Pressure   :   ");
  Serial.print(readValue, 2);
  Serial.println(" hPa");

  readValue=bme.readAltitude(SEALEVELPRESSURE_HPA);
  mqttClient.publish("weatherstation/altitude", (char *) String(readValue).c_str());
  Serial.print("Altitude   :   ");
  Serial.print(readValue, 2);
  Serial.println(" m");

  readValue=bme.readHumidity();
  mqttClient.publish("weatherstation/humidity", (char *) String(readValue).c_str());
  Serial.print("Humidity   :   ");
  Serial.print(readValue, 2);
  Serial.println(" %");

  readADS1115();

  Serial.print("A0 (free)  : ");
  Serial.println((float)(Voltage0),2);
  
  Serial.print("A1  (free)  : ");
  Serial.println((float)(Voltage1),2);

  Serial.print("A2 (free)  : ");
  Serial.println((float)(Voltage2),2);
  
  Serial.print("A3 (wind)  : ");
  Serial.print((float)(Voltage3),2);
  Serial.print("  Speed: ");
  windSpeed=(Voltage3-0.39)*32.4/1.6;
  Serial.println((float)(windSpeed),2);
  mqttClient.publish("weatherstation/wind/speed", (char *) String(windSpeed).c_str());

  if(luxSensor.isReady()) {
    float luxLight=luxSensor.getLux();

    Serial.print("Light      : ");
    Serial.print(luxLight,2);
    Serial.println(" Lx");
    mqttClient.publish("weatherstation/light", (char *)String(luxLight).c_str());
  }

  capacitiveRain=analogRead(A0);
  Serial.print("Capacitive rain: ");
  Serial.println(capacitiveRain);
  mqttClient.publish("weatherstation/rain/capacitive", (char *)String(capacitiveRain).c_str());

  // Rain tipping bucket
  long tippingBucketReading=tippingBucketPulseCount;
  tippingBucketPulseCount=0;
  Serial.print("Rain tipping   :");
  Serial.println(tippingBucketReading);
  mqttClient.publish("weatherstation/rain/tipping", (char *)String(tippingBucketReading).c_str());

  // PMS5003
  reportPMS5003ToMqtt();
  
  digitalWrite(LED_BUILTIN, HIGH);
  }
}
