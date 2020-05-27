#define USE_WIFI 1
#define USE_BME280 1
#define USE_CAYENNE 0
#define USE_MQTT 0

#define HAS_PMU 0
#define HAS_GPS 0

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "globals.h"


Helios helios;


double dAzimuth;
double dElevation;



Servo servoMain; // Define our Servo


void servo_sweep()
{

servoMain.write(0); // Turn Servo Left to 45 degrees
delay(2000); // Wait 1 second

servoMain.write(180); // Turn Servo Left to 45 degrees
delay(2000); // Wait 1 second


servoMain.write(45); // Turn Servo Left to 45 degrees
delay(1000); // Wait 1 second
servoMain.write(0); // Turn Servo Left to 0 degrees
delay(1000); // Wait 1 second
servoMain.write(90); // Turn Servo back to center position (90 degrees)
delay(1000); // Wait 1 second
servoMain.write(135); // Turn Servo Right to 135 degrees
delay(1000); // Wait 1 second
servoMain.write(180); // Turn Servo Right to 180 degrees
delay(1000); // Wait 1 second
servoMain.write(90); // Turn Servo back to center position (90 degrees)
delay(1000); // Wait 1 second
}




//--------------------------------------------------------------------------
// Store preferences in NVS Flash
//--------------------------------------------------------------------------
Preferences preferences;
char lastword[10];

unsigned long uptime_seconds_old;
unsigned long uptime_seconds_new;
unsigned long uptime_seconds_actual;

//--------------------------------------------------------------------------
// Tasks
//--------------------------------------------------------------------------
Ticker sleepTicker;
Ticker displayTicker;

//--------------------------------------------------------------------------
// Sensors
//--------------------------------------------------------------------------
Adafruit_BME280 bme; // I2C   PIN 21 + 22

Button* b = NULL;

//--------------------------------------------------------------------------
// ESP Sleep Mode
//--------------------------------------------------------------------------
#define ESP_SLEEP 0            // Main switch
const float sleepPeriod = 2;   //seconds
#define uS_TO_S_FACTOR 1000000 //* Conversion factor for micro seconds to seconds */

#define display_refresh 1      // every second
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define SEALEVELPRESSURE_HPA (1013.25)

int runmode = 0;
String stringOne = "";
static const char TAG[] = __FILE__;
char s[32]; // used to sprintf for Serial output
uint8_t txBuffer[9];

//--------------------------------------------------------------------------
// Wifi Settings
//--------------------------------------------------------------------------
const char ssid[] = "MrFlexi";
const char wifiPassword[] = "Linde-123";
WiFiClient wifiClient;

void print_ina()
{
  Serial.println("------------------------------");
  float shuntvoltage1 = 0;
  float busvoltage1 = 0;
  float current_mA1 = 0;
  float loadvoltage1 = 0;

  busvoltage1 = ina3221.getBusVoltage_V(1);
  shuntvoltage1 = ina3221.getShuntVoltage_mV(1);
  current_mA1 = -ina3221.getCurrent_mA(1); // minus is to get the "sense" right.   - means the battery is charging, + that it is discharging
  loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000);

  Serial.print("LIPO_Battery Bus Voltage:   ");
  Serial.print(busvoltage1);
  Serial.println(" V");
  Serial.print("LIPO_Battery Shunt Voltage: ");
  Serial.print(shuntvoltage1);
  Serial.println(" mV");
  Serial.print("LIPO_Battery Load Voltage:  ");
  Serial.print(loadvoltage1);
  Serial.println(" V");
  Serial.print("LIPO_Battery Current 1:       ");
  Serial.print(current_mA1);
  Serial.println(" mA");
  Serial.println("");
}

//--------------------------------------------------------------------------
// MQTT
//--------------------------------------------------------------------------
#if (USE_MQTT)
#include <PubSubClient.h>
#endif
//const char *mqtt_server = "192.168.1.144"; // Laptop
//const char *mqtt_server = "test.mosquitto.org"; // Laptop
const char *mqtt_server = "192.168.1.100"; // Raspberry
const char *mqtt_topic = "mrflexi/solarserver/";

#if (USE_MQTT)
PubSubClient client(wifiClient);
long lastMsgAlive = 0;
long lastMsgDist = 0;
#endif

#if (USE_MQTT)
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");

  u8g2log.print(topic);
  u8g2log.print("\n");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    u8g2log.print((char)payload[i]);
  }
  Serial.println();
  u8g2log.print("\n");

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1')
  {
    digitalWrite(LED_BUILTIN, LOW); // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH); // Turn the LED off by making the voltage HIGH
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("Mqtt Client"))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("MrFlexi/nodemcu", "connected");
      // ... and resubscribe
      client.subscribe(mqtt_topic);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup_mqtt()
{
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  if (!client.connected())
  {
    reconnect();
  }
  log_display("Mqtt connected");
  client.publish("mrflexi/solarserver/info", "ESP32 is alive...");
}
#endif





void save_uptime()
{
  uptime_seconds_new = uptime_seconds_old + uptime_seconds_actual;
  preferences.putULong("uptime", uptime_seconds_new);
  Serial.println("ESP32 total uptime" + String(uptime_seconds_new) + " Seconds");
}

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

void setup_sensors()
{

#if (USE_BME280)
  ESP_LOGI(TAG, "BME280 Setup...");
  unsigned status;

  status = bme.begin(0x76);
  if (!status)
  {
    ESP_LOGI(TAG, "Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  }
  else
  {
    Serial.println();
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");
    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");
    Serial.println();
  }
#endif
}

void t_sleep()
{
  dataBuffer.data.sleepCounter--;

  //-----------------------------------------------------
  // Deep sleep
  //-----------------------------------------------------
#if (ESP_SLEEP)
  if (dataBuffer.data.sleepCounter <= 0 || dataBuffer.data.txCounter >= SLEEP_AFTER_N_TX_COUNT)
  {
    runmode = 0;
    gps.enable_sleep();
    Serial.flush();
    showPage(PAGE_SLEEP);
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  }
#endif
}

void t_display()
{
  static char volbuffer[20];

  String stringOne;

// Temperatur
#if (USE_BME280)
  snprintf(volbuffer, sizeof(volbuffer), "%.1fC/%.1f%", bme.readTemperature(), bme.readHumidity());
  log_display(volbuffer);
#endif

// Voltage and Current
  dataBuffer.data.busvoltage1 = ina3221.getBusVoltage_V(1);
  dataBuffer.data.current_1   = ina3221.getCurrent_mA(1);
  dataBuffer.data.current_2   = ina3221.getCurrent_mA(2);
  dataBuffer.data.current_3   = ina3221.getCurrent_mA(3);

// GPS
  gps.encode();

// Update Display
  showPage(PAGE_VALUES);
}

void setup_wifi()
{

#if (USE_WIFI)
  // WIFI Setup
  WiFi.begin(ssid, wifiPassword);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    ESP_LOGI(TAG, "Connecting to WiFi..");
  }
  //ESP_LOGI(TAG, String(WiFi.localIP()) );
#endif
}


void show(char nameStr[], double val, boolean newline) {
  Serial.print(nameStr);  
  Serial.print("=");
  if (newline)
       Serial.println(val);
  else Serial.print(val);
}

void setup()
{
  Serial.begin(115200);
  delay(10);
  Serial.println(" ");
  Serial.printf("size of double: %d\n", sizeof(double));
  Serial.printf("size of float: %d\n", sizeof(float));
    Serial.println(" ");

  print_wakeup_reason();

  //---------------------------------------------------------------
  // Get preferences from Flash
  //---------------------------------------------------------------
  preferences.begin("config", false); // NVS Flash RW mode
  preferences.getULong("uptime", uptime_seconds_old);
  Serial.println("Uptime old: " + String(uptime_seconds_old));
  preferences.getString("info", lastword, sizeof(lastword));

  ESP_LOGI(TAG, "Starting..");
  Serial.println(F("ESP32 Sensor Board"));
  i2c_scan();

  ina3221.begin();
  Serial.print("Manufactures ID=0x");
  int MID;
  MID = ina3221.getManufID();
  Serial.println(MID, HEX);

  dataBuffer.data.txCounter = 0;
  dataBuffer.data.sleepCounter = TIME_TO_NEXT_SLEEP;

  setup_display();
  setup_sensors();
  setup_wifi();
  
  //gps.init();
  //gps.wakeup();
  //gps.ecoMode();

#if (USE_MQTT)
  setup_mqtt();
#endif

  // Tasks
  sleepTicker.attach(60, t_sleep);
  displayTicker.attach(display_refresh, t_display);

  runmode = 1; // Switch from Terminal Mode to page Display
  showPage(1);

  //---------------------------------------------------------------
  // Deep sleep settings
  //---------------------------------------------------------------
  #if (ESP_SLEEP)
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR * 60);
  Serial.println("Setup ESP32 to wake-up via timer after " + String(TIME_TO_SLEEP) +
                 " Minutes");
  #endif

// Calc Sun Position München
Serial.println("");
Serial.println("Sun Azimuth and Elevation Munich");
helios.calcSunPos(2019,10,18,11.00,00.00,00.00,11.54184,48.15496); 

dAzimuth=helios.dAzimuth;show("dAzimuth",dAzimuth,true);
dElevation=helios.dElevation;show("dElevation",dElevation,true);
dataBuffer.data.sun_azimuth = helios.dAzimuth;
dataBuffer.data.sun_elevation = helios.dElevation;


b = new Button(39);  // Boot Button

    b->setOnDoubleClicked([]() {
        Serial.println("doubleclicked");
    });

    b->setOnClicked([]() {
        Serial.println("clicked");
    });

    b->setOnHolding([]() {
        Serial.println("holding");
    });



servoMain.attach(17); // servo on digital pin 10

servo_sweep();

}

void loop()
{
#if (USE_MQTT)
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
#endif

 b->update();
}