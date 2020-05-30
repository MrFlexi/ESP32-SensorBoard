
//------------------------------------------------------
//               PINOUT
//        GPIO        Function
//        21          SDA
//        22          SCL
//        32          Lichtschranke   Richtung
//        33          Lichtschranke   Puls
//        34          Servo

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "globals.h"

//--------------------------------------------------------------------------
// Sun Elevation Calculation
//--------------------------------------------------------------------------
Helios helios;

double dAzimuth;
double dElevation;

//----------------------------------------------------------------------
// Lichtschranke Pins
//----------------------------------------------------------------------
gpio_num_t aPinNumber = GPIO_NUM_33;
gpio_num_t bPinNumber = GPIO_NUM_32;

//----------------------------------------------------------------------
// Puls Counter
//----------------------------------------------------------------------
unsigned long durationA;
unsigned long durationB;
volatile int32_t count = 0;
pcnt_unit_t unit = PCNT_UNIT_0;
pcnt_config_t r_enc_config;

//--------------------------------------------------------------------------
// Sensors
//--------------------------------------------------------------------------
#define SERVO_PIN GPIO_NUM_26
Servo servoMain; // Define our Servo

//--------------------------------------------------------------------------
// Sensors
//--------------------------------------------------------------------------
SDL_Arduino_INA3221 ina3221; // I2C


//--------------------------------------------------------------------------
// OTA Settings
//--------------------------------------------------------------------------
#include "SecureOTA.h"
const uint16_t OTA_CHECK_INTERVAL = 3000; // ms
uint32_t _lastOTACheck = 0;
bool wifi_connected = false;

int16_t getCountRaw()
{
  int16_t c;
  pcnt_get_counter_value(unit, &c);
  return c;
}




void setup_pulsecounter()
{

  // Puls Counter Setup
  gpio_pad_select_gpio(aPinNumber);
  gpio_set_direction(aPinNumber, GPIO_MODE_INPUT);
  gpio_pulldown_en(aPinNumber);

  r_enc_config.pulse_gpio_num = aPinNumber; //Rotary Encoder Chan A
  r_enc_config.ctrl_gpio_num = bPinNumber;  //Rotary Encoder Chan B

  r_enc_config.unit = unit;
  r_enc_config.channel = PCNT_CHANNEL_0;

  r_enc_config.pos_mode = PCNT_COUNT_INC; //Count Only On Rising-Edges
  r_enc_config.neg_mode = PCNT_COUNT_DIS; // Discard Falling-Edge

  r_enc_config.lctrl_mode = PCNT_MODE_KEEP;    // Rising A on HIGH B = CW Step
  r_enc_config.hctrl_mode = PCNT_MODE_REVERSE; // Rising A on LOW B = CCW Step

  r_enc_config.counter_h_lim = INT16_MAX;
  r_enc_config.counter_l_lim = INT16_MIN;

  pcnt_unit_config(&r_enc_config);

  // Filter out bounces and noise
  pcnt_set_filter_value(unit, 250); // Filter Runt Pulses
  pcnt_filter_enable(unit);

  /* Enable events on  maximum and minimum limit values */
  //pcnt_event_enable(unit, PCNT_EVT_H_LIM);
  //pcnt_event_enable(unit, PCNT_EVT_L_LIM);

  pcnt_counter_pause(unit); // Initial PCNT init
  pcnt_counter_clear(unit);
  pcnt_intr_enable(unit);
  pcnt_counter_resume(unit);
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

Button *b = NULL;

//--------------------------------------------------------------------------
// ESP Sleep Mode
//--------------------------------------------------------------------------

const float sleepPeriod = 2;   //seconds
#define uS_TO_S_FACTOR 1000000 //* Conversion factor for micro seconds to seconds */

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
#endif
}

//--------------------------------------------------------------------------
// MQTT
//--------------------------------------------------------------------------

//const char *mqtt_server = "192.168.1.144"; // Laptop
//const char *mqtt_server = "test.mosquitto.org"; // Laptop
const char *mqtt_server = "192.168.1.100"; // Raspberry
const char *mqtt_topic = "mrflexi/solarserver/";

#if (USE_MQTT)
PubSubClient client(wifiClient);
long lastMsgAlive = 0;
long lastMsgDist = 0;

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


void getLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  
  dataBuffer.data.timeinfo = timeinfo;
  dataBuffer.data.timeinfo.tm_year = dataBuffer.data.timeinfo.tm_year + 1900;
  dataBuffer.data.timeinfo.tm_mon = dataBuffer.data.timeinfo.tm_mon + 1;
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  }

void adjust_panel()
{
  int degree;
  degree = 90 - dataBuffer.data.sun_elevation ;
  Serial.printf("Panel degree: %d\n", degree);
  servoMain.write(degree); // Turn Servo Left to 45 degrees
 
}

void t_cyclic()
{
  dataBuffer.data.sleepCounter--;

  // Init and get the time
  getLocalTime();

   //----------------------------------------
  // Calc Sun Position München
  //----------------------------------------
  Serial.println();Serial.println();
  Serial.println("Sun Azimuth and Elevation Munich");
  helios.calcSunPos(2020, dataBuffer.data.timeinfo.tm_mon, dataBuffer.data.timeinfo.tm_mday, dataBuffer.data.timeinfo.tm_hour-2, dataBuffer.data.timeinfo.tm_min, 00.00, 11.57754, 48.13641);
  dataBuffer.data.sun_azimuth = helios.dAzimuth;
  dataBuffer.data.sun_elevation = helios.dElevation;
 

  Serial.printf("Azimuth: %f3\n", helios.dAzimuth);
  Serial.printf("Elevation: %f3\n", helios.dElevation);

  adjust_panel();
  delay(2000);
  
  //-----------------------------------------------------
  // Deep sleep
  //-----------------------------------------------------
#if (ESP_SLEEP)
  if (dataBuffer.data.sleepCounter <= 0)
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
  // Voltage and Current
  dataBuffer.data.busvoltage1 = ina3221.getBusVoltage_V(1);
  dataBuffer.data.current_1 = ina3221.getCurrent_mA(1);
  dataBuffer.data.current_2 = ina3221.getCurrent_mA(2);
  dataBuffer.data.current_3 = ina3221.getCurrent_mA(3);

   Serial.printf("Panel Voltage: %.2fV\n", dataBuffer.data.busvoltage1 );
   Serial.printf("Panel Current: %.0fmA\n", dataBuffer.data.current_1 );

// Temperatur
#if (USE_BME280)
  snprintf(volbuffer, sizeof(volbuffer), "%.1fC/%.1f%", bme.readTemperature(), bme.readHumidity());
  log_display(volbuffer);
#endif

  // GPS
  //gps.encode();

  // Update Display
  showPage(PAGE_VALUES);
}


void servo_sweep()
{

  servoMain.write(90); // Turn Servo Left to 45 degrees
  delay(2000);        // Wait 1 second
  servoMain.write(80); // Turn Servo Left to 45 degrees
  delay(2000);          // Wait 1 second
  servoMain.write(100);  // Turn Servo Left to 45 degrees
  delay(2000);          // Wait 1 second
  servoMain.write(90);   // Turn Servo Left to 0 degrees
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

  ESP_LOGI(TAG, "Starting..");
  Serial.println(F("ESP32 Sensor Board"));

 

  //----------------------------------------
  // Read I2C Devices
  //----------------------------------------
  i2c_scan();

  //-----------------------------------------------------
  // Hardeware Puls Counter for Motor Position Controll
  //-----------------------------------------------------
  setup_pulsecounter();

  //----------------------------------------
  // INA3221 I2C Power Sensor
  //----------------------------------------
  ina3221.begin();
  Serial.print("Manufactures ID=0x");
  int MID;
  MID = ina3221.getManufID();
  Serial.println(MID, HEX);

  dataBuffer.data.txCounter = 0;
  dataBuffer.data.sleepCounter = TIME_TO_NEXT_SLEEP;

  setup_display();
  //setup_sensors();
  setup_wifi();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  //----------------------------------------
  // OTA Over the air Update
  //----------------------------------------
#if (USE_OTA)
  if (WiFi.status() == WL_CONNECTED)
  {
    _lastOTACheck = millis();
    checkFirmwareUpdates();
  }
#endif

//----------------------------------------
// GPS
//----------------------------------------
//gps.init();
//gps.wakeup();
//gps.ecoMode();

//----------------------------------------
// Connection to MQTT Broker
//----------------------------------------
#if (USE_MQTT)
  setup_mqtt();
#endif

  //----------------------------------------
  // Tasks
  //----------------------------------------
  sleepTicker.attach(60, t_cyclic);
  displayTicker.attach(display_refresh, t_display);

  runmode = 1; // Switch from Terminal Mode to page Display
  //showPage(1);

//---------------------------------------------------------------
// Deep sleep settings
//---------------------------------------------------------------
#if (ESP_SLEEP)
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR * 60);
  Serial.println("Setup ESP32 to wake-up via timer after " + String(TIME_TO_SLEEP) +
                 " Minutes");
#endif

 

  b = new Button(39); // Boot Button

  b->setOnDoubleClicked([]() {
    Serial.println("doubleclicked");
  });
  b->setOnClicked([]() {
    Serial.println("clicked");
  });
  b->setOnHolding([]() {
    Serial.println("holding");
  });

  servoMain.attach(SERVO_PIN); // servo on digital pin 10

  servo_sweep();
  t_cyclic();
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

  //b->update();
}