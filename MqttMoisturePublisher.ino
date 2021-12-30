/* 
 * Defaultly disabled. More details: https://docs.particle.io/reference/firmware/photon/#system-thread 
 */
//SYSTEM_THREAD(ENABLED);

/*
 * Defaultly disabled. If BLE setup is enabled, when the Duo is in the Listening Mode, it will de-initialize and re-initialize the BT stack.
 * Then it broadcasts as a BLE peripheral, which enables you to set up the Duo via BLE using the RedBear Duo App or customized
 * App by following the BLE setup protocol: https://github.com/redbear/Duo/blob/master/docs/listening_mode_setup_protocol.md#ble-peripheral 
 * 
 * NOTE: If enabled and upon/after the Duo enters/leaves the Listening Mode, the BLE functionality in your application will not work properly.
 */
//BLE_SETUP(ENABLED);

/*
 * SYSTEM_MODE:
 *     - AUTOMATIC: Automatically try to connect to Wi-Fi and the Particle Cloud and handle the cloud messages.
 *     - SEMI_AUTOMATIC: Manually connect to Wi-Fi and the Particle Cloud, but automatically handle the cloud messages.
 *     - MANUAL: Manually connect to Wi-Fi and the Particle Cloud and handle the cloud messages.
 *     
 * SYSTEM_MODE(AUTOMATIC) does not need to be called, because it is the default state. 
 * However the user can invoke this method to make the mode explicit.
 * Learn more about system modes: https://docs.particle.io/reference/firmware/photon/#system-modes .
 */
#if defined(ARDUINO) 
SYSTEM_MODE(SEMI_AUTOMATIC); 
#endif

// Simple test of using sleep(SLEEP_MODE_DEEP, 30). This halts execution of your code, and when it
// wakes up again, it goes through setup() again with all variables cleared.

// Running this test without a battery connected to VBAT is interesting, however, because retained
// variables ARE preserved in deep sleep, even without a battery.

// IMPORTANT NOTE: If using retained variables to preserve values across deep sleep when NOT using
// a battery, be sure to tie VBAT to ground. This somewhat counter-intuitive step is necessary
// because otherwise when you first power up the device, the retained memory will not be initialized,
// so it will contain random values, which will probably confuse your code!

// Source: https://community.particle.io/t/sleep-mode-explained-using-code-samples/21173
// STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

// Using JSON is a pain with standard library
// Information at https://docs.particle.io/reference/device-os/firmware/#json

/*
 *************** Configure Duo ***************
 */

const int DUO_BLUE_LED = D7;
const unsigned long SLEEP_DURATION = 15UL * 60UL * 1000UL; // 15 minutes

/*
 *************** Configure MQTT ***************
 */

#include "MQTT.h"

char BROKER[] = "192.168.1.186";
char CLIENTID[] = "duo_moisture";
char TOPIC_AVAILABILITY[] = "duo/moisture/availability";
char TOPIC_MOISTURE[] = "duo/moisture/";
char TOPIC_SHT10[] = "duo/moisture/sht10";

char topic[32];
char payload[128];
char buf[2];

// This is called when a message is received. However, we do not use this feature in
// this project so it will be left empty
void callback(char* topic, byte* payload, unsigned int length) 
{}

MQTT client(BROKER, 1883, callback);


/*
 *************** Configure Moisture Sensors ***************
 */

// moisture sensor configuration: data pin, power pin
const unsigned int SENSORS = 2;
const unsigned int MOISTURE_SENSORS[][SENSORS] = {{A1, D1}, {A3, D2}};
const unsigned long MOISTURE_SETTLING_DURATION = 10UL * 1000UL;  // 10 seconds

void moistureConfig() {
  for (unsigned int i = 0; i < SENSORS; ++i)
  {
    // set data pins as imputs
    pinMode(MOISTURE_SENSORS[i][0], INPUT);
    // set power pins as outputs
    pinMode(MOISTURE_SENSORS[i][1], OUTPUT);
    digitalWrite(MOISTURE_SENSORS[i][1], LOW);
  }
}

void moistureMeasure() {
  for (unsigned int i = 0; i < SENSORS; ++i)
  {
    unsigned int data_pin = MOISTURE_SENSORS[i][0];
    unsigned int power_pin = MOISTURE_SENSORS[i][1];
    // switch on sensor
    digitalWrite(power_pin, HIGH);
    digitalWrite(DUO_BLUE_LED, HIGH);
    // let sensor stabalise
    delay(MOISTURE_SETTLING_DURATION);
    // take reading
    unsigned int measurement = analogRead(data_pin);
    // switch off sensor
    digitalWrite(power_pin, LOW);
    digitalWrite(DUO_BLUE_LED, LOW);
    // prepare MQTT topic and payload
    topic[0] = '\0';
    payload[0] = '\0';
    buf[0] = '\0';
    // create topic
    strcat(topic, TOPIC_MOISTURE);
    strcat(topic, "/");
    strcat(topic, itoa(i, buf, 10));
    // create payload
    JSONBufferWriter writer(payload, sizeof(payload));
    writer.beginObject();
    writer.name("measurement").value(measurement);
    writer.name("pin_data").value(data_pin);
    writer.name("pin_power").value(power_pin);
    writer.endObject();
    writer.buffer()[min(writer.bufferSize(), writer.dataSize())] = 0;
    // publish reading
    client.publish(topic, payload);
    // time before next measurement
    delay(1000);
  }
}


/*
 *************** Configure temperature/humidity sensor ***************
 */

// #include <SHT1x.h>
#include <SHT1x-ESP.h>
#include "dtostrf.h"

// Specify data and clock connections and instantiate SHT1x object
// https://www.seeedstudio.com/Soil-Moisture-Temperature-Sensor-p-1356.html
// https://github.com/greenthegarden/SHT1x
// https://www.sparkfun.com/datasheets/Sensors/SHT1x_datasheet.pdf
const unsigned int SHT10_DATA_PIN = D15; // blue/white (UNO: 10)
const unsigned int SHT10_CLOCK_PIN = D16; // yellow (UNO: 11)
const unsigned int SHT10_POWER_PIN = D6; // red
const unsigned long SHT10_SETTLING_DURATION = 5UL * 1000UL; // 3 seconds

// initialise sensor
SHT1x sht1x(SHT10_DATA_PIN, SHT10_CLOCK_PIN, SHT1x::Voltage::DC_3_3v);

float temp_c;
float humidity;

void sht10Config()
{
  // set power pin as output
  pinMode(SHT10_POWER_PIN, OUTPUT);
  digitalWrite(SHT10_POWER_PIN, LOW);
}

void sht10Measurement()
{
  // switch on sensor
  digitalWrite(SHT10_POWER_PIN, HIGH);
  digitalWrite(DUO_BLUE_LED, HIGH);
  // let sensor stabalise
  delay(SHT10_SETTLING_DURATION);
  // read value from the sensor
  temp_c = sht1x.readTemperatureC();
  humidity = sht1x.readHumidity();
  // switch off sensor
  digitalWrite(SHT10_POWER_PIN, LOW);
  digitalWrite(DUO_BLUE_LED, LOW);
  // prepare MQTT topic and payload for temperature
  topic[0] = '\0';
  payload[0] = '\0';
  // create topic
  strcat(topic, TOPIC_SHT10);
  // create payload
  JSONBufferWriter writer(payload, sizeof(payload));
  writer.beginObject();
  writer.name("temperature").value(temp_c);
  writer.name("humidity").value(humidity);
  writer.endObject();
  writer.buffer()[min(writer.bufferSize(), writer.dataSize())] = 0;
  // publish reading
  client.publish(topic, payload);
}


/*
 *************** Configure Network ***************
 */

#include "secrets.h"


/*
 *************** Arduino Methods ***************
 */

// put your setup code here, to run once:
void setup() {

  pinMode(DUO_BLUE_LED, OUTPUT);

  // sht10 sensor configuration
  sht10Config();
  // moisture sensors configuration
  moistureConfig();

  // Connect to network
  WiFi.on();
  WiFi.setCredentials(SSID, PASSWORD);
  WiFi.connect();

  // Connect to the MQTT broker as CLIENTID
  client.connect(CLIENTID);
}

// put your main code here, to run repeatedly:
void loop() {

  // publish/subscribe
  if (client.isConnected())
  {
    client.publish(TOPIC_AVAILABILITY, "online");
    // Take measurementa
    sht10Measurement();
    moistureMeasure();
  }

  // Sleep system for SLEEP_DURATION seconds
  // System.sleep(SLEEP_MODE_DEEP, SLEEP_DURATION);
  delay(15000);
}
