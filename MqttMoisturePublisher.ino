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
 *************** Configuration details ***************
 */

// for debugging.
#define DEBUG_CODE_SERIAL_OUTPUT true
#if defined(DEBUG_CODE_SERIAL_OUTPUT)
#define code_debug_print(fmt, ...) Serial.printf("[DEBUG] CODE " fmt, ##__VA_ARGS__)
#else /* !DEBUG_CODE_SERIAL_OUTPUT */
#define code_debug_print(fmt, ...) ((void)0)
#endif /* DEBUG_CODE_SERIAL_OUTPUT */

#include "secrets.h"

// set USE_SLEEP to false to run a delay rather than sleep duo
#ifndef USE_SLEEP
#define USE_SLEEP true
#endif

// set to true to run test => will not sleep
#ifndef TEST_MODE
#define TEST_MODE true
#endif

#if TEST_MODE
#define DEBUG_MQTT_SERIAL_OUTPUT true
const unsigned long SAMPLE_INTERVAL = 1UL * 60UL * 1000UL; // 60 seconds
#else
const unsigned long SAMPLE_INTERVAL = 15UL * 60UL * 1000UL; // 15 minutes
#endif


/*
 *************** Configure Duo ***************
 */

const int DUO_BLUE_LED = D7;


/*
 *************** Configure MQTT ***************
 */

#include "MQTT.h"

// buffer for payload
const unsigned int PAYLOAD_LENGTH = 1023;
const unsigned int KEEP_ALIVE = 60;

char payload[PAYLOAD_LENGTH];

String client_id;

// This is called when a message is received. However, we do not use this feature in
// this project so it will be left empty
void callback(char* topic, byte* payload, unsigned int length) 
{}

/**
 * if want to use IP address,
 * const uint8_t server[] = { XXX,XXX,XXX,XXX };
 * MQTT client(server, 1883, callback);
 * want to use domain name,
 * exp) iot.eclipse.org is Eclipse Open MQTT Broker: https://iot.eclipse.org/getting-started
 * MQTT client("mqtt.eclipse.org", 1883, callback);
 **/
// MQTT(const char *domain, uint16_t port, int maxpacketsize, int keepalive, void (*callback)(char *, uint8_t *, unsigned int), bool thread = false);
MQTT client(BROKER_IP, BROKER_PORT, PAYLOAD_LENGTH, KEEP_ALIVE, callback);

/*
 *************** Configure Home Assistant Integration ***************
 */

// #include <HAMqttDevice.h>
#include "HAMqttDevice.h"

char HA_MQTT_PREFIX[] = "homeassistant";

HAMqttDevice soil_device_memory("Soil Device Memory", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
HAMqttDevice soil_sht10_temperature("Soil SHT10 Temperature", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
HAMqttDevice soil_sht10_humidity("Soil SHT10 Humidity", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
HAMqttDevice soil_moisture_1("Soil Moisture 1", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
HAMqttDevice soil_moisture_2("Soil Moisture 2", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));

/*
 *************** Configure Device Message ***************
 */

// if formatted will return in form ab:cd:ef:01
// else will return in form abcded01
String macAddressToString(bool formatted=true)
{
  // the MAC address of your Wifi
  byte mac_buffer[6];

  // print your MAC address:
  WiFi.macAddress(mac_buffer);
  String mac_address = "";
  for (byte octet = 0; octet < 6; octet++)
  {
    mac_address += String(mac_buffer[octet], HEX);
    if (octet < 5 && formatted)
    {
      mac_address += ':';
    }
  }
  return mac_address;
}

void deviceConfig()
{
  soil_device_memory.enableStateTopic();
  soil_device_memory.enableAttributesTopic();
  soil_device_memory
      .addConfigVar("stat_cla", "measurement")
      .addConfigVar("unit_of_meas", "bytes")
      .addConfigVar("dev", "{\"ids\": \"duo_test\", \"name\": \"Moisture Sensor\", \"mdl\": \"Duo\", \"sa\": \"garden\", \"mf\": \"Redbear\"}");
  soil_device_memory
      .addAttribute("sample_interval", String(SAMPLE_INTERVAL / 1000))
      .addAttribute("ssid", String(WiFi.SSID()))
      .addAttribute("ip_address", String(WiFi.localIP()))
      .addAttribute("mac_address", macAddressToString())
      .addAttribute("client_id", client_id);
}

void devicePublish()
{
  int memory = System.freeMemory();
  client.publish(soil_device_memory.getStateTopic(), String(memory));
}

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

  soil_moisture_1.enableStateTopic();
  soil_moisture_1
      .addConfigVar("stat_cla", "measurement")
      .addConfigVar("val_tpl", "{{ value_json.measurement | int(0) }}")
      .addConfigVar("dev", "{\"ids\": \"moisture_sensor_1\", \"name\": \"Capacitive Soil Moisture Sensor\", \"mdl\": \"V1.2\", \"sa\": \"garden\", \"mf\": \"DIYMORE.CC\"}");
  soil_moisture_1.enableAttributesTopic();
  soil_moisture_1
      .addAttribute("pin_data", String(MOISTURE_SENSORS[0][0]))
      .addAttribute("power_pin", String(MOISTURE_SENSORS[0][1]));

  soil_moisture_2.enableStateTopic();
  soil_moisture_2
      .addConfigVar("stat_cla", "measurement")
      .addConfigVar("val_tpl", "{{ value_json.measurement | int(0) }}")
      .addConfigVar("dev", "{\"ids\": \"moisture_sensor_2\", \"name\": \"Capacitive Soil Moisture Sensor\", \"mdl\": \"V1.2\", \"sa\": \"garden\", \"mf\": \"DIYMORE.CC\"}");
  soil_moisture_2.enableAttributesTopic();
  soil_moisture_2
      .addAttribute("pin_data", String(MOISTURE_SENSORS[1][0]))
      .addAttribute("power_pin", String(MOISTURE_SENSORS[1][1]));
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
    // publish reading
    String topic;
    if (i==0)
      topic = soil_moisture_1.getStateTopic();
    else if (i == 1)
      topic = soil_moisture_2.getStateTopic();
    client.publish(topic, String(measurement));

    // time before next measurement
    delay(1000);
  }
}


/*
 *************** Configure temperature/humidity sensor ***************
 */

#include <SHT1x-ESP.h>

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

char SHT10_STATUS_TOPIC[] = "duo/sensor/soil_sht10";

void sht10Config()
{
  // set power pin as output
  pinMode(SHT10_POWER_PIN, OUTPUT);
  digitalWrite(SHT10_POWER_PIN, LOW);

  soil_sht10_temperature
      .addConfigVar("dev_cla", "temperature")
      .addConfigVar("stat_cla", "measurement")
      .addConfigVar("unit_of_meas", "°C")
      .addConfigVar("stat_t", SHT10_STATUS_TOPIC)
      .addConfigVar("val_tpl", "{{ value_json.temperature | int(0) }}")
      .addConfigVar("dev", "{\"ids\": \"temp_hum_sensor\", \"name\": \"Moisture Sensor\", \"mdl\": \"SHT10\", \"sa\": \"garden\", \"mf\": \"Seeed\"}");
  soil_sht10_humidity
      .addConfigVar("dev_cla", "humidity")
      .addConfigVar("stat_cla", "measurement")
      .addConfigVar("unit_of_meas", "%")
      .addConfigVar("stat_t", SHT10_STATUS_TOPIC)
      .addConfigVar("val_tpl", "{{ value_json.humidity | int(0) }}")
      .addConfigVar("dev", "{\"ids\": \"temp_hum_sensor\", \"name\": \"Moisture Sensor\", \"mdl\": \"SHT10\", \"sa\": \"garden\", \"mf\": \"Seeed\"}");
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
  // prepare payload
  payload[0] = '\0';
  // create payload
  JSONBufferWriter writer(payload, sizeof(payload));
  writer.beginObject();
  writer.name("temperature").value(temp_c);
  writer.name("humidity").value(humidity);
  writer.endObject();
  writer.buffer()[min(writer.bufferSize(), writer.dataSize())] = 0;
  // publish reading
  client.publish(SHT10_STATUS_TOPIC, payload);
}


/*
 *************** Arduino Methods ***************
 */

// put your setup code here, to run once:
void setup() {

  code_debug_print(" Setup ...\n");

  pinMode(DUO_BLUE_LED, OUTPUT);

  // sht10 sensor configuration
  sht10Config();
  // moisture sensors configuration
  moistureConfig();

  // Connect to network
  WiFi.on();
  WiFi.setCredentials(SSID, PASSWORD);
  WiFi.connect();

  // wait for Wifi connection
  // while (WiFi.connecting())
  // {
  // }

  // Wait for wifi to be ready
  while (!WiFi.ready()) {}
  //   code_debug_print(" Did not connect to %s\n", SSID);

  // wait for wifi connection to be established
  delay(3000);

  // connect to broker with unique client ID based on MAC address
  client_id = String(CLIENTID) + "_" + macAddressToString(false);
  String willTopic = String("duo") + String("/") + macAddressToString(false) + String("/status");
 
  // client.connect(client_id);
  // return connect(id, NULL, NULL, 0, QOS0, 0, 0, true);
  // connect(id, user, pass, 0, QOS0, 0, 0, true);
  // bool connect(const char *id, const char *user, const char *pass, const char* willTopic, EMQTT_QOS willQos, uint8_t willRetain, const char* willMessage, bool cleanSession, MQTT_VERSION version = MQTT_V311);
  client.connect(client_id, NULL, NULL, willTopic, MQTT::EMQTT_QOS::QOS0, 0, "offline", true);

  if (client.isConnected())
  {
    client.publish(willTopic, "online", true);

    code_debug_print("Connected to broker");

    // device memory configuration
    // needs to occur after connection to network
    deviceConfig();

    // Publish config payloads
    client.publish(soil_device_memory.getConfigTopic(), soil_device_memory.getConfigPayload());
    client.publish(soil_sht10_temperature.getConfigTopic(), soil_sht10_temperature.getConfigPayload());
    client.publish(soil_sht10_humidity.getConfigTopic(), soil_sht10_humidity.getConfigPayload());
    client.publish(soil_moisture_1.getConfigTopic(), soil_moisture_1.getConfigPayload());
    client.publish(soil_moisture_2.getConfigTopic(), soil_moisture_2.getConfigPayload());

    // Publish attributes payloads
    client.publish(soil_device_memory.getAttributesTopic(), soil_device_memory.getAttributesPayload());
    client.publish(soil_moisture_1.getAttributesTopic(), soil_moisture_1.getAttributesPayload());
    client.publish(soil_moisture_2.getAttributesTopic(), soil_moisture_2.getAttributesPayload());
  }
}

// put your main code here, to run repeatedly:
void loop() {

  if (client.isConnected())
  {
    // client.loop();

    // Publish measurements
    devicePublish();
    sht10Measurement();
    moistureMeasure(); 
  }
  #if TEST_MODE
    delay(SAMPLE_INTERVAL);
  #else
    #if USE_SLEEP
      // Sleep system for SLEEP_DURATION seconds
      System.sleep(SLEEP_MODE_DEEP, SAMPLE_INTERVAL);
    #else
      delay(SAMPLE_INTERVAL);
    #endif
  #endif

}
