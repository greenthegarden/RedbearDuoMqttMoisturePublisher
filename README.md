# MQTT Moisture Publisher

Project to use a Redbear Duo microprossing board to gather and publish soil moisture, temperature and humidity via MQTT. The structure of the topics are compatible with Home Assistant MQTT Discovery.

## Hardware

### Redbear Duo

The [Redbear Duo](https://github.com/redbear/Duo) was acquired via a [Kickstarter campaign](https://www.kickstarter.com/projects/redbearinc/redbear-duo-a-small-and-powerful-wi-fi-ble-iot-boa). The board is compatible with Particle boards.

LED Status Indicators

| RGB LED State | Duo State |
| ------------- | --------- |
| flashing blue | listening mode |
| flashing green | attempting connection to access point |
| pulsing green | 
| solid green | listening mode |
| solid yellow | |

Device ID 3a001d000d47353033323637

### Battery Management

Solar power and battery management is provided by a [Seeedstudio LiPo Rider Pro](https://www.seeedstudio.com/LiPo-Rider-Pro.html). Unfortunately, this board does not provide telemetry but fitted within the case used. A x mAh LiPO battery was located within the case and 3W solar panel utilised.

### Soil Moisture Sensors

[Capacitive Soil Moisture Sensors](https://www.diymore.cc/products/2pcs-capacitive-soil-moisture-sensor-v1-2-analog-corrosion-resistant-dc-3-3-5-5v?_pos=3&_sid=a9bfd7fa9&_ss=r) were utilised as they are more robust for longer periods in the ground and not too expensive. Longer leads where created to enable the sensors to be relocated without moving the case and solar panel. The current draw for these type of sensors is approximately 5mA (source: https://thecavepearlproject.org/2020/10/27/hacking-a-capacitive-soil-moisture-sensor-for-frequency-output/).

### Temperature and Humidity Sensor

A [SHT10 Temperature and Humidity sensor](https://www.seeedstudio.com/Soil-Moisture-Temperature-Sensor-p-1356.html) with a environment shield is used to measure the temperature and humidity at the surface of the soil. The SHT10 sensor, consumes approximately 3mW when taking measurements (source: https://www.adafruit.com/product/1298). The [SHT1x-ESP](https://github.com/beegee-tokyo/SHT1x-ESP) library is used to get the measurements from the sensor.



https://www.digikey.jp/ja/maker/projects/how-to-build-a-photon-mqtt-logger/876ce49a8f914f0799a0f8b94519acc1

https://github.com/redbear/Duo

## Software

The Arduino IDE is required to be used to support the Redbear Duo board. I was not able to add the board definition to the PlatformIO my preferred Arduino development platform. Support the the Redbear Duo board is added using the Board Manger.

The source code for the project is hosted at https://github.com/greenthegarden/RedbearDuoMqttMoisturePublisher.

Before compiling the code, add a file named `secrets.h` to the root project directory with the following format

```cpp
//SSID (network name)
char SSID[] = "";
// Network password
char PASSWORD[] = "";
// MQTT Broker details
char BROKER_IP[] = "";
const uint16_t BROKER_PORT = 1883;
char CLIENTID[] = "duo_moisture";
```

## Libraries

The project uses modified versions of the following libraries. Due to the modifications made the library source code has been added directly to this project. A script, `libraries/get_libraries.sh` had been provided to get updated copies of the libraries.

### HAMqttDevice

The [HAMqttDevice library](https://www.arduino.cc/reference/en/libraries/hamqttdevice/) is used to provide support for [Home Assistant](https://www.home-assistant.io/) [MQTT Discovery](https://www.home-assistant.io/docs/mqtt/discovery/). It has been modified to support the Particle version of the Vector library utilised by the Redbear Duo.

### MQTT

THe [MQTT library](https://github.com/hirotakaster/MQTT) was found to be the most compatible MQTT library for the Redbear Duo.

## MQTT Messages

Five Home Assistant devices are defined using

```cpp
HAMqttDevice soil_device_memory("Soil Device Memory", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
HAMqttDevice soil_sht10_temperature("Soil SHT10 Temperature", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
HAMqttDevice soil_sht10_humidity("Soil SHT10 Humidity", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
HAMqttDevice soil_moisture_1("Soil Moisture 1", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
HAMqttDevice soil_moisture_2("Soil Moisture 2", HAMqttDevice::SENSOR, String(HA_MQTT_PREFIX));
```

### Device

Config Topic: `homeassistant/sensor/soil_device_memory/config`

Config Message structure:

```json
{
  "~": "homeassistant/sensor/soil_device_memory",
  "name": "Soil Device Memory",
  "unique_id": "soil_device_memory",
  "stat_t": "~/state",
  "json_attr_t": "~/attr",
  "stat_cla": "measurement",
  "unit_of_meas": "bytes",
  "dev": {
    "ids": "duo_test",
    "name": "Moisture Sensor",
    "mdl": "Duo",
    "sa": "garden",
    "mf": "Redbear"
  }
}
```

Attribute Topic: `homeassistant/sensor/soil_device_memory/attr`

Attribute Message structure: 

```json
{
  "sample_interval": "60",
  "ssid": "videoAtHome-2.4g",
  "ip_address": "192.168.1.165",
  "mac_address": "94:a1:a2:fd:71:f5",
  "client_id": "duo_moisture_94a1a2fd71f5"
}
```

State Topic: `homeassistant/sensor/soil_device_memory/state`

State Message structure: int

### Soil Temperature

Config Topic: `homeassistant/sensor/soil_sht10_temperature/config`

Config Message structure:

```json
{
  "~": "homeassistant/sensor/soil_sht10_temperature",
  "name": "Soil SHT10 Temperature",
  "unique_id": "soil_sht10_temperature",
  "dev_cla": "temperature",
  "stat_cla": "measurement",
  "unit_of_meas": "°C",
  "stat_t": "duo/sensor/soil_sht10",
  "val_tpl": "{{ value_json.temperature | int(0) }}",
  "dev":{
    "ids": "temp_hum_sensor",
    "name": "Moisture Sensor",
    "mdl": "SHT10",
    "sa": "garden",
    "mf": "Seeed"
  }
}
```

State Topic: `duo/sensor/soil_sht10`

State Message structure: 

```json
{
  "temperature": 24.75,
  "humidity": 68.517
}
```

### Soil Humidity

Config Topic: ` homeassistant/sensor/soil_sht10_humidity/config`

Config Message structure:

```json
{
  "~": "homeassistant/sensor/soil_sht10_humidity",
  "name": "Soil SHT10 Humidity",
  "unique_id": "soil_sht10_humidity",
  "dev_cla": "humidity",
  "stat_cla": "measurement",
  "unit_of_meas": "%",
  "stat_t": "duo/sensor/soil_sht10",
  "val_tpl": "{{ value_json.humidity | int(0) }}",
  "dev": {
    "ids": "temp_hum_sensor",
    "name": "Moisture Sensor",
    "mdl": "SHT10",
    "sa": "garden",
    "mf": "Seeed"
  }
}
```

State Topic: `duo/sensor/soil_sht10`

State Message structure: 

```json
{
  "temperature": 24.75,
  "humidity": 68.517
}
```

### Soil Moisture Sensor 1

Config Topic: `homeassistant/sensor/soil_moisture_1/config`

Config Message structure:

```json
{
  "~": "homeassistant/sensor/soil_moisture_1",
  "name": "Soil Moisture 1",
  "unique_id": "soil_moisture_1",
  "stat_t": "~/state",
  "stat_cla": "measurement",
  "val_tpl": "{{ value_json.measurement | int(0) }}",
  "dev": {
    "ids": "moisture_sensor_1",
    "name": "Capacitive Soil Moisture Sensor",
    "mdl": "V1.2",
    "sa": "garden",
    "mf": "DIYMORE.CC"
  },
  "json_attr_t": "~/attr"
}
```

Attribute Topic: `homeassistant/sensor/soil_moisture_1/attr`

Attribute Message structure:


```json
{
  "pin_data": "11",
  "power_pin": "1"
}
```

State Topic: `homeassistant/sensor/soil_moisture_1/state`

State Message structure: int

### Soil Moisture Sensor 2

Config Topic: `homeassistant/sensor/soil_moisture_2/config`

Config Message structure:

```json
{
  "~": "homeassistant/sensor/soil_moisture_2",
  "name": "Soil Moisture 2",
  "unique_id": "soil_moisture_2",
  "stat_t": "~/state",
  "stat_cla": "measurement",
  "val_tpl": "{{ value_json.measurement | int(0) }}",
  "dev": {
    "ids": "moisture_sensor_2",
    "name": "Capacitive Soil Moisture Sensor",
    "mdl": "V1.2",
    "sa": "garden",
    "mf": "DIYMORE.CC"
  }
}
```

Attribute Topic: `homeassistant/sensor/soil_moisture_2/attr`

Attribute Message structure: 

```json
{
  "pin_data": "13",
  "power_pin": "2"
}
```

State Topic: `homeassistant/sensor/soil_moisture_2/state`

State Message structure: int
