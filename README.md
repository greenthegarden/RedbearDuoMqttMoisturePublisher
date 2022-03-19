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

A [SHT10 Temperature and Humidity sensor](https://www.seeedstudio.com/Soil-Moisture-Temperature-Sensor-p-1356.html) with a environment shield is used to measure the temperature and humidity at the surface of the soil. The SHT10 sensor, consumes approximately 3mW when taking measurements (source: https://www.adafruit.com/product/1298)



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

