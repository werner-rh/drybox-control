DryBox dryController
===

## Version 0.50

## Requirements
- Arduino IDE 1.8.2 or higher
- NewliquidCrystal (I2C) https://github.com/fmalpartida/New-LiquidCrystal/tree/master
- ESP8266: PubSubClient by Nick O'Leary
- SEP8266: ArduinoJson by Benoit Blanchon

- 3D printing files and other parts list: https://www.printables.com/de/model/724722-filament-dry-box-heated-ventilated-arduino-control
- Improved version: https://www.printables.com/model/1093573-enhanced-filament-dry-box-heated-ventilated-arduin

## Description
Controller for an active heated and ventilated Filament DryBox. The Dryer contains a PTC Heater and a Fan for fresh dry air.

Original version is for Arduino Nano
Improved Version is for Wemos D1 Mini (ESP8266); additional features:
- sensor data transmission via mqtt
- example files for mqtt --> telegraf --> influxdb --> grafana
- PCB layout in kicad
- ==> pay attention to set your wifi credentials in Secrets.h (for ESP8266)

With one Heater, the box may barely reaches 55 degree. As max i reached 57 degree. I recommend not to go over 52-53 degree. If you need higher temperatures, install a second heater. The MosFet switch will manage this. But you also need a stronger power supply (12V, 8-10A). A simple improvement is insulating the walls inside. I used a car sun protect shield, with reflection and insulating. This improves the performance of the box sigifignant. The warm up is much faster and also eaysier to to reach 55 degrees.


## Recommended drying temperatures
- PLA, PLA+                : 40-45 degree
- ABS, ABS+,  PETG         : 45-50 degree
- PVA,PVB, ASA, TPU, PMMA  : 50-55 degree

## Trouble shooting for ESP8266 version
- Check step by step in the chain  esp --> mqtt --> telegraf --> influxdb --> grafana
- Let DryBox run for all of the following tests.
- esp: Check the serial console: Is wifi working? Do you have a mqtt connection? Have you changed the corresponding parameters in Secrets.h?
- mqtt: Use the commandline or the tool mqtt explorer to observe the mqtt messages from DryBox. There should be a main node labeled like in Secrets.h
- telegraf: telegraf has input and output interfaces. First check the log for any errors in the input interface. Then check if the output interface wrote into the file listed in telegraf.conf. If this is working, you can comment out the file section in telegraf.conf. Now check telegraf´s output interface to influxdb. Do you have set the influxdb token in telegraf.conf?
- influxdb: Have you generated a bucket "DryBox" an configured this in Secrets.h? Have you generated a token connected to the bucket for telegraf and grafana with the necessary permissions? Use the influxdb data explorer to check if any DryBox data is written from telegraf.
- grafana: Have you created a datasource "InfluxDB" with query language "flux"? Is the organization (e.g. "Homelab") the same in the config files for telegraf, influxdb and grafana? Did you use the right token from influxdb? Did you select the configured datasource when importing the telegraf.conf file? 
