DryBox dryController
===

## Version 0.49

## Requirements
- Arduino IDE 1.8.2 or higher
- NewliquidCrystal (I2C)

- 3D printing files and other parts list: https://www.printables.com/de/model/724722-filament-dry-box-heated-ventilated-arduino-control

## Description
Controller for an active heated and ventilated Filament DryBox. The Dryer contains a PTC Heater and a Fan for fresh dry air.

With one Heater, the box may barely reaches 55 degree. As max i reached 57 degree. I recommend not to go over 52-53 degree. If you need higher temperatures,
install a second heater. The MosFet switch will manage this. But you also need a stronger power supply (12V, 8-10A). A simple improvement
is insulating the walls inside. I used a car sun protect shield, with reflection and insulating. This improves the performance of
the box sigifignant. The warm up is much faster and also eaysier to to reach 55 degrees.


## Recommended drying temperatures
- PLA, PLA+                : 40-45 degree
- ABS, ABS+,  PETG         : 45-50 degree
- PVA,PVB, ASA, TPU, PMMA  : 50-55 degree
