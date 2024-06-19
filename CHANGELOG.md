# Changelog
All notable changes for this project will be documented in this file

## [0.49] - 2024-06-18
### Added
- Handling sensor error. If sensor fails during drying cycle, then drying process will be stopped and error message appear on the screen.
  
## [0.48] - 2024-05-08
### changed
- Refactoring Heating data. All values for heating power and heater fan speed are handled in an own class 
### Added
- Turbo mode for faster heating up in the lower ranges
- values for ventialtion time and final ventilation time
- replaced fix define for air exchange intervall with value in HeatingData

## [0.47] - 2024-05-05
### changed
- bugfix in function dryController(). Heater now is shut down after ventilation.
- bugfix in rampUpTable. Range over 50 degrees wasn't regarded and so powered to low.
### Added
- additional range over over 45 and 50 degrees. Now there are 6 ranges.
- display showing state of heater, heaterfan and ventilation during drying.
- refactored initializing power values for each tempeture range.
- adjust current values for low Hysteresis

## [0.46]
### Added
- last final ventilation after dry process is finished

## [0.45]
### Added
- first final version
