Author: Patrick Souty
Initial creation date: 31/01/2020

- Version 1.0 on 01/02/2021 
  - functionnal version 
  - only limitation : CAN resolution gives a measurment threshold around 20 W (quantification noise)
- Version 2.0 on 02/02/2021
  - Web server function added and Time library use
- Version 3.0 on 02/02/2021
  - 5V autocalibration by initial measurment of the internal bandgap diode
- Version 4.0 on 03/02/2021
  - current measurement using differential mode of the ADC with internal gain set to 10
- Version 5.0 on 05/12/2021
  - automatic setting of ADC gain based on current measurements and processing time information displayed
- Version 6.0 on 10/02/2021
  - data log storage on SD card (prelimnary version)
  - computing of cumulative power consumption
- Versio 7.0 on 11/02/2021 
  - data log storage on SD card (final version)
- Version 7.1 on  13/02/2021
  - switch-off LCD display backlight after 1st data storage on SD card
- Version 8.0 on 06/03/2021
  - transfer of definitions and standard functions into MegaBox.h library
  - management of measurement range by push buttons instead of automatic management
  - management of LCD display backlight by a dedicated switch

Next foreseen evolution:    # on line SD card files downloading through Web server (based on my other project DataLogger)
                             
                             