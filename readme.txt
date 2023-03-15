This repo is an open source version of firmware for the Iguana Smart Sack available at https://www.amazon.com/Smart-Sack-Iguana/dp/B07SYBHXJ1/
It can be modified and uploaded to the Bluetooth module in the Smart Sack. See the file DFU notes for more info on the Over The Air (OTA) update process.

All required Nordic SDK files are included in this repo.
IDE: Segger SES for ARM 7.10a (free to use for Nordic microcontrollers)

Default behavior: module wakes up when it detects freefall. Accelerometer values (x, y, z) are updated in the advertising data once per second. The module goes to sleep after 60 seconds. A connection can be made from a mobile device using nRF Connect. Text commands can be sent on the "Unknown Service".
Commands:
'a' - start streaming accelerometer data 
'c' - set advertising power to 4
'd' - set advertising power to 0
'e' - set advertising power to -40
'fxxx' - set go to sleep time in seconds ('f060' is 60 seconds)
'g' - go to sleep now
'm' - never go to sleep




