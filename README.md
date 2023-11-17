
# F.E.R.N. Don't Burn - An IoT LoRaWAN Electronic Sensor Box to Detect Utility Pole Failures
### Jan 2023 - May 2023

With failures in utility poles one of the leading causes of wildfires, we were inspired to design a device to detect fallen utility poles. Our "F.E.R.N" Box is run through a MKRWAN 1310 (LoRaWAN compatible) with a MKR Shield soldered breadboard connecting a 6 axis accelerometer/gyroscope and a CO2 sensor. Using another MKRWAN 1310 to create a makeshift LoRaWAN gateway, our sender Arduino would communicate pole status info to our cloud server hosted in Firebase. Our server monitors the data and uses the Twilio API to send an SMS message in case of a potential fire. The entire project is housed in a fully water/weatherproof IP67 box and solar charged to minimize the amount of required maintenance.

Through multiple tests and tens of thousands of server read/writes, our project successfully sent alerts when the pole exceeded a tilt angle or if smoke was present.

The code for the Arduino and Firebase server were created by Jaythan Dao and Cailean Fernandes. 
- https://www.linkedin.com/in/jaythan-dao/ 
- https://www.linkedin.com/in/cailean-fernandes/
