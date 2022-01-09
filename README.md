# Arduino-PCD
Power consumption measurement device

Based on an Arduino Mega board associated with a current transformer sensor and an Airlift Wifi shield.

Samples AC current # 410 Hz (ajustable) and computes RMS current integrated over 200 samples.

Then computes instanenous AC power in W and total consumption over time in Wh.

WiFi allows to retrieve UTC time from a NTP server for measurement time stamp.

Measurements are averaged over 10 s and stored on a SD card for further analysis.
