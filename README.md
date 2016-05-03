Repo for bachelor thesis project for group E1624 at NTNU/HiST.

This repo contains the firmware for nRF52. 
It's an ultra low power BLE stepcounter utilizing an ADXL362 accellerometer to put the nRF52 to sleep when the accelleration drops below a certain threshold and wakes it up when it goes above another threshold.
The stepcounter is implemented with a on-chip comparator which input signal is driven by a piezo buzzer mounted in the heel of a shoe. 
This piezo buzzer also powers the circuit through an LTC3588-1, an energy harvesting solution ment for piezo elements. 

Project should compile with one warning if extracted correctly into the SDK.

Made by Håkon S. Holdhus and Lars J. Hammervold, 
with help from Hans Elfberg at Nordic Semiconductor.
