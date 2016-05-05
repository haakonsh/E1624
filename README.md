

Wireless biometric data acquisition powered by human locomotion
===================
Repo for bachelor thesis project for group E1624 at NTNU/HiST.

This repo contains the firmware for nRF52. It's an ultra low power BLE stepcounter utilizing an ADXL362 accellerometer to put the nRF52 to sleep when the accelleration drops below a certain threshold, and wakes it up when it goes above another threshold. The stepcounter is implemented with a on-chip comparator which input signal is driven by a piezo buzzer mounted in the heel of a shoe. This piezo buzzer also powers the circuit through an LTC3588-1, an energy harvesting solution ment for oscillating piezoelectric materials.

The BLE protocol is not fully implemented. At this point the Bluetooth Low Energy(BLE) Payload Data Unit(PDU) is not conforming to any protocol. It's just sending out indirect advertising packets. The reason is the high current consumption of the SoftDevice's boot sequence, and that of a BLE connection event (listening for packets is very expensive). In the future, some protocols (Eddystone etc.) may support indirect advertising without a connection.

The radio and spi are controlled with proprietary lightweight drivers included in the repo, however the spi could easily run from the SDK driver.

Project should compile with one warning if extracted correctly into the [nRF5_SDK_v11.x.x](https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v11.x.x/) SDK



---------
Flow chart:
----------
![Flow chart](http://i.imgur.com/0QBc4jC.png)
----------

####Made by Håkon S. Holdhus and Lars J. Hammervold, with help from Hans Elfberg at Nordic Semiconductor.
![Nordic logo](https://upload.wikimedia.org/wikipedia/commons/thumb/a/ae/Nordic_Semiconductor.svg/2000px-Nordic_Semiconductor.svg.png)

