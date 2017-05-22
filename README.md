# Ambient Light Sensor Service

This repository provides a way of registering a GATT Service with Nordic’s SoftDevice, specifically their S110 Peripheral variety.  The GATT Service registered provides the ambient lux as an attribute that can be read by a BLE Central device.  To do this, the nRF51 must be connected to an AMS TSL2561 Ambient Light Sensor via an I2C interface.

## Organization
* ext_drivers: A driver that relies on Nordic's TWI peripheral libraries to initialize and read from the TSL2561 over I2C.  The Lux calculation also occurs in these files.
* ext_services: The definition of the ALSS GATT Service as it pertains to registering with Nordic's SoftDevice.

## Use
1. Create a struct of type ble_alss_t.
2. Pass the struct to ble_alss_init().
3. Call ble_alss_on_ble_evt() from the on_ble_evt function you registered with the SoftDevice.
4. Use ble_alss_on_lux_change() to notify the BLE Central of changes to Lux values asynchronously.
