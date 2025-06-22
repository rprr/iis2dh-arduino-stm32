## For iis2dh on Arduino stm32duino

I have a STWINKT1B eval board with a bunch of sensors. 

https://www.st.com/en/evaluation-tools/steval-stwinkt1b.html

The MCU on the board is STM32L4R9ZIJ6

The sensors are:
- ultra-wide bandwidth (up to 6 kHz), low-noise, 3-axis digital vibration sensor (IIS3DWB)
- 3D accelerometer + 3D Gyro iNEMO inertial measurement unit (ISM330DHCX) with machine learning core
- ultra-low-power high performance MEMS motion sensor (IIS2DH)
- ultra-low-power 3-axis magnetometer (IIS2MDC)
- digital absolute pressure sensor (LPS22HH)
- low-voltage digital local temperature sensor (STTS751)
- industrial grade digital MEMS microphone (IMP34DT05)
- analog MEMS microphone with frequency response up to 80 kHz (IMP23ABSU)

There are stm32duino drivers for some of them. 

This git repo provides an exampke program for the IIS2DH motion sensor. Inspired by 
the example: 
https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/iis2dh_STdC/examples/iis2dh_read_data_polling.c

## Instructions:
- Modify the sketch to match your SPI Pin connections including CS
- Copy the iis2dh_reg.c and iis2dh_reg.h files into separate tabs
- You may want to modify the Serial Port definition
- I also use LibPrintf
- Change LEDs as needed
