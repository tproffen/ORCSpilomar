# Code for the microcontroller

## Tiny2040
This contains the unchanged code for the Tiny2040 controller taking in to account the limited computer power and memory.

## Tiny2350
This contains the unchanged code for the new Tiny2350 controller. You can run this code with the TMC2209 divers rather than the DRV8825. However, this requires these changes if you are using the original pilomar PCB:

* Remove the header pins for M2, AZ-F and ALT-F from Tiny. Run wire from Tiny 5V to AZ-F and ALT-F which will supply the needed 5V to the new stepper controllers.  

* On the bottom of the PCB run a wire from the 5V pin of the Tiny controller to the pins labelled AZ-F and ALT-F. THese connect to the logic 5V in on the TMC2209 (same location as the fault pin on the DRV8825). Note that in this configuration the fault detection will (obvioulsy) not work.

* Remove (bottom) header pins PDN,PDN,CLK from the new controller. These pins are also accessible from the top on some boards. 

No software changes needed, except for an update to the microstepping parameters in `data/pilomar_params.json` to match the microstepping selected with M0 and M1. 


## Tiny2350-tmc2209
This code is a very early draft version to use the UART interface of the TMC2209 stepper controllers. Because the Tiny only supports two UART connections (and one is used for talking to the Raspberry Pi), we use one UART to address both steppers. The driver board can have their ID set using the M0 and M1 pins. For details check the draft schematics and board in the `KiCad` folder.

The TMC driver code is based on https://github.com/Chr157i4n/TMC2209_Raspberry_Pi 
