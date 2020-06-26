# EV_VCM_1
Open Source EV VCM
This project is my development of an EV VCM to run my personal car. Others may find this to be useful so i'm putting it out there. Not everything is going to work for everyone,
but if you have the know how, you can use this as a launchpad to re-work it to suite your own needs.
I deliberately wrote the code to be very comment heavy. I know what a brain drain it is to look at other peoples uncommented code, or even just a complex one that you wrote yourself.
Overall, the function of the VCM is to manage the drive unit, Battery balancing and management, charge control, throttle and regen sensor handling, inertial traction control,
thermal management, datalogging, and whatever else might fall into it's wheelhouse.

key things to know, This equipment and code is geared to work with some specific hardware:
specifically written for Teensy 4.1 (too many I/O for a  4.0)
Open source inverter controller (huebner/EVBMW inverter boards) operated directly via CAN
Open source tesla battery charger (currently on the gen2/3 V4 board)
Tesla model S charge port
Chevy Volt gen 1 battery packs, either 1 or 2 of them. on the 500kbps can bus, so the K16 module needs to be there.
E46 BMW SMG shifter
Chevy Volt heater control valves, 1 or 2 of them
Chevy Volt electric coolant pumps (or optionally, a PWM controlled pump)
Chevy Volt A/C compressor
Chevy Volt DC-DC converter
up to 2 SPAL brushless fans (chevy volts, or newer GMC acadia, ZL1 camaro, c8 Vette) or any other PWM operated fan.
2, 0-5V A/C pressure tranducers (1 high, 1 low)
3 thermistor 2 for A/C temps and 1 ambient sensor
dual channel accelerator 0-5V compatible
dual channel brake sensor 0-5V compatible (thinking of using a position sensor and a pressure sensor seperately)

Like I said before, the code is and will be comment heavy, so there are more functional descriptions in the code itself.
