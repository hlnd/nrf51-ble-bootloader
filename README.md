nrf51-ble-bootloader
====================

A proof-of-concept bootloader application for the nRF51822/S110 softdevice

This application is to be written in the bottom of code region 1 on an nRF51822, with the S110 present. It allows you to
update an application residing in the rest of code region 1. It is set up to use the space from 0x14000 to 0x20000, but 
doesn't actually need all this space.

If an application has been written, and button 0 is not pressed when starting, the application will be started. If 
button 0 is held, the bootloader will be run instead. 

The application was developed with GCC and on the Evaluation Kit, PCA10001. The quality should be considered to be a 
proof of concept, nothing more. 

Steps to use
------------
1. Enable notifications on the response characteristic. 
2. Write the Erase app command to the command characteristic. The device will disconnect, then do the erase, and start 
advertising again.
3. Do 1 again.
4. Write the hex file, line by line, split in 20 byte chuncks to the data characteristic. Each line will be acked with a 
response notification. 
5. When all lines have been written and acked, write the Reset and run command. The bootloader will reset and start the 
application. 

Architecture
------------
The bootloader runs in the bottom of code region 1, on top of the S110, and implements assembly wrappers for all 
interrupts. If the bootloader is not currently running, any interrupts will be forwarded to the application, so that 
they can be handled. This gives some extra latency. 

Known issues
------------
- The writing is quite slow. It hasn't been checked in detail why this is, but transmitting a hex file is not an 
efficient solution. It was chosen for the simplicity of the PC-side implementation.
- The application has not been tested very thoroughly, and might very well have issues. 
- As stated above, this has been developed with GCC, and has not been tested or verified at all with Keil. 

Dependencies
------------
This uses my libhexwriter for parsing hex files. See http://github.com/hlnd/libhexwriter/
