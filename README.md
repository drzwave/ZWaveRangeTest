# ZWaveRangeTest

ZWaveRangeTest.py is a command line Python3 script used to determine the RF range of a Z-Wave device.

# Setup
- Hardware
	- PC or Linux based computer (development was done using a Raspberry Pi)
	- UZB or other USB->Z-Wave interface
	- Z-Wave Developers Kit (DevKit)
		- Typically a WSTK with a Zen Gecko Radio board with a 908Mhz antenna
		- Can be any Z-Wave device as long as it supports PowerLevel Command Class
	- Device Under Test (DUT)
	- Open area with at least 200' open space - usually outdoors
	- Non-metalic method to hold the DevKit and DUT off the ground a few feet. A cardboard box works fine.
- Software
	- This program and Python3 installed
- Setup Procedure
1. Setup the PC/RPi with software. These must be located within direct Z-Wave range of the DevKit. 
2. Open a CMD window
3. Start the script - python3 ZWaveRangeTest.py
4. Reset the UZB - use the RST command
5. Join the DevKit and DUT to the PC/RPi (Use the INC command - remember to either Factory Reset the DevKit/DUT or exclude them first)
6. Place the DevKit within direct Z-Wave range of the PC/RPi.
7. Place the DUT initially about 50' away - use batteries or lots of extension cords!
8. Run a RF Range test to ensure the system is working
9. Repeat steps 7 and 8 after placing the DUT further away.
10. Decide what the cutoff you want to use will be for the range - either Full Power or one of the lower settings. When you get less than 5 ACKs then the previous power level is the minimum for that range.
