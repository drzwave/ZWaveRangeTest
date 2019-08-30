''' Z-Wave RF Range Test Python script

    This program is a DEMO only and is provided AS-IS and without support. 
    But feel free to copy and improve!

    Usage: python3 ZWaveRangeTest.py [DevKitNodeID] [DUTNodeID] [COMx] 
    DevKitNodeID is the NodeID of the Z-Wave DevKit that will send the NOPs to the DUT
    DUTNodeID is the NodeID of the Device Under Test
    COMx is the COM port or /dev/tty* port of the Z-Wave interface.

   SerialAPI: https://www.silabs.com/documents/login/user-guides/INS12350-Serial-API-Host-Appl.-Prg.-Guide.pdf (or search "SerialAPI" on the silabs site)
'''

import serial           # serial port control
import sys
import time
import os
from struct            import * # PACK


COMPORT       = "/dev/ttyAMA0" # Serial port default - typically /dev/ttyACM0 on Linux or COMxx on Windows.

VERSION       = "1.0 - 8/30/2019"       # Version of this python program
DEBUG         = 10     # [0-10] higher values print out more debugging info - 0=off

# By default the DevKit is NodeID 2 and the DUT is NodeID 3 but these can be passed as arguments
DEVKITNODEID = 2
DUTNODEID = 3

# Handy defines mostly copied from ZW_transport_api.py
FUNC_ID_SERIAL_API_GET_INIT_DATA    = 0x02
FUNC_ID_SERIAL_API_APPL_NODE_INFORMATION = 0x03
FUNC_ID_SERIAL_API_GET_CAPABILITIES = 0x07
FUNC_ID_SERIAL_API_SOFT_RESET       = 0x08
FUNC_ID_ZW_GET_PROTOCOL_VERSION     = 0x09
FUNC_ID_SERIAL_API_STARTED          = 0x0A
FUNC_ID_ZW_SET_RF_RECEIVE_MODE      = 0x10
FUNC_ID_ZW_SEND_DATA                = 0x13
FUNC_ID_ZW_GET_VERSION              = 0x15
FUNC_ID_ZW_ADD_NODE_TO_NETWORK      = 0x4A
FUNC_ID_ZW_REMOVE_NODE_FROM_NETWORK = 0x4B
FUNC_ID_ZW_FIRMWARE_UPDATE_NVM      = 0x78

# Z-Wave Library Types
ZW_LIB_CONTROLLER_STATIC  = 0x01
ZW_LIB_CONTROLLER         = 0x02
ZW_LIB_SLAVE_ENHANCED     = 0x03
ZW_LIB_SLAVE              = 0x04
ZW_LIB_INSTALLER          = 0x05
ZW_LIB_SLAVE_ROUTING      = 0x06
ZW_LIB_CONTROLLER_BRIDGE  = 0x07
ZW_LIB_DUT                = 0x08
ZW_LIB_AVREMOTE           = 0x0A
ZW_LIB_AVDEVICE           = 0x0B
libType = {
ZW_LIB_CONTROLLER_STATIC  : "Static Controller",
ZW_LIB_CONTROLLER         : "Controller",
ZW_LIB_SLAVE_ENHANCED     : "Slave Enhanced",
ZW_LIB_SLAVE              : "Slave",
ZW_LIB_INSTALLER          : "Installer",
ZW_LIB_SLAVE_ROUTING      : "Slave Routing",
ZW_LIB_CONTROLLER_BRIDGE  : "Bridge Controller",
ZW_LIB_DUT                : "DUT",
ZW_LIB_AVREMOTE           : "AVREMOTE",
ZW_LIB_AVDEVICE           : "AVDEVICE" }

ADD_NODE_ANY       = 0x01
ADD_NODE_CONTROLLER= 0x02
ADD_NODE_SLAVE     = 0x03
ADD_NODE_EXISTING  = 0x04
ADD_NODE_STOP      = 0x05
ADD_NODE_SMART_START = 0x09
TRANSMIT_COMPLETE_OK      =0x00
TRANSMIT_COMPLETE_NO_ACK  =0x01 
TRANSMIT_COMPLETE_FAIL    =0x02 
TRANSMIT_ROUTING_NOT_IDLE =0x03
TRANSMIT_OPTION_ACK = 0x01
TRANSMIT_OPTION_AUTO_ROUTE = 0x04
TRANSMIT_OPTION_EXPLORE = 0x20
# SerialAPI defines
SOF = 0x01
ACK = 0x06
NAK = 0x15
CAN = 0x18
REQUEST = 0x00
RESPONSE = 0x01
# Most Z-Wave commands want the autoroute option on to be sure it gets thru. Don't use Explorer though as that causes unnecessary delays.
TXOPTS = TRANSMIT_OPTION_AUTO_ROUTE | TRANSMIT_OPTION_ACK

# See INS13954-7 section 7 Application Note: Z-Wave Protocol Versions on page 433
ZWAVE_VER_DECODE = {# Z-Wave version to SDK decoder: https://www.silabs.com/products/development-tools/software/z-wave/embedded-sdk/previous-versions
        "6.04" : "SDK 6.81.03 01/2019",
        "6.02" : "SDK 6.81.01 10/2018",
        "6.01" : "SDK 6.81.00 09/2018",
        "5.03" : "SDK 6.71.03        ",
        "5.02" : "SDK 6.71.02 07/2017",
        "4.61" : "SDK 6.71.01 03/2017",
        "4.60" : "SDK 6.71.00 01/2017",
        "4.62" : "SDK 6.61.01 04/2017",  # This is the INTERMEDIATE version?
        "4.33" : "SDK 6.61.00 04/2016",
        "4.54" : "SDK 6.51.10 02/2017",
        "4.38" : "SDK 6.51.09 07/2016",
        "4.34" : "SDK 6.51.08 05/2016",
        "4.24" : "SDK 6.51.07 02/2016",
        "4.05" : "SDK 6.51.06 06/2015 or SDK 6.51.05 12/2014",
        "4.01" : "SDK 6.51.04 05/2014",
        "3.99" : "SDK 6.51.03 07/2014",
        "3.95" : "SDK 6.51.02 05/2014",
        "3.92" : "SDK 6.51.01 04/2014",
        "3.83" : "SDK 6.51.00 12/2013",
        "3.79" : "SDK 6.50.01        ",
        "3.71" : "SDK 6.50.00        ",
        "3.35" : "SDK 6.10.00        ",
        "3.41" : "SDK 6.02.00        ",
        "3.37" : "SDK 6.01.03        "
        }

class ZWaveRangeTest():
    ''' Z-Wave Range Test '''
    def __init__(self):         # parse the command line arguments and open the serial port
        self.COMPORT=COMPORT
        self.filename=""
        if len(sys.argv)==1:     # No arguments then just print the status if the serial port can be opened
            pass
        elif len(sys.argv)==2: 
            if "COM" in sys.argv[1] or "tty" in sys.argv[1]: # no filename - just check the status
                self.COMPORT=sys.argv[1]
            else:                                           # use the default COMPORT
                self.filename=sys.argv[1]
        elif len(sys.argv)==3:                           # Both comport and filename
            if "COM" in sys.argv[2] or "tty" in sys.argv[2]:
                self.COMPORT=sys.argv[2]
                self.filename=sys.argv[1]
            elif "COM" in sys.argv[1] or "tty" in sys.argv[1]:
                self.COMPORT=sys.argv[1]
                self.filename=sys.argv[2]
        else:
            self.usage()
            sys.exit()
        if DEBUG>3: print("COM Port set to {}".format(self.COMPORT))
        if DEBUG>3: print("Filename set to {}".format(self.filename))
        try:
            self.UZB= serial.Serial(self.COMPORT,'115200',timeout=2)
        except serial.SerialException:
            print("Unable to open serial port {}".format(self.COMPORT))
            exit()

    def checksum(self,pkt):
        ''' compute the Z-Wave SerialAPI checksum at the end of each frame'''
        s=0xff
        for c in pkt:
            #s ^= ord(c)
            s ^= c
        return s

    def GetRxChar( self, timeout=100):
        ''' Get a character from the UART or timeout in 100ms'''
        while timeout >0 and not self.UZB.inWaiting():
            time.sleep(0.001)
            timeout -=1
        if timeout>0:
            retval= self.UZB.read()
        else:
            retval= None
        return retval

    def GetZWave( self, timeout=5000):
        ''' Receive a frame from the UART and return the binary string or timeout in TIMEOUT ms and return None'''
        pkt=b""
        c=self.GetRxChar(timeout)
        if c == None:
            if DEBUG>1: print("GetZWave Timeout!")
            return None
        while ord(c)!=SOF:   # get synced on the SOF
            if DEBUG>5: print("SerialAPI Not SYNCed {:02X}".format(ord(c)))
            c=self.GetRxChar(timeout)
        if ord(c)!=SOF:
            return None
        length=ord(self.GetRxChar())
        for i in range(length):
            c=self.GetRxChar()
            pkt += c
        checksum= self.checksum(pkt)
        checksum ^= length  # checksum includes the length
        if checksum!=0:
            if DEBUG>1: print("GetZWave checksum failed {:02x}".format(checksum))
        self.UZB.write(pack("B",ACK))  # ACK the returned frame - we don't send anything else even if the checksum is wrong
        return pkt[1:-1] # strip off the type and checksum
 
 
    def Send2ZWave( self, SerialAPIcmd, returnStringFlag=False, timeout=5000):
        ''' Send the command via the SerialAPI to the Z-Wave chip and optionally wait for a response.
            If ReturnStringFlag=True then returns a binary string of the SerialAPI frame response within TIMEOUT ms
            else returns None
            Waits 100ms for the ACK/NAK/CAN for the SerialAPI and strips that off. 
            Removes all SerialAPI data from the UART before sending and ACKs to clear any retries.
        '''
        if self.UZB.inWaiting(): 
            self.UZB.write(pack("B",ACK))  # ACK just to clear out any retries
            if DEBUG>5: print("Dumping ",)
        while self.UZB.inWaiting(): # purge UART RX to remove any old frames we don't want
            c=self.UZB.read()
            if DEBUG>5: print("{:02X}".format(ord(c)),)
        frame = pack("2B", len(SerialAPIcmd)+2, REQUEST) + SerialAPIcmd # add LEN and REQ bytes which are part of the checksum
        chksum= self.checksum(frame)
        pkt = (pack("B",SOF) + frame + pack("B",chksum)) # add SOF to front and CHECKSUM to end
        for retries in range(1,4):                        # retry up to 3 times. Z-Wave traffic often causes the UART to lose the SOF and drop the frame.
            if DEBUG>9: print("Sending ", )
            for c in pkt:
                if DEBUG>9: print("{:02X},".format(c),end="")
                self.UZB.write(c.to_bytes(1,byteorder='big'))  # send the command
            if DEBUG>9: print(" ")
            # should always get an ACK/NAK/CAN so wait for it here
            c=self.GetRxChar(500) # wait for the ACK
            if c==None:
                if DEBUG>1: print("no ACK on try #{}".format(retries))
                for i in range(32):
                    self.UZB.write(pack("B",ACK))       # send ACKs to see if the LEN was incorrectly received 
                    if self.UZB.inWaiting(): break      # if we get an ACK/NAK/CAN then stop sending ACKs and retry
            elif ord(c)==ACK:                       # then the frame is OK so no need to retry
                break
            elif ord(c)!=ACK:                       # didn't expect this so just retry
                if DEBUG>1: print("Error - not ACKed = 0x{:02X}".format(ord(c)))
                self.UZB.write(pack("B",ACK))       # send an ACK to try clear out whatever the problem might be
                while self.UZB.inWaiting():         # purge UART RX to remove any old frames we don't want
                    c=self.UZB.read()
        if retries>1 and DEBUG>5:
            print("Took {} tries".format(retries))
        response=None
        if returnStringFlag:    # wait for the returning frame for up to 5 seconds
            response=self.GetZWave(timeout)    
        return response
            

    def RemoveLifeline( self, NodeID):
        ''' Remove the Lifeline Association from the NodeID (integer). 
            Helps eliminate interfering traffic being sent to the controller during the middle of range testing.
        '''
        pkt=self.Send2ZWave(pack("!9B",FUNC_ID_ZW_SEND_DATA, NodeID, 4, 0x85, 0x04, 0x01, 0x01, TXOPTS, 78),True)
        pkt=self.GetZWave(10*1000)
        if pkt==None or ord(pkt[2])!=0:
            if DEBUG>1: print("Failed to remove Lifeline")
        else:
            print("Lifeline removed")
        if DEBUG>10: 
            for i in range(len(pkt)): 
                print("{:02X}".format(ord(pkt[i])),)

    def PrintVersion(self):
        pkt=self.Send2ZWave(pack("B",FUNC_ID_SERIAL_API_GET_CAPABILITIES),True)
        (ver, rev, man_id, man_prod_type, man_prod_type_id, supported) = unpack("!2B3H32s", pkt[1:])
        print("SerialAPI Ver={0}.{1}".format(ver,rev))   # SerialAPI version is different than the SDK version
        print("Mfg={:04X}".format(man_id),)
        if man_id==0: 
            print("Silicon Labs")
        else:
            print("")
        print("ProdID/TypeID={0:02X}:{1:02X}".format(man_prod_type,man_prod_type_id))
        pkt=self.Send2ZWave(pack("B",FUNC_ID_ZW_GET_VERSION),True)  # SDK version
        (VerStr, lib) = unpack("!12sB", pkt[1:])
        VersionKey=VerStr[-5:-1]
        if VersionKey in ZWAVE_VER_DECODE:
            print("{} {}".format(VerStr,ZWAVE_VER_DECODE[VersionKey]))
        else:
            print("Z-Wave version unknown = {}".format(VerStr))
        print("Library={} {}".format(lib,libType[lib]))
        pkt=self.Send2ZWave(pack("B",FUNC_ID_SERIAL_API_GET_INIT_DATA),True)
        if pkt!=None and len(pkt)>33:
            print("NodeIDs=",)
            for k in [4,28+4]:
                j=ord(pkt[k]) # this is the first 8 nodes
                for i in range(0,8):
                    if (1<<i)&j:
                        print("{},".format(i+1+ 8*(k-4)),)
            print(" ")
        pkt=self.Send2ZWave(pack("BB",FUNC_ID_ZW_FIRMWARE_UPDATE_NVM,FIRMWARE_UPDATE_NVM_INIT),True)
        (cmd, FirmwareUpdateSupported) = unpack("!BB", pkt[1:])
        if FirmwareUpdateSupported!=0x01:
            print("Firmware is not OTW capable - exiting {}".format(FirmwareUpdateSupported))
            exit()
        if self.filename=="":        # Skip OTW if no hex file is on the command line
            exit()

    def usage(self):
        print("")
        print("Usage: python3 ZWaveRangeTest.py [DEVKITNODEID=xx] [DUTNODEID=xx] [COMxx]")
        print("Version {}".format(VERSION))
        print("COMxx is the Z-Wave UART interface - typically COMxx for windows and /dev/ttyXXXX for Linux")
        print("")

if __name__ == "__main__":
    ''' Start the app if this file is executed'''
    try:
        self=ZWaveRangeTest()
    except:
        print('error - unable to start program')
        self.usage()
        exit()

    # fetch and display various attributes of the Controller - these are not required
    self.PrintVersion()

    exit()
