''' Z-Wave RF Range Test Python script

    This program is a DEMO only and is provided AS-IS and without support. 
    But feel free to copy and improve!

    Usage: python3 ZWaveRangeTest.py [DEVKITNODEID=xx] [DUTNODEID=xx] [COMPORT=/dev/ttyxxxx or COMx] [inc | exc | rst]
    DEVkitnodeid is the NodeID of the Z-Wave DevKit that will send the NOPs to the DUT
    DUTnodeid is the NodeID of the Device Under Test
    COMPORT is the COM port (windows) or /dev/tty* port (linux) of the Z-Wave interface.
    INC/EXC/RST are Z-Wave network management commands and will Include, Exclude a node or Reset the UZB.

    The Range Test utilizes the PowerLevel Command Class to send NOPs at various RF power levels from DEVKIT to DUT.
    PC/RPi>-----PowerLevel Test Node Set----->DEVKIT>-----NOPs---------------------->DUT
                                                                                      |
                                                                                      V
    PC/RPi<-----PowerLevel Test Node Report--<DEVKIT<-----ACKs----------------------<DUT

    The PC/RPi running this script sends Powerlevel Test Node SET commands to the DevKit.
    The DevKit then sends the desired number of NOPs to the DUT at the chosen RF power level.
    The DUT ACKs the NOPs if it hears them.
    The DevKit records the number of ACKs and reports that final number back to the PC/RPi.
    The RF power is adjusted down to find the level where it fails.
    There are only 4 power levels tested as the other levels generally don't make much of a difference.
    FULL
    MINUS4db
    MINUS8db
    MINUS9db (which is really -10dB)

    Only the DevKit needs to support the Powerlevel Command class though all Z-Wave Plus devices are required to support it.
    The DevKit is assumed to be running a relatively new version of one of the always-on sample apps (SwitchOnOff) which
    will automatically send the Power Level Test Report upon completion of the NOPs. 
    The DUT must be awake. The Devkit will send NOPs which are ACKed by the Z-Wave protocol so there is no requirements
    for the DUT application code.

    NOTE NOTE NOTE !!! This test cannot officially be used for Z-Wave Certification of the CER as the NOPs are sent 
    3 times if they are not ACKed. The retries are the normal protocol operation so this is what will happen in typical networks.
    The RailTest procedure listed in INS14283 is the official procedure which will only send the NOP once.
    However, this program can be used to quickly and easily measure the range of any Z-Wave device for basic information purposes.

    Power Level Command Class is in SDS14784 - Z-Wave Network Protocol CC
    SerialAPI: https://www.silabs.com/documents/login/user-guides/INS12350-Serial-API-Host-Appl.-Prg.-Guide.pdf (or search "SerialAPI" on the silabs site)

    Author - Eric Ryherd - drzwave@silabs.com - Still learning Python 3 so there may be some python 2 holdovers...
'''

import serial           # serial port control
import sys
import time
import os
from struct            import * # PACK

VERSION       = "1.2 - 9/3/2019"       # Version of this python program
DEBUG         = 4     # [0-10] higher values print out more debugging info - 0=off

COMPORT     = "/dev/ttyAMA0"    # default COMPORT for the RPi
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
FUNC_ID_ZW_SET_DEFAULT              = 0x42
FUNC_ID_ZW_ADD_NODE_TO_NETWORK      = 0x4A
FUNC_ID_ZW_REMOVE_NODE_FROM_NETWORK = 0x4B
FUNC_ID_ZW_FIRMWARE_UPDATE_NVM      = 0x78

# PowerLevel Command Class defines
COMMAND_CLASS_POWERLEVEL            = 0x73
POWERLEVEL_TEST_NODE_SET            = 0x04
POWERLEVEL_TEST_NODE_GET            = 0x05
POWERLEVEL_TEST_NODE_REPORT         = 0x06
POWERLEVEL_SET_NORMALPOWER          = 0x00
POWERLEVEL_SET_MINUS1DBM            = 0x01
POWERLEVEL_SET_MINUS2DBM            = 0x02
POWERLEVEL_SET_MINUS3DBM            = 0x03
POWERLEVEL_SET_MINUS4DBM            = 0x04
POWERLEVEL_SET_MINUS5DBM            = 0x05
POWERLEVEL_SET_MINUS6DBM            = 0x06
POWERLEVEL_SET_MINUS7DBM            = 0x07
POWERLEVEL_SET_MINUS8DBM            = 0x08
POWERLEVEL_SET_MINUS9DBM            = 0x09

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

ADD_NODE_ANY                    = 0x01
ADD_NODE_CONTROLLER             = 0x02
ADD_NODE_SLAVE                  = 0x03
ADD_NODE_EXISTING               = 0x04
ADD_NODE_STOP                   = 0x05
ADD_NODE_SMART_START            = 0x09
ADD_NODE_OPTION_NETWORK_WIDE    = 0x40
ADD_NODE_OPTION_NORMAL_POWER    = 0x80
ADD_NODE_MODE = ADD_NODE_ANY | ADD_NODE_OPTION_NETWORK_WIDE | ADD_NODE_OPTION_NORMAL_POWER

TRANSMIT_COMPLETE_OK      =0x00
TRANSMIT_COMPLETE_NO_ACK  =0x01 
TRANSMIT_COMPLETE_FAIL    =0x02 
TRANSMIT_ROUTING_NOT_IDLE =0x03

TRANSMIT_OPTION_ACK = 0x01
TRANSMIT_OPTION_AUTO_ROUTE = 0x04
TRANSMIT_OPTION_NO_ROUTE = 0x10
TRANSMIT_OPTION_EXPLORE = 0x20

# Callback states from ZW_AddNodeToNetwork
ADD_NODE_STATUS_LEARN_READY          =1
ADD_NODE_STATUS_NODE_FOUND           =2
ADD_NODE_STATUS_ADDING_SLAVE         =3
ADD_NODE_STATUS_ADDING_CONTROLLER    =4
ADD_NODE_STATUS_PROTOCOL_DONE        =5
ADD_NODE_STATUS_DONE                 =6
ADD_NODE_STATUS_FAILED               =7
ADD_NODE_STATUS_NOT_PRIMARY          =0x23

# SerialAPI defines
SOF = 0x01
ACK = 0x06
NAK = 0x15
CAN = 0x18
REQUEST = 0x00
RESPONSE = 0x01
# Use the normal routing to deliver the SET commands. Don't bother with explorer frames which waste time.
TXOPTS = TRANSMIT_OPTION_AUTO_ROUTE | TRANSMIT_OPTION_ACK

# See INS13954-7 section 7 Application Note: Z-Wave Protocol Versions on page 433
ZWAVE_VER_DECODE = {# Z-Wave version to SDK decoder: https://www.silabs.com/products/development-tools/software/z-wave/embedded-sdk/previous-versions
        b"6.04" : "SDK 6.81.03 01/2019",
        b"6.02" : "SDK 6.81.01 10/2018",
        b"6.01" : "SDK 6.81.00 09/2018",
        b"5.03" : "SDK 6.71.03        ",
        b"5.02" : "SDK 6.71.02 07/2017",
        b"4.61" : "SDK 6.71.01 03/2017",
        b"4.60" : "SDK 6.71.00 01/2017",
        b"4.62" : "SDK 6.61.01 04/2017",  # This is the INTERMEDIATE version?
        b"4.33" : "SDK 6.61.00 04/2016",
        b"4.54" : "SDK 6.51.10 02/2017",
        b"4.38" : "SDK 6.51.09 07/2016",
        b"4.34" : "SDK 6.51.08 05/2016",
        b"4.24" : "SDK 6.51.07 02/2016",
        b"4.05" : "SDK 6.51.06 06/2015 or SDK 6.51.05 12/2014",
        b"4.01" : "SDK 6.51.04 05/2014",
        b"3.99" : "SDK 6.51.03 07/2014",
        b"3.95" : "SDK 6.51.02 05/2014",
        b"3.92" : "SDK 6.51.01 04/2014",
        b"3.83" : "SDK 6.51.00 12/2013",
        b"3.79" : "SDK 6.50.01        ",
        b"3.71" : "SDK 6.50.00        ",
        b"3.35" : "SDK 6.10.00        ",
        b"3.41" : "SDK 6.02.00        ",
        b"3.37" : "SDK 6.01.03        "
        }

class ZWaveRangeTest():
    ''' Z-Wave Range Test '''
    def __init__(self):         # parse the command line arguments and open the serial port
        self.COMPORT=COMPORT
        if DEBUG>3: print("COM Port set to {}".format(self.COMPORT))
        try:
            self.UZB= serial.Serial(self.COMPORT,'115200',timeout=2)
        except:
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
            if DEBUG>5: print("Dumping ",end="")
        while self.UZB.inWaiting(): # purge UART RX to remove any old frames we don't want
            c=self.UZB.read()
            if DEBUG>5: print("{:02X}".format(ord(c)),end="")
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
                print("{:02X}".format(ord(pkt[i])),end="")

    def PrintVersion(self):
        pkt=self.Send2ZWave(pack("B",FUNC_ID_SERIAL_API_GET_CAPABILITIES),True)
        (ver, rev, man_id, man_prod_type, man_prod_type_id, supported) = unpack("!2B3H32s", pkt[1:])
        print("SerialAPI Ver={0}.{1}".format(ver,rev))   # SerialAPI version is different than the SDK version
        print("Mfg={:04X}".format(man_id),end="")
        if man_id==0: 
            print("Silicon Labs")
        else:
            print("")
        print("ProdID/TypeID={0:02X}:{1:02X}".format(man_prod_type,man_prod_type_id))
        pkt=self.Send2ZWave(pack("B",FUNC_ID_ZW_GET_VERSION),True)  # SDK version
        (VerStr, lib) = unpack("!12sB", pkt[1:])
        VersionKey=VerStr[-5:-1]
        if VersionKey in ZWAVE_VER_DECODE:
            print("{} {}".format(VerStr.decode('utf-8'),ZWAVE_VER_DECODE[VersionKey]))
        else:
            print("Z-Wave version unknown = {}".format(VerStr))
        print("Library={} {}".format(lib,libType[lib]))
        pkt=self.Send2ZWave(pack("B",FUNC_ID_SERIAL_API_GET_INIT_DATA),True)
        if pkt!=None and len(pkt)>33:
            print("NodeIDs=",end="")
            for k in [4,28+4]:
                j=pkt[k] # this is the first 8 nodes
                for i in range(0,8):
                    if (1<<i)&j:
                        print("{},".format(i+1+ 8*(k-4)),end="")
            print(" ")

    def usage():
        print("")
        print("Usage: python3 ZWaveRangeTest.py [DEVKITNODEID=xx] [DUTNODEID=xx] [COMPORT=COMxx] [inc|exc|rst]")
        print("Version {}".format(VERSION))
        print("DEVKITNODEID=NodeID in HEX for the Developer Kit Node. This is the node that sends the NOPs")
        print("DUTNODEID=NodeID in HEX for Device Under Test")
        print("COMxx is the Z-Wave UART interface - typically COMxx for windows and /dev/ttyXXXX for Linux")
        print("There are 3 optional commandline arguments for managing the Z-Wave Network")
        print(" inc=Include a node into the network - the NodeID will be printed when complete")
        print(" exc=Exclude a node out of the network")
        print(" rst=Delete the Z-Wave network and start fresh - ZW_SetDefault()")
        print(" Using one of these 3 options does the requested operation and no range testing is done")
        print("")

if __name__ == "__main__":
    ''' Start the app if this file is executed'''

    for i in range(len(sys.argv)):          # first check if the COMPORT has been passed in
        if "COMPORT" in sys.argv[i]:
            temp=sys.argv[i].split("=")
            if len(temp)<2:
                ZWaveRangeTest.usage()
                exit()
            COMPORT=temp[1]

    try:
        self=ZWaveRangeTest()       # open the serial port to the UZB or Z-Wave interface
    except:
        print('error - unable to start program')
        ZWaveRangeTest.usage()
        exit()

    if "inc" in sys.argv:   # The Network Management commands just exit upon the completion of the command
        print("Press button on device to be Included",end="")
        pkt=self.Send2ZWave(pack("!3B",FUNC_ID_ZW_ADD_NODE_TO_NETWORK,ADD_NODE_MODE,0x98),True,timeout=10000)
        if pkt[2]!=0x01:    # try again if we don't get a READY
            pkt=self.Send2ZWave(pack("!3B",FUNC_ID_ZW_REMOVE_NODE_FROM_NETWORK,ADD_NODE_MODE,0x99),True,timeout=10000)
        print(" Now")
        pkt=self.GetZWave(timeout=10000) # might be a while before the user presses the button so extend timeout
        state=0
        while state<5 and len(pkt)>2 and not (pkt[2]==0x05 or pkt[2]==0x06):
            state+=1
            pkt=self.GetZWave()
            if DEBUG>7: print(pkt)
            if len(pkt)>3 and (pkt[2]==ADD_NODE_SLAVE or pkt[2]==ADD_NODE_CONTROLLER):
                print("adding NodeID={}".format(pkt[3]))
        self.Send2ZWave(pack("!3B",FUNC_ID_ZW_ADD_NODE_TO_NETWORK,ADD_NODE_STOP,0x00),timeout=10000)
        exit()
    elif "exc" in sys.argv:
        print("Press button on device to be Excluded",end="")
        pkt=self.Send2ZWave(pack("!3B",FUNC_ID_ZW_REMOVE_NODE_FROM_NETWORK,ADD_NODE_MODE,0x99),True,timeout=10000)
        if pkt[2]!=0x01:    # try again if we don't get a READY
            pkt=self.Send2ZWave(pack("!3B",FUNC_ID_ZW_REMOVE_NODE_FROM_NETWORK,ADD_NODE_MODE,0x99),True,timeout=10000)
        print(" Now")
        pkt=self.GetZWave(timeout=10000) # might be a while before the user presses the button so extend timeout
        state=0
        while state<5 and len(pkt)>2 and not (pkt[2]==0x05 or pkt[2]==0x06):
            state+=1
            pkt=self.GetZWave()
            if DEBUG>7: print(pkt)
            if len(pkt)>3 and (pkt[2]==ADD_NODE_SLAVE or pkt[2]==ADD_NODE_CONTROLLER):
                print("Excluding NodeID={}".format(pkt[3]))
        self.Send2ZWave(pack("!3B",FUNC_ID_ZW_REMOVE_NODE_FROM_NETWORK,ADD_NODE_STOP,0x00),timeout=10000)
        exit()
    elif "rst" in sys.argv:
        print("Resetting Z-Wave network to factory defaults - please wait")
        self.Send2ZWave(pack("!B",FUNC_ID_ZW_SET_DEFAULT),False)
        time.sleep(2)
        #pkt=self.GetZWave()
        #if DEBUG>7: print(pkt)
        self.PrintVersion() # fetch and display various attributes of the Controller
        exit()
    elif "help" in sys.argv or "-help" in sys.argv:
        ZWaveRangeTest.usage()
        self.PrintVersion() # fetch and display various attributes of the Controller
        exit()

    # Parse command line for other options
    for i in range(len(sys.argv)):
        if "DEV" in sys.argv[i]:
            temp=sys.argv[i].split("=")
            if len(temp)!=2:
                print("{} ignored".format(sys.argv[i]))
            else:
                DEVKITNODEID=int(temp[1])
        if "DUT" in sys.argv[i]:
            temp=sys.argv[i].split("=")
            if len(temp)!=2:
                print("{} ignored".format(sys.argv[i]))
            else:
                DUTNODEID=int(temp[1])
    if DEBUG>2: print("DEVKITNODE={}, DUTNODEID={}".format(DEVKITNODEID,DUTNODEID))

    # run a range test

    # First check that things are working at full power then do a more complete test
    # ZW_SEND_DATA(Dest NodeID, Len, data, txopts, funcID)
    # PowerLevelTestNodeSet(TestNodeID to send NOPs to, power_level, CountMSB, CountLSB)
    pkt=self.Send2ZWave(pack("!11B",FUNC_ID_ZW_SEND_DATA, DEVKITNODEID,6, 
    COMMAND_CLASS_POWERLEVEL, POWERLEVEL_TEST_NODE_SET, DUTNODEID, POWERLEVEL_SET_NORMALPOWER, 0, 3, 
    TXOPTS, 44), True)
    if len(pkt)<2 or pkt[1]!=0x01: # unable to deliver the SEND_DATA to the serialAPI - just exit
        print("SerialAPI rejected Z-Wave send {}".format(pkt))
        exit()
    pkt=self.GetZWave() # This is the callback confirming the DevKit ACKed the powerlevel_test_set command
    if len(pkt)<3 or pkt[2]!=0x00:
        print("DevKit did not ACK Z-Wave command. Is DEV={} the correct NodeID?".format(DEVKITNODEID))
        if DEBUG>5: print(pkt)
        exit()
    pkt=self.GetZWave(timeout=10000)     # This should be the report which can take a few seconds
    if len(pkt)!=10 or pkt[9]<1:
        print("Failed to NOP the DUT, please move DUT closer to DEVKIT")
        exit()

    # Able to reach the DUT at full power, so now run a test to determine the lowest RF power at this range

    PowerLevelTests = ( POWERLEVEL_SET_NORMALPOWER, POWERLEVEL_SET_MINUS4DBM, POWERLEVEL_SET_MINUS8DBM, POWERLEVEL_SET_MINUS9DBM) 
    PowerLevelText = { POWERLEVEL_SET_NORMALPOWER: "full", 
    POWERLEVEL_SET_MINUS4DBM: "-4dBm",
    POWERLEVEL_SET_MINUS8DBM: "-8dBm",
    POWERLEVEL_SET_MINUS9DBM: "-10dBm"}
    lastPowerLevel=POWERLEVEL_SET_NORMALPOWER
    lastAck=0
    for powerlevel in PowerLevelTests:
        pkt=self.Send2ZWave(pack("!11B",FUNC_ID_ZW_SEND_DATA, DEVKITNODEID,6, 
        COMMAND_CLASS_POWERLEVEL, POWERLEVEL_TEST_NODE_SET, DUTNODEID, powerlevel, 0, 10, 
        TXOPTS, 33), True)
        pkt=self.GetZWave() # Devkit ACK
        pkt=self.GetZWave(timeout=10000)     # This should be the report which can take a few seconds
        if DEBUG>1 and len(pkt)==10: 
            print("Acks={} at power={}".format(pkt[9],PowerLevelText[powerlevel]))
        if len(pkt)!=10 or pkt[9]<5:       # less than 50% ACK rate is the cutoff for passing or not
            break
        lastPowerLevel=powerlevel
        lastAck=pkt[9]

    print("DUTNodeID={}, ACK={}%, Min Power level={}".format(DUTNODEID,lastAck*10,PowerLevelText[lastPowerLevel]))

    exit()
