"""
Copyright (c) 2020 Imagimob AB. Distributed under MIT license.
"""

import serial
import time
import struct
import math
import binascii
import codecs
import numpy as np 
import csv 
import pandas as pd 


def start_TI_radar(filePath):
    global GestureData, FrameIndex
    
    dtype = [('framenumber', np.int16), ('objnumber', np.int16), ('Range', np.float64),
             ('velocity', np.float64), ('peakval', np.float64), ('x', np.float64), ('y', np.float64),  ('z', np.float64)]

    GestureData = np.array([(0, 0, 0, 0, 0, 0,0, 0)], dtype=dtype)

    serialPort_CFG = serial.Serial(   # opening serial CFG_port
        port="COM3",
        baudrate=115200,
        bytesize=8,
        timeout=None,
        stopbits=serial.STOPBITS_ONE
    )

    serialPort_CFG.close()
    time.sleep(0.1)
    serialPort_CFG.open()

    commandsList = []
    
    with open(filePath,"r") as configFile:
            for line in configFile:        
                if line[0] != '%':
                    commandsList.append(line)

    for command in commandsList:
        serialPort_CFG.write(command.encode()) # sending commands to radar encoded as bytes
        time.sleep(0.1)

    print("\nTI radar initialized ! \n")


def stop_TI_radar():

    serialPort_CFG = serial.Serial(   # opening serial CFG_port
        port="COM3",
        baudrate=115200,
        bytesize=8,
        timeout=None,
        stopbits=serial.STOPBITS_ONE
    )

    serialPort_CFG.write(b'sensorStop\n')
    time.sleep(0.1)
    serialPort_CFG.close()

    print("\nTI radar stopped ! \n")


def start_data_stream_TI_radar():

    serialPort_DATA = serial.Serial(  # opening serial DATA_port
        port="COM4",
        baudrate=921600,
        bytesize=8,
        timeout=None,
        stopbits=serial.STOPBITS_ONE
    )

    serialPort_DATA.close()
    time.sleep(0.1)
    serialPort_DATA.open()

    print("\nData stream started... \n")

    return serialPort_DATA



def getUint32(data):
    """!
       This function coverts 4 bytes to a 32-bit unsigned integer.

        @param data : 1-demension byte array  
        @return     : 32-bit unsigned integer
    """ 
    return (data[0] +
            data[1]*256 +
            data[2]*65536 +
            data[3]*16777216)

def getUint16(data):
    """!
       This function coverts 2 bytes to a 16-bit unsigned integer.

        @param data : 1-demension byte array
        @return     : 16-bit unsigned integer
    """ 
    return (data[0] +
            data[1]*256)


def getHex(data):
    """!
       This function coverts 4 bytes to a 32-bit unsigned integer in hex.

        @param data : 1-demension byte array
        @return     : 32-bit unsigned integer in hex
    """ 
    return (binascii.hexlify(data[::-1]))


def checkMagicPattern(data):
    """!
       This function check if data arrary contains the magic pattern which is the start of one mmw demo output packet.  

        @param data : 1-demension byte array
        @return     : 1 if magic pattern is found
                      0 if magic pattern is not found 
    """ 
    found = 0
    if (data[0] == 2 and data[1] == 1 and data[2] == 4 and data[3] == 3 and data[4] == 6 and data[5] == 5 and data[6] == 8 and data[7] == 7):
        found = 1
    return (found)


def parserOnePacket(serialPort_DATA):   # function used to parse 1 data packet transmitted to the DATA_port

    PI = 3.14159265
    headerNumBytes = 40
    MW_found = False
    
    while not MW_found:
        header_MW = serialPort_DATA.read(8)
        if checkMagicPattern(header_MW) == 1:
            MW_found = True
        else:
            print(header_MW)
            MW_found = False
            serialPort_DATA.reset_input_buffer()
            #serialPort_DATA.reset_output_buffer()
            break  

    #if checkMagicPattern(header_MW) == 1:
    if MW_found:    
        header = serialPort_DATA.read(headerNumBytes - 8)
        headerStartIndex = -8
        
        totalPacketNumBytes = getUint32(header[headerStartIndex+12:headerStartIndex+16:1])
        data = serialPort_DATA.read(totalPacketNumBytes - headerNumBytes)

        platform            = getHex(header[headerStartIndex+16:headerStartIndex+20:1])
        frameNumber         = getUint32(header[headerStartIndex+20:headerStartIndex+24:1])
        timeCpuCycles       = getUint32(header[headerStartIndex+24:headerStartIndex+28:1])
        numDetObj           = getUint32(header[headerStartIndex+28:headerStartIndex+32:1])
        numTlv              = getUint32(header[headerStartIndex+32:headerStartIndex+36:1])
        subFrameNumber      = getUint32(header[headerStartIndex+36:headerStartIndex+40:1])

        #if numDetObj <=0:
        #    print("Negative Object", numDetObj)
        #if subFrameNumber > 3:
        #    print("Sub Frame Error", subFrameNumber)
        """
            print("headerStartIndex    = %d" % (headerStartIndex))
            print("totalPacketNumBytes = %d" % (totalPacketNumBytes))
            print("platform            = %s" % (platform)) 
            print("frameNumber         = %d" % (frameNumber)) 
            print("timeCpuCycles       = %d" % (timeCpuCycles))   
            print("numDetObj           = %d" % (numDetObj)) 
            print("numTlv              = %d" % (numTlv))
            print("subFrameNumber      = %d" % (subFrameNumber))
        """
        #tlvStart = headerStartIndex + headerNumBytes
        tlvStart = 0
                                                
        tlvType    = getUint32(data[tlvStart+0:tlvStart+4:1])
        tlvLen     = getUint32(data[tlvStart+4:tlvStart+8:1])       
        offset = 8
        """            
            print("The 1st TLV") 
            print("    type %d" % (tlvType))
            print("    len %d bytes" % (tlvLen))
        """

        #======================================================================================================================================

        detectedX_array = []
        detectedY_array = []
        detectedZ_array = []
        detectedV_array = []
        detectedRange_array = []
        detectedAzimuth_array = []
        detectedElevAngle_array = []
        detectedSNR_array = []
        detectedNoise_array = []

        output_array = []

        if numDetObj <=0:
            detectedX_array.append(0)
            detectedY_array.append(0)
            detectedZ_array.append(0)
            detectedV_array.append(0)
            detectedRange_array.append(0)
            detectedAzimuth_array.append(0)
            detectedElevAngle_array.append(0)
            detectedSNR_array.append(0)
            detectedNoise_array.append(0)
            detectedRange_array.append(0)
            detectedAzimuth_array.append(0)
            detectedElevAngle_array.append(0)

            numDetObj = 1
            tlvType = 0


        if tlvType == 1 and tlvLen < (totalPacketNumBytes - headerNumBytes):#MMWDEMO_UART_MSG_DETECTED_POINTS
                        
            # TLV type 1 contains x, y, z, v values of all detect objects. 
            # each x, y, z, v are 32-bit float in IEEE 754 single-precision binary floating-point format, so every 16 bytes represent x, y, z, v values of one detect objects.    
            
            # for each detect objects, extract/convert float x, y, z, v values and calculate range profile and azimuth                           
            for obj in range(numDetObj):
                # convert byte0 to byte3 to float x value
                x = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset:tlvStart + offset+4:1]),'hex'))[0]

                # convert byte4 to byte7 to float y value
                y = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset+4:tlvStart + offset+8:1]),'hex'))[0]

                # convert byte8 to byte11 to float z value
                z = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset+8:tlvStart + offset+12:1]),'hex'))[0]

                # convert byte12 to byte15 to float v value
                v = struct.unpack('<f', codecs.decode(binascii.hexlify(data[tlvStart + offset+12:tlvStart + offset+16:1]),'hex'))[0]

                # calculate range profile from x, y, z
                compDetectedRange = math.sqrt((x * x)+(y * y)+(z * z))

                # calculate azimuth from x, y           
                if y == 0:
                    if x >= 0:
                        detectedAzimuth = 90
                    else:
                        detectedAzimuth = -90 
                else:
                    detectedAzimuth = math.atan(x/y) * 180 / PI

                # calculate elevation angle from x, y, z
                if x == 0 and y == 0:
                    if z >= 0:
                        detectedElevAngle = 90
                    else: 
                        detectedElevAngle = -90
                else:
                    detectedElevAngle = math.atan(z/math.sqrt((x * x)+(y * y))) * 180 / PI
                        
                detectedX_array.append(x)
                detectedY_array.append(y)
                detectedZ_array.append(z)
                detectedV_array.append(v)
                detectedRange_array.append(compDetectedRange)
                detectedAzimuth_array.append(detectedAzimuth)
                detectedElevAngle_array.append(detectedElevAngle)
                                                            
                offset = offset + 16
            # end of for obj in range(numDetObj) for 1st TLV

        # Process the 2nd TLV
        tlvStart = tlvStart + 8 + tlvLen
                                                
        tlvType    = getUint32(data[tlvStart+0:tlvStart+4:1])
        tlvLen     = getUint32(data[tlvStart+4:tlvStart+8:1])      
        offset = 8
        """            
            print("The 2nd TLV") 
            print("    type %d" % (tlvType))
            print("    len %d bytes" % (tlvLen))
        """                                                    
        if tlvType == 7: 
            
            # TLV type 7 contains snr and noise of all detect objects.
            # each snr and noise are 16-bit integer represented by 2 bytes, so every 4 bytes represent snr and noise of one detect objects.    
        
            # for each detect objects, extract snr and noise                                            
            for obj in range(numDetObj):
                # byte0 and byte1 represent snr. convert 2 bytes to 16-bit integer
                snr   = getUint16(data[tlvStart + offset + 0:tlvStart + offset + 2:1])
                # byte2 and byte3 represent noise. convert 2 bytes to 16-bit integer 
                noise = getUint16(data[tlvStart + offset + 2:tlvStart + offset + 4:1])

                detectedSNR_array.append(snr)
                detectedNoise_array.append(noise)
                                                                
                offset = offset + 4
        else:
            for obj in range(numDetObj):
                detectedSNR_array.append(0)
                detectedNoise_array.append(0)
        # end of if tlvType == 7
        
        
        # print("                  x(m)         y(m)         z(m)        v(m/s)    Com0range(m)  azimuth(deg)  elevAngle(deg)  snr(0.1dB)    noise(0.1dB)")
        for obj in range(numDetObj):
            # print("    obj%3d: %12f %12f %12f %12f %12f %12f %12d %12d %12d" % (obj, detectedX_array[obj], detectedY_array[obj], detectedZ_array[obj], detectedV_array[obj], detectedRange_array[obj], detectedAzimuth_array[obj], detectedElevAngle_array[obj], detectedSNR_array[obj], detectedNoise_array[obj]))
            output_array.append([obj, detectedX_array[obj],\
                                           detectedY_array[obj],\
                                           detectedZ_array[obj],\
                                           detectedV_array[obj],\
                                           detectedRange_array[obj],\
                                           detectedAzimuth_array[obj],\
                                           detectedElevAngle_array[obj],
                                           detectedSNR_array[obj],\
                                           detectedNoise_array[obj]])

        #======================================================================================================================================
        
        print("\n")
        #print("\n Succeeded !!!!!!!!!!! \n")
        #print("End Input Buffer Bytes", serialPort.in_waiting)
        #print("End Output Buffer Bytes", serialPort.out_waiting)
        #serialPort.reset_input_buffer()
    
    else:
        print("\nCORRUPTED FRAME\n")
        #time.sleep(5)
        time.sleep(0.01)  # to avoid double timestamp when corrupted frame occurs (?!?!)
        output_array = [[0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]]
        numDetObj = 1
        #break

    return numDetObj, output_array


#=======================================================================================================================================================================

def save_data(output_array): 
    # print("                  x(m)         y(m)         z(m)        v(m/s)    Com0range(m)  azimuth(deg)  elevAngle(deg)  snr(0.1dB)    noise(0.1dB)")
    header = ["time","x","y","z","v","com","azi","ele","snr","noise"]
    df = pd.DataFrame(output_array)
    df.to_csv('my_csv.csv', index=False, header=header)
    
# Testing functioning of above defined functions 

file_path = "xwr68xx.cfg"
start_TI_radar(file_path)
# time.sleep(10)
time.sleep(2)
serialPort_DATA = start_data_stream_TI_radar()
print(serialPort_DATA)
while 1: 
    output_TI_radar = parserOnePacket(serialPort_DATA)
    numDetObj, output_array = output_TI_radar
    print(output_array)
    save_data(output_array)

stop_TI_radar()

