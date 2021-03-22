#!/usr/bin/python

import serial, time
import crcmod
   
ser = serial.Serial()
ser.port = "/dev/tty485"
   
#115200,N,8,1
ser.baudrate = 115200
ser.bytesize = serial.EIGHTBITS #number of bits per bytes
ser.parity = serial.PARITY_NONE #set parity check
ser.stopbits = serial.STOPBITS_ONE #number of stop bits
   
ser.timeout = 0.5          #non-block read 0.5s
ser.writeTimeout = 0.5     #timeout for write 0.5s
ser.xonxoff = False    #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False     #disable hardware (DSR/DTR) flow control

def crc(m_input):
    _CRC_FUNC = crcmod.mkCrcFun(poly=0x18005, initCrc=0xffff, rev=0x4b37, xorOut=0x0000)
    
    data = bytearray.fromhex(m_input) #read two rpm
    crc = _CRC_FUNC(data)
    data.append(crc & 0xff)
    data.append(((crc >> 8) & 0xff))

    return data

def send(m_data):
    try:
        ser.flushInput() #flush input buffer
        ser.flushOutput() #flush output buffer
   
        #write 8 byte data
        ser.write(data)
        print("write")
        print(data)
        
        time.sleep(0.2)  #wait 0.2s
   
        #read 8 byte data
        response = ser.read(9)
        print("read byte data:")
        print(response)
        print("---------------------------------------------------------")
    except Exception as e1:
        print ("communicating error " + str(e1))
        
try: 
    ser.open()
except Exception as ex:
    print ("open serial port error " + str(ex))
    exit()
   
if ser.isOpen():


    #data = bytearray.fromhex("010620880064") #set left 100 rpm
    #data = bytearray.fromhex("010320a50002") #read two status, 0000 is ok
    data = crc("010320ab0002") #read two status
    send(data)

    data = crc("0110203000020410000000")#set encoder 4096
    send(data)

    data = crc("0106200d0003")#set speed mode
    send(data)
    data = crc("0106208001F4")#set speed mode
    send(data)    
    data = crc("0106208101F4")#set speed mode
    send(data)
    data = crc("0106208201F4")#set speed mode
    send(data)
    data = crc("0106208301F4")#set speed mode
    send(data)
    data = crc("0106200E0008")#set speed mode
    send(data)
 

    data = crc("010320a50002")#read two status, 0000 is ok
    send(data)


        
    
    cnt=0
    while(cnt<10):
        data = crc("010620880010")#set left 16 rpm
        send(data)   
        cnt+=1
    
    ser.close()

   
