import serial

ser = serial.Serial()
ser.baudrate = 9600
ser.port = 'COM9'
print(ser)
ser.open()
while True:
    binary = ser.readline()
    line = str(binary, encoding='ascii')
    
    line = line.strip()

    datapoints = line.split(',')

    print(datapoints)
    #ser.write(b'hello world')
    
ser.close()



#### MAKE INTO LIST AND HAVE IT RETURN THE PARSED "datapoints"
######## MAKE SURE TO PASS THROUGH THE ser ITEMS





"""

#from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice #Import the needed classes from the xbee library

#Import the xbee library
import digi.xbee.devices as digi

#Define the local device using the port and baud rate
local_xbee = digi.XBeeDevice('COM9', 9600)

#Define the remote device (64 bit hex address) based off of the local one
remote_xbee = digi.RemoteXBeeDevice(local_xbee, digi.XBee64BitAddress.from_hex_string('13A2004105F9B1'))

#Opens the local device
local_xbee.close()
local_xbee.open()
while True:
    #local_xbee.send_data_broadcast('Hello World!')

#Sends a specific string to the defined device
    #local_xbee.send_data(remote_xbee, 'hello world')

#Polls the remote xbee for any data
    remote_xbee_message = local_xbee.read_data(1000)
    if remote_xbee_message is not None:
        print(remote_xbee_message.data)

#Closes the local device
local_xbee.close()

#print(remote_xbee_message)


#device.open()
#device.send_data_broadcast('Hello World!') ##Used to broadcast to all devices on the network
#device.close()


"""