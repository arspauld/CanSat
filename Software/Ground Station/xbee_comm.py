#from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice #Import the needed classes from the xbee library

#Import the xbee library
import digi.xbee.devices as digi

#Define the local device using the port and baud rate
local_xbee = digi.XBeeDevice('COM9', 115200)

#Define the remote device (64 bit hex address) based off of the local one
remote_xbee = digi.RemoteXBeeDevice(local_xbee, digi.XBee64BitAddress.from_hex_string("13A200410711D9"))

#Opens the local device
local_xbee.open()

#Sends a specific string to the defined device
local_xbee.send_data(remote_xbee, 'hello world')

#Polls the remote xbee for any data
remote_xbee_message = local_xbee.read_data(remote_xbee)

#Closes the local device
local_xbee.close()


#device.open()
#device.send_data_broadcast('Hello World!') ##Used to broadcast to all devices on the network
#device.close()