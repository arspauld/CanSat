w = open('Flight_5343.csv','w+')

def csv_write(packet):
    """Writes the packet to the .csv file as required by the dumbass judges"""

    packet.split(',')
    w.write(packet)
    w.write('\n')

headers = 'Team ID,Mission Time,Packet Count,Altitude,Pressure,Temperature,Voltage,GPS Time,GPS Latitude,GPS Longitude,GPS Altitude,GPS Sats,Pitch,Roll,Blade Spin Rate,Flight State,Bonus Direction'
packet='5343,200,12,450.5,100000,25.6,5.0,83.50000,39.50000,-111.2,1000,3,0.1,0.5,60,2,10'

csv_write(headers)
csv_write(packet)

w.close()



"""

team_id = packet[0]  #Team ID
mission_time = packet[1]  #Mission Time
packet_count = packet[2]  #Packet Count
altitude = packet[3]  #Altitude
pressure = packet[4]  #Pressure
temperature = packet[5]  #Temperature
voltage = packet[6]  #Voltage
gps_time = packet[7]  #GPS Time
gps_latitude = packet[8]  #GPS Latitude
gps_longitude = packet[9]  #GPS Longitude
gps_altitude = packet[10]  #GPS Altitude
gps_sats = packet[11]  #GPS Sats
pith = packet[12]  #Pitch
roll = packet[13]  #Roll
blade_spin_rate = packet[14]  #Blade Spin Rate
flight_state = packet[15]  #Flight State
bonus_direction = packet[16]  #Bonus Direction

example:  packet='5343,200,12,450.5,100000,25.6,5.0,83.50000,39.50000,-111.2,1000,3,0.1,0.5,60,2,10'

"""
