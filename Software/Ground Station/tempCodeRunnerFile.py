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