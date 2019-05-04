import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph import console
from pyqtgraph.dockarea import *
from realplot import RealTimePlot, update
from csv_creation import csv_write
import random
import time
import serial


#   Commands
"""For use with the ground station to control the GUI"""
RESET=0xFF
CALIBRATE=0xEE
CALIBRATE_CAMERA=0xDD
CALIBRATE_ALTITUDE=0xCC
CALIBRATE_ANGLE=0xBB
GPS=0xAA



f = open("Telemetry/Flight_5343.csv", "w+")      # Creates a file for the telemetry data
telem_form = "TEAM ID,MISSION TIME,PACKET COUNT,ALTITUDE,PRESSURE,TEMP,VOLTAGE,GPS TIME,GPS LATITUDE,GPS LONGITUDE,GPS ALTITUDE,GPS SATS,PITCH,ROLL,BLADE SPIN RATE,SOFTWARE STATE,BONUS DIRECTION"
f.write(telem_form)

app = QtGui.QApplication([])                    # Starts the GUI application
win = QtGui.QMainWindow()                       # Creates the main storage for docks
area = DockArea()                               # Creates an area to dock into
win.setCentralWidget(area)                      # Adds the area to the window space
win.resize(1400,800)                            # Sets the window size to 1400 pixels wide by 800 pixels tall
win.setWindowTitle("Ground Station Control")    # Names the window

## Creates Docks and inserts them into the area
altituded  = Dock("Altitude")       # Check realplot.py for documentation
pressured  = Dock("Pressure")       # Creates plotting elements for each telemetry section
tempd      = Dock("Temp")
voltaged   = Dock("Voltage")
pitchd     = Dock("Pitch")
rolld      = Dock("Roll")
spind      = Dock("Blade Spin Rate")
directiond = Dock("Bonus Direction")
gpsd       = Dock("GPS")
messages   = Dock("Messages and Communication")
area.addDock(altituded, 'left')
area.addDock(pressured, 'bottom', altituded)
area.addDock(voltaged, 'bottom', pressured)

area.addDock(tempd, 'right', altituded)
area.addDock(gpsd, 'right', pressured)
area.addDock(pitchd, 'right', voltaged)

area.addDock(directiond, 'right', tempd)
area.addDock(spind, 'right', gpsd)
area.addDock(rolld, 'right', pitchd)

area.addDock(messages, 'right')

## Add Widgets to the docks
# Plots
wid = QtGui.QWidget()                           # Creates an empty Widget to hold a layout
lay = QtGui.QGridLayout()                       # Creates a layout to orgonaize the plots
wid.setLayout(lay)                              # Adds the layout to the container widget

altitude  = RealTimePlot(name="Altitude")       # Check realplot.py for documentation
pressure  = RealTimePlot(name="Pressure")       # Creates plotting elements for each telemetry section
temp      = RealTimePlot(name="Temp")
voltage   = RealTimePlot(name="Voltage")
pitch     = RealTimePlot(name="Pitch")
roll      = RealTimePlot(name="Roll")
spin      = RealTimePlot(name="Blade Spin Rate")
direction = RealTimePlot(name="Bonus Direction")
gps       = RealTimePlot(name="GPS")
altituded.addWidget(altitude.plot)          # Adds the widgets to the layout
pressured.addWidget(pressure.plot)
tempd.addWidget(temp.plot)
voltaged.addWidget(voltage.plot)
pitchd.addWidget(pitch.plot)
rolld.addWidget(roll.plot)
gpsd.addWidget(gps.plot)
spind.addWidget(spin.plot)
directiond.addWidget(direction.plot)


# Buttons
w = QtGui.QWidget()                             # Creates a large widget to hold the others

halt = QtGui.QPushButton('halt plot')            # A checkable box
calibrate = QtGui.QPushButton("Calibrate Payload")
sys_check = QtGui.QPushButton("Check Systems")
reset_btn = QtGui.QPushButton("RESET")

halt.setCheckable(True)                          # Allows for a button to stay pressed
calibrate.setCheckable(True)
sys_check.setCheckable(True)
reset_btn.setCheckable(True)

listw = QtGui.QListWidget()                     # Creates a Display list box

layout2 = QtGui.QGridLayout()                   # Creates a layout element
w.setLayout(layout2)                            # Adds the layout to the large widget
messages.addWidget(w)                                 # Adds the large widget to the dock

layout2.addWidget(reset_btn,    0, 0, 1, 2)     # button goes in row 0, column 0, spanning 1 row and 2 columns
layout2.addWidget(halt,          2, 0, 1, 2)     # text edit goes in middle-left
layout2.addWidget(listw,        1, 0, 1, 2)     # list widget goes in upper-left
layout2.addWidget(calibrate,    3, 0, 1, 1)
layout2.addWidget(sys_check,    3, 1, 1, 1)

# Shows Window
win.show()                                      # Shows window


listw.addItem(telem_form)

"""
#Open the serial port
ser = serial.Serial()
ser.baudrate = 9600
ser.port = 'COM9'
ser.open()
read = True


while True:
    while read:
        binary = ser.readline()
        line = str(binary, encoding='ascii')
        
        line = line.strip()

        datapoints = line.split(',') #This is the actual data that can be written into the .csv file

        print(datapoints)
        #ser.write(b'hello world')

        read = False
    
    while not read:

        altitude.add(datapoints[1],datapoints[3])
        pressure.add(datapoints[1],datapoints[4])
        temp.add(datapoints[1],datapoints[5])
        voltage.add(datapoints[1],datapoints[6])
        #flight.add(datapoints[],datapoints[]) #This graph needs to be different or split into 2, not sure yet
        spin.add(datapoints[1],datapoints[14])
        direction.add(datapoints[1],datapoints[16])
        gps.add(datapoints[9],datapoints[8])

        red = True
ser.close()
"""



## Real Time Graphing (main loop)
points = 1
going = True
while True:
    while going: #Loop for random plotting
        altitude.add(   points-1, random.randint(0, points))   #adds a point x = number of points, y = random integer from 0 to x
        pressure.add(   points-1, random.randint(0, points))
        temp.add(       points-1, random.randint(0, points))
        voltage.add(    points-1, random.randint(0, points))
        pitch.add(      points-1, random.randint(0, points))
        roll.add(       points-1, random.randint(0, points))
        spin.add(       points-1, random.randint(0, points))
        direction.add(  points-1, random.randint(0, points))
        #if points % 10 == 0: listw.addItem(str(points))
        points += 1
        app.processEvents()
        if reset_btn.isChecked():
            listw.addItem("Restarted")
            break

        if halt.isChecked():
            listw.addItem("Ended")
            break

    while going:
        altitude.add(   points-1, random.randint(0, points))   #adds a point x = number of points, y = random integer from 0 to x
        pressure.add(   points-1, random.randint(0, points))
        temp.add(       points-1, random.randint(0, points))
        voltage.add(    points-1, random.randint(0, points))
        pitch.add(      points-1, random.randint(0, points))
        roll.add(       points-1, random.randint(0, points))
        spin.add(       points-1, random.randint(0, points))
        direction.add(  points-1, random.randint(0, points))

    if reset_btn.isChecked():                   # Checks if the button is pressed
        reset_btn.toggle()
        points = 1
        altitude.clear()
        pressure.clear()
        temp.clear()
        voltage.clear()
        pitch.clear()
        roll.clear()
        gps.clear()
        spin.clear()
        direction.clear()


    if halt.isChecked():
        break



#"""
#while True:
#	while going:
#		update(app, altitude, x, y)
#		update(app, pressure, x, y)
#		update(app, temp, x, y)
#		update(app, voltage, x, y)
#		update(app, flight, x, y)
#		update(app, gps, x, y)
#		update(app, spin, x, y)
#		update(app, direction, x, y)
#"""





		
app.exec_()                                     # Executes application








