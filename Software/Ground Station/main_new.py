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
import os

#  Open the serial port
ser = serial.Serial()
ser.timeout = 0.02
ser.baudrate = 9600
ser.port = 'COM9'
ser.open()

#  Commands
"""For use with the ground station to control the GUI"""
RESET=0xFF
CALIBRATE=0xEE
CALIBRATE_CAMERA=0xDD
CALIBRATE_ALTITUDE=0xCC
CALIBRATE_ANGLE=0xBB
GPS=0xAA

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
messages   = Dock("Communication")
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

halt_btn = QtGui.QPushButton('HALT')            # A checkable box
calibrate_btn = QtGui.QPushButton("Calibrate Payload")
calibrate_camera_btn = QtGui.QPushButton('Calibrate Camera')
calibrate_altitude_btn = QtGui.QPushButton('Calibrate Altitude')
calibrate_angle_btn = QtGui.QPushButton('Calibrate Angle')
gps_btn = QtGui.QPushButton("GPS")
reset_btn = QtGui.QPushButton("RESET")

halt_btn.setCheckable(True)                          # Allows for a button to stay pressed
calibrate_btn.setCheckable(True)
calibrate_camera_btn.setCheckable(True)
calibrate_altitude_btn.setCheckable(True)
calibrate_angle_btn.setCheckable(True)
gps_btn.setCheckable(True)
reset_btn.setCheckable(True)

listw = QtGui.QListWidget()                    # Creates a Display list box
cmdw = QtGui.QLineEdit()

layout2 = QtGui.QGridLayout()                   # Creates a layout element
w.setLayout(layout2)                            # Adds the layout to the large widget
messages.addWidget(w)                                 # Adds the large widget to the dock

layout2.addWidget(reset_btn,                2, 0, 1, 2)     # button goes in row 0, column 0, spanning 1 row and 2 columns
layout2.addWidget(halt_btn,                 3, 0, 1, 1)     # text edit goes in middle-left
layout2.addWidget(listw,                    0, 0, 1, 2)     # list widget goes in upper-left
layout2.addWidget(cmdw,                     1, 0, 1, 2)
layout2.addWidget(calibrate_camera_btn,     5, 1, 1, 1)
layout2.addWidget(calibrate_altitude_btn,   4, 1, 1, 1)
layout2.addWidget(calibrate_angle_btn,      3, 1, 1, 1)
layout2.addWidget(calibrate_btn,            5, 0, 1, 1)
layout2.addWidget(gps_btn,                  4, 0, 1, 1)

# Shows Window
win.show()  

listw.addItem('Team ID: 5343')

#GUI Functions
def addItem():
    listw.addItem(cmdw.text())
    cmdw.setText('')

def write2payload():
    text = cmdw.text()
    binary = text.encode('ascii')
    ser.write(binary)

#Button Functions
def halt():
    if halt_btn.isChecked():
        timer.stop()
    else:
        timer.start()

def reset():
    altitude.clear()
    pressure.clear()
    temp.clear()
    voltage.clear()
    pitch.clear()
    roll.clear()
    spin.clear()
    direction.clear()
    gps.clear()

    ser.write(b'0xFF')

def calibrate():
    reset()
    
    ser.write(b'0xEE')

def calibrate_camera():
    direction.clear()
    
    ser.write(b'0xDD')

def calibrate_altitude():
    altitude.clear()
    
    ser.write(b'0xCC')

def calibrate_angle():
    pitch.clear()
    roll.clear()
    
    ser.write(b'0xBB')

def get_gps():
    if len(datapoints) > 1:
        coordinates = 'Latitude, Longitude: ' + str(datapoints[8]) + ', ' + str(datapoints[9])
        listw.addItem(coordinates)

        ser.write(b'0xAA')

#Graphing and Data Functions
def graph(datapoints):
    altitude.add(datapoints[1],datapoints[3])
    pressure.add(datapoints[1],datapoints[4])
    temp.add(datapoints[1],datapoints[5])
    voltage.add(datapoints[1],datapoints[6])
    pitch.add(datapoints[1],datapoints[12])
    roll.add(datapoints[1],datapoints[13])
    spin.add(datapoints[1],datapoints[14])
    direction.add(datapoints[1],datapoints[16])
    gps.add(datapoints[9],datapoints[8])

def parse_serial():
    binary = ser.readline()
    line = str(binary, encoding='ascii')
    
    line = line.strip()

    datapoints = line.split(',') #This is the actual data that can be written into the .csv file

    if len(line) > 0:
        listw.addItem(line)
        graph(datapoints)

#Timer
timer = QtCore.QTimer()
timer.setInterval(100)
timer.timeout.connect(parse_serial)
timer.start()

#Connect Signals with Events (Functions)
cmdw.returnPressed.connect(write2payload)
cmdw.returnPressed.connect(addItem)
halt_btn.clicked.connect(halt)
calibrate_btn.clicked.connect(calibrate)
calibrate_camera_btn.clicked.connect(calibrate_camera)
calibrate_altitude_btn.clicked.connect(calibrate_altitude)
calibrate_angle_btn.clicked.connect(calibrate_angle)
gps_btn.clicked.connect(get_gps)
reset_btn.clicked.connect(reset)






#Execute the GUI application
app.exec_()
#Close the serial port
ser.close()