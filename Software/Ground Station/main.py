import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph import console
from pyqtgraph.dockarea import *
from realplot import RealTimePlot
import random
import time

f = open("Telemetry/Flight_0115.csv", "w")      # Creates a file for the telemetry data
telem_form = "TEAM ID,MISSION TIME,PACKET COUNT,ALTITUDE,PRESSURE,TEMP,VOLTAGE,GPS TIME,GPS LATITUDE,GPS LONGITUDE,GPS ALTITUDE,GPS SATS,PITCH,ROLL,BLADE SPIN RATE,SOFTWARE STATE,BONUS DIRECTION"
f.write(telem_form)                             # Creates column headers for the csv file

app = QtGui.QApplication([])                    # Starts the GUI application
win = QtGui.QMainWindow()                       # Creates the main storage for docks
area = DockArea()                               # Creates an area to dock into
win.setCentralWidget(area)                      # Adds the area to the window space
win.resize(1400,800)                            # Sets the window size to 1400 pixels wide by 800 tall
win.setWindowTitle("Ground Station Control")    # Names the window

## Creates two Docks and inserts them into the area
plots = Dock("Plots", size=(1000, 800))
d2 = Dock("Messages and Communication", size=(400, 800))
area.addDock(plots, 'left')
area.addDock(d2, 'right')

## Add Widgets to the docks
# Plots
wid = QtGui.QWidget()                           # Creates an empty Widget to hold a layout
lay = QtGui.QGridLayout()                       # Creates a layout to orgonaize the plots
altitude = RealTimePlot(name="Altitude")        # Check realplot.py for documentation
lay.addWidget(altitude.plot, 0, 0)              # Adds the widget to the layout
pressure = RealTimePlot(name="Pressure")
lay.addWidget(pressure.plot, 0, 1)
temp = RealTimePlot(name="Temp")
lay.addWidget(temp.plot, 1, 0)
voltage = RealTimePlot(name="Voltage")
lay.addWidget(voltage.plot, 1, 1)
pitch = RealTimePlot(name="Pitch")
lay.addWidget(pitch.plot, 2, 0)
roll = RealTimePlot(name="Roll")
lay.addWidget(roll.plot, 2, 1)
spin = RealTimePlot(name="Blade Spin Rate")
lay.addWidget(spin.plot, 3, 0)
direction = RealTimePlot(name="Bonus Direction")
lay.addWidget(direction.plot, 3, 1)

wid.setLayout(lay)                              # Adds the layout to the container widget
plots.addWidget(wid)                            # Adds the container widget to the Dock


#Buttons
w = QtGui.QWidget()                             # Creates a large widget to hold the others
btn1 = QtGui.QPushButton('restart plot')        # A checkable box
btn1.setCheckable(True)                         # Allows for a button to stay pressed
btn2 = QtGui.QPushButton('halt plot')           # Create a second button
btn2.setCheckable(True)

calibrate = QtGui.QPushButton("Calibrate Payload")
calibrate.setCheckable(True)
sys_check = QtGui.QPushButton("Check Systems")
sys_check.setCheckable(True)

listw = QtGui.QListWidget()                     # Creates a Display list box

layout2 = QtGui.QGridLayout()                   # Creates a layout element
w.setLayout(layout2)                            # Adds the layout to the large widget
d2.addWidget(w)                                 # Adds the large widget to the dock

layout2.addWidget(btn1, 2, 0, 1, 2)             # button goes in bottom-left
layout2.addWidget(btn2, 1, 0, 1, 2)             # text edit goes in middle-left
layout2.addWidget(listw, 0, 0, 1, 2)            # list widget goes in upper-left
layout2.addWidget(calibrate, 3, 0, 1, 1)
layout2.addWidget(sys_check, 3, 1, 1, 1)

win.show()                                      # Shows window

## Real Time Graphing
listw.addItem(telem_form)
points = 1
going = True
while True:
    while going:
        altitude.add(points-1, random.randint(0, points))
        points += 1
        app.processEvents()
        if btn1.isChecked():
            listw.addItem("Restarted")
            break

        if btn2.isChecked():
            listw.addItem("Ended")
            break

    if btn1.isChecked():                         # Checks if the button is pressed
        btn1.toggle()
        points = 1
        altitude.clear()

    if btn2.isChecked():
        break

app.exec_()                                     # Executes application
