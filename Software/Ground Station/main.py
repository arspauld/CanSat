import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph import console
from pyqtgraph.dockarea import *
from realplot import RealTimePlot
import random
import time

app = QtGui.QApplication([])                    # Starts the GUI application
win = QtGui.QMainWindow()                       # Creates the main storage for docks
area = DockArea()                               # Creates an area to dock into
win.setCentralWidget(area)                      # Adds the area to the window space
win.resize(1000,500)                            # Sets the window size to 1000 pixels wide by 500 tall
win.setWindowTitle("Ground Station Control")    # Names the window

## Creates two Docks and inserts them into the area
d1 = Dock("Plot")
d2 = Dock("Messages and Communication")
area.addDock(d1, 'left')
area.addDock(d2, 'right')

## Add Widgets to the docks
plot = RealTimePlot(name="Example")             # Check realplot.py for documentation
d1.addWidget(plot.plot)                         # Adds the widget to the dock

w = QtGui.QWidget()                             # Creates a large widget to hold the others
btn = QtGui.QPushButton('restart plot')         # A checkable box
btn.setCheckable(True)                          # Allows for a button to stay pressed


listw = QtGui.QListWidget()                     # Creates a Display list box

layout = QtGui.QGridLayout()                    # Creates a layout element
w.setLayout(layout)                             # Adds the layout to the large widget
d2.addWidget(w)                                 # Adds the large widget to the dock

layout.addWidget(btn, 2, 0)                     # button goes in upper-left
layout.addWidget(text, 1, 0)                    # text edit goes in middle-left
layout.addWidget(listw, 0, 0)                   # list widget goes in bottom-left

win.show()                                      # Shows window

## Real Time Graphing
points = 1
going = True
while True:
    while going:
        plot.add(points-1, random.randint(0, points))
        points += 1
        app.processEvents()
        if btn.isChecked():
            listw.addItem("Restarted")
            break
    if btn.isChecked():                         # Checks if the button is pressed
        btn.toggle()
        points = 1
        plot.clear()

app.exec_()                                     # Executes application
