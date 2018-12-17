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
plot = RealTimePlot(name="Example")
d1.addWidget(plot.plot)

w = QtGui.QWidget()
btn = QtGui.QPushButton('restart plot')
btn.setCheckable(True)
text = QtGui.QLineEdit('enter text')
listw = QtGui.QListWidget()
layout = QtGui.QGridLayout()
w.setLayout(layout)
d2.addWidget(w)

layout.addWidget(btn, 2, 0)   # button goes in upper-left
layout.addWidget(text, 1, 0)   # text edit goes in middle-left
layout.addWidget(listw, 0, 0)  # list widget goes in bottom-left

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
    if btn.isChecked():
        btn.toggle()
        points = 1
        plot.clear()

app.exec_()                                     # Executes application
