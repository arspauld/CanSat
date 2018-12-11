import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph import console
from pyqtgraph.dockarea import *
from realplot import RealTimePlot

app = QtGui.QApplication([])                    # Starts the GUI application
win = QtGui.QMainWindow()                       # Creates the main storage for docks
area = DockArea()                               # Creates an area to dock into
win.setCentralWidget(area)                      # Adds the area to the window space
win.resize(1000,500)                            # Sets the window size to 1000 pixels wide by 500 tall
win.setWindowTitle("Ground Station Control")    # Names the window

## Creates two Docks and inserts them into the area
d1 = Dock("Plot")
d2 = Dock("Console")
area.addDock(d1, 'left')
area.addDock(d2, 'right')

## Add Widgets to the docks
plot = RealTimePlot(name="Example", x_val=np.arange(0,100), y_val=np.random.normal(0, 100, size=100))
d1.addWidget(plot.plot)

console = console.ConsoleWidget()
d2.addWidget(console)

win.show()                                      # Shows window
app.exec_()                                     # Executes application
