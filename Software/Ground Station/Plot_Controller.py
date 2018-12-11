"""Plot Controller Class used to handle
plotting duties for Ground Station"""
###guided by fmarengo on stackoverflow https://stackoverflow.com/questions/45046239/python-realtime-plot-using-pyqtgraph###
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import random
import time

class RealTimePlot:
    def __init__(self, name, layout, r, c, rspan=1, cspan=1):
        self.x_val = []
        self.y_val = []
        self.plot = pg.PlotWidget(title=name)
        layout.addWidget(self.plot, r, c, rspan, cspan)

    def add(self, x, y):
        self.plot.clear()
        self.x_val.append(x)
        self.y_val.append(y)
        self.plot.plot(self.x_val, self.y_val)

    def clear(self):
        self.plot.clear()
        self.x_val = []
        self.y_val = []


def classtest():
    app = QtGui.QApplication([])            #initializes the GUI
    win = QtGui.QWidget()                   #creates a widget item
    layout = QtGui.QGridLayout()            #creates a grid layout object that can hold plots
    win.setLayout(layout)                   #adds the layout to the
    plot = RealTimePlot(name="Test", layout=layout, r=0, c=0)
    win.show()                              #shows the window

    points = int(input("How many random points?\n"))
    start = time.time()
    for i in range(points):
        plot.add(i, random.randint(0, points))
        app.processEvents()
    end = time.time()
    runtime = end - start
    print("Runtime: %5.2f" %(runtime))

    app.exec_() #execute application


def realtimetest():
    app = QtGui.QApplication([])            #initializes the GUI
    win = QtGui.QWidget()                   #creates a widget item
    layout = QtGui.QGridLayout()            #creates a grid layout object that can hold plots
    win.setLayout(layout)                   #adds the layout to the
    plot = pg.PlotWidget(title="Test")      #creates a plot item
    layout.addWidget(plot, 0, 0)            #adds the widget to row 0, column 0, for 1 space each
    win.show()                              #shows the window

    points = int(input("How many random points?\n"))
    start = time.time()
    for i in range(points):
        update(plot, i, random.randint(0,points,))
    end = time.time()
    runtime = end - start
    print("Runtime: %5.2f" %(runtime))

    app.exec_() #execute application


x_val = []                              #storage of x values
y_val = []                              #storage of y values
def update(plot, x, y):
    plot.clear()                        #empties plot to get rid of old info
    x_val.append(x)                     #adds the new point
    y_val.append(y)
    plot.plot(x_val, y_val)             #adds the point
    app.processEvents()                 #forces the program to update now


if __name__ == "__main__":
    #realtimetest()
    classtest()
