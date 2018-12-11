from PyQt5 import QtGui
import pyqtgraph as pg
app = QtGui.QApplication([]) #necessary
w = QtGui.QWidget() #Widget item
layout = QtGui.QGridLayout()
w.setLayout(layout)
plot1 = pg.PlotWidget() #plot elements
plot2 = pg.PlotWidget()
plot3 = pg.PlotWidget()
layout.addWidget(plot1, 0, 0, 1, 1)
layout.addWidget(plot2, 0, 1, 1, 1)
layout.addWidget(plot3, 1, 0, 1, 1)
w.show() #show the widget
app.exec_() #execute application