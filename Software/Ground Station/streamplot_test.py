import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
from streamplot import PlotManager
import time

length = 1
costs  = np.arange(length)

plt_mgr = PlotManager(
	title="plots",
	nline=3)

for i in range(length):
	cost = costs[i]
	plt_mgr.add("cost", cost)
	plt_mgr.add("time", time.time())
	plt_mgr.add("time2", time.time())
	plt_mgr.update()

plt_mgr.close()
