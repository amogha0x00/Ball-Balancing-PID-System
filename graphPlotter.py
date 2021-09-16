"""
	Author: Amoghavarsha S G
"""

import numpy as np
from collections import deque
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg


def qtPlotter(setpointPlot, ballPosePlot, plotTerminate, maxLimit=800, size=800, identifier=''):

	app = QtGui.QApplication([])
	timer = QtCore.QTimer()

	win = pg.GraphicsWindow(title=identifier, size=(1000, 900))
	tVec = np.linspace(0, 1, size)

	graph1 = win.addPlot(title='position v/s time')
	graph1.setRange(yRange=[0, maxLimit])
	graph1.addLegend()
	setXVec = deque(np.zeros(size, dtype='uint8'))
	setYVec = deque(np.zeros(size, dtype='uint8'))
	setXcurve = graph1.plot(tVec, setXVec, pen=(0, 0, 255, 255), name="setXcurve")
	setYcurve = graph1.plot(tVec, setYVec, pen=(255, 0, 0, 255), name="setYcurve")

	ballXVec = deque(np.zeros(size, dtype='uint8'))
	ballYVec = deque(np.zeros(size, dtype='uint8'))
	ballXcurve = graph1.plot(tVec, ballXVec, pen=(0, 255, 255, 128), name="ballXcurve")
	ballYcurve = graph1.plot(tVec, ballYVec, pen=(255, 150, 0, 128), name="ballYcurve")

	win.nextRow()
	graph2 = win.addPlot(title='error v/s time')
	graph2.setRange(yRange=[-maxLimit, maxLimit])
	graph2.addLegend()
	errorXVec = deque(np.zeros(size, dtype='uint8'))
	errorYVec = deque(np.zeros(size, dtype='uint8'))
	errorXYVec = deque(np.zeros(size, dtype='uint8'))

	errorXcurve = graph2.plot(tVec, errorXVec, pen='b', name="errorXcurve")
	errorYcurve = graph2.plot(tVec, errorYVec, pen='r', name="errorYcurve")
	errorXYcurve = graph2.plot(tVec, errorYVec, pen='w', name="errorXYcurve")

	def updateGraph():
		setPoseX, setPoseY = setpointPlot
		ballPoseX, ballPoseY = ballPosePlot
		errorX = setPoseX - ballPoseX
		errorY = setPoseY - ballPoseY

		setXVec.popleft()
		setXVec.append(setPoseX)
		setYVec.popleft()
		setYVec.append(setPoseY)
		setXcurve.setData(tVec, setXVec)
		setYcurve.setData(tVec, setYVec)

		ballXVec.popleft()
		ballXVec.append(ballPoseX)
		ballYVec.popleft()
		ballYVec.append(ballPoseY)
		ballXcurve.setData(tVec, ballXVec)
		ballYcurve.setData(tVec, ballYVec)

		errorXVec.popleft()
		errorXVec.append(errorX)
		errorYVec.popleft()
		errorYVec.append(errorY)
		errorXYVec.popleft()
		errorXYVec.append(((errorY)**2 + (errorX)**2)**0.5)
		errorXcurve.setData(tVec, errorXVec)
		errorYcurve.setData(tVec, errorYVec)
		errorXYcurve.setData(tVec, errorXYVec)

		app.processEvents()

		if plotTerminate.value:
			timer.stop()
			win.close()

	timer.timeout.connect(updateGraph)
	timer.start(tVec[1])
	app.exec_()


def matPlotter(setpointPlot, plotTerminate, maxLimit=800, size=100, identifier=''):

	import matplotlib.pyplot as plt

	tVec = np.linspace(0, 1, size)
	yVec = deque(np.zeros(size, dtype='uint8'))
	xVec = deque(np.zeros(size, dtype='uint8'))

	yVecMin = 0
	yVecMax = 0

	plt.style.use('dark_background')
	# this is the call to matplotlib that allows dynamic plotting
	plt.ion()
	fig = plt.figure(figsize=(8,5))
	ax = fig.add_subplot(111)
	# create a variable for the line so we can later update it
	line1, = ax.plot(tVec, yVec, '-b', label='Y')
	line2, = ax.plot(tVec, xVec, '-g', label='X')
	#ax.plot(tVec, np.full(size, 640/2), '-.w', label='YCentre')
	#ax.plot(tVec, np.full(size, 480/2), '-.y', label='XCentre')
	plt.ylim([0, maxLimit])

	#update plot label/title
	plt.ylabel('SETPOINT')
	plt.title('Title: {}'.format(identifier))
	ax.legend()

	plt.show()

	while not plotTerminate.value:
		x, y = setpointPlot
		yVec.popleft()
		yVec.append(y)

		xVec.popleft()
		xVec.append(x)

		# if y < yVecMin:
		# 	yVecMin = y
		# if y > yVecMax:
		# 	yVecMax = y	
		# after the figure, axis, and line are created, we only need to update the y-data
		line1.set_ydata(yVec)
		line2.set_ydata(xVec)

		# adjust limits if new data goes beyond bounds
		# if yVecMin <= line1.axes.get_ylim()[0] or yVecMax >= line1.axes.get_ylim()[1]:
		# 	plt.ylim([yVecMin-np.std(yVec), yVecMax+np.std(yVec)])
		# this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
		plt.pause(0.01)
		#yVec = np.append(yVec[1:], 0)

	plt.close()
