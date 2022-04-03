"""
	Author: Amoghavarsha S G
"""
import sys, signal
import numpy as np
from collections import deque
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg


def qt_plotter(setpoint_plot, ball_pose_plot, pid_plot, plot_terminate, max_limit=800, size=800, identifier=''):

	app = QtGui.QApplication([])
	timer = QtCore.QTimer()

	win = pg.GraphicsWindow(title=identifier, size=(1000, 900))
	t_vec = np.linspace(0, 1, size)

	graph1 = win.addPlot(title='position v/s time')
	graph1.setRange(yRange=[0, max_limit])
	graph1.addLegend()
	set_x_vec = deque(np.zeros(size, dtype='uint8'))
	set_y_vec = deque(np.zeros(size, dtype='uint8'))
	set_x_curve = graph1.plot(t_vec, set_x_vec, pen=(0, 0, 255, 255), name="set_x_curve")
	set_y_curve = graph1.plot(t_vec, set_y_vec, pen=(255, 0, 0, 255), name="set_y_curve")

	ball_x_vec = deque(np.zeros(size, dtype='uint8'))
	ball_y_vec = deque(np.zeros(size, dtype='uint8'))
	ball_x_curve = graph1.plot(t_vec, ball_x_vec, pen=(0, 255, 255, 128), name="ball_x_curve")
	ball_y_curve = graph1.plot(t_vec, ball_y_vec, pen=(255, 150, 0, 128), name="ball_y_curve")

	win.nextRow()
	graph2 = win.addPlot(title='error v/s time')
	graph2.setRange(yRange=[-max_limit, max_limit])
	graph2.addLegend()
	error_x_vec = deque(np.zeros(size, dtype='uint8'))
	error_y_Vec = deque(np.zeros(size, dtype='uint8'))
	error_xy_Vec = deque(np.zeros(size, dtype='uint8'))

	error_x_curve = graph2.plot(t_vec, error_x_vec, pen='b', name="error_x_curve")
	error_y_curve = graph2.plot(t_vec, error_y_Vec, pen='r', name="error_y_curve")
	error_xy_curve = graph2.plot(t_vec, error_y_Vec, pen='w', name="error_xy_curve")

	win.nextRow()
	graph3 = win.addPlot(title='pid v/s time')
	graph3.setRange(yRange=[-max_limit, max_limit])
	graph3.addLegend()
	pid_x_vec = deque(np.zeros(size, dtype='uint8'))
	pid_y_vec = deque(np.zeros(size, dtype='uint8'))

	pid_x_curve = graph3.plot(t_vec, pid_x_vec, pen='b', name="pid_x_vec")
	pid_y_curve = graph3.plot(t_vec, pid_y_vec, pen='r', name="pid_y_vec")

	def updateGraph():
		try:
			set_pose_x, set_pose_y = setpoint_plot
			ball_pose_x, ball_pose_y = ball_pose_plot
			pid_x_value, pid_y_value = pid_plot
			error_x = set_pose_x - ball_pose_x
			error_y = set_pose_y - ball_pose_y

			set_x_vec.popleft()
			set_x_vec.append(set_pose_x)
			set_y_vec.popleft()
			set_y_vec.append(set_pose_y)
			set_x_curve.setData(t_vec, set_x_vec)
			set_y_curve.setData(t_vec, set_y_vec)

			ball_x_vec.popleft()
			ball_x_vec.append(ball_pose_x)
			ball_y_vec.popleft()
			ball_y_vec.append(ball_pose_y)
			ball_x_curve.setData(t_vec, ball_x_vec)
			ball_y_curve.setData(t_vec, ball_y_vec)

			error_x_vec.popleft()
			error_x_vec.append(error_x)
			error_y_Vec.popleft()
			error_y_Vec.append(error_y)
			error_xy_Vec.popleft()
			error_xy_Vec.append(((error_y)**2 + (error_x)**2)**0.5)
			error_x_curve.setData(t_vec, error_x_vec)
			error_y_curve.setData(t_vec, error_y_Vec)
			error_xy_curve.setData(t_vec, error_xy_Vec)

			pid_x_vec.popleft()
			pid_x_vec.append(pid_x_value)
			pid_y_vec.popleft()
			pid_y_vec.append(pid_y_value)
			pid_x_curve.setData(t_vec, pid_x_vec)
			pid_y_curve.setData(t_vec, pid_y_vec)


			app.processEvents()

			if plot_terminate.value:
				timer.stop()
				win.close()
		except KeyboardInterrupt:
				plot_terminate.value = 1
				timer.stop()
				win.close()

	timer.timeout.connect(updateGraph)
	timer.start(t_vec[1])
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	sys.exit(app.exec_())

# def matPlotter(setpoint_plot, plot_terminate, max_limit=800, size=100, identifier=''):

# 	import matplotlib.pyplot as plt

# 	t_vec = np.linspace(0, 1, size)
# 	yVec = deque(np.zeros(size, dtype='uint8'))
# 	xVec = deque(np.zeros(size, dtype='uint8'))

# 	yVecMin = 0
# 	yVecMax = 0

# 	plt.style.use('dark_background')
# 	# this is the call to matplotlib that allows dynamic plotting
# 	plt.ion()
# 	fig = plt.figure(figsize=(8,5))
# 	ax = fig.add_subplot(111)
# 	# create a variable for the line so we can later update it
# 	line1, = ax.plot(t_vec, yVec, '-b', label='Y')
# 	line2, = ax.plot(t_vec, xVec, '-g', label='X')
# 	#ax.plot(t_vec, np.full(size, 640/2), '-.w', label='YCentre')
# 	#ax.plot(t_vec, np.full(size, 480/2), '-.y', label='XCentre')
# 	plt.ylim([0, max_limit])

# 	#update plot label/title
# 	plt.ylabel('SETPOINT')
# 	plt.title('Title: {}'.format(identifier))
# 	ax.legend()

# 	plt.show()

# 	while not plot_terminate.value:
# 		x, y = setpoint_plot
# 		yVec.popleft()
# 		yVec.append(y)

# 		xVec.popleft()
# 		xVec.append(x)

# 		# if y < yVecMin:
# 		# 	yVecMin = y
# 		# if y > yVecMax:
# 		# 	yVecMax = y	
# 		# after the figure, axis, and line are created, we only need to update the y-data
# 		line1.set_ydata(yVec)
# 		line2.set_ydata(xVec)

# 		# adjust limits if new data goes beyond bounds
# 		# if yVecMin <= line1.axes.get_ylim()[0] or yVecMax >= line1.axes.get_ylim()[1]:
# 		# 	plt.ylim([yVecMin-np.std(yVec), yVecMax+np.std(yVec)])
# 		# this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
# 		plt.pause(0.01)
# 		#yVec = np.append(yVec[1:], 0)

# 	plt.close()
