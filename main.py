#!/usr/bin/env python3

"""
	Author : Amoghavarsha S G
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import threading
import serial
from time import sleep, perf_counter
import io
import math
from multiprocessing import Process, Array, Value
import traceback

###### Written in this Folder ######
from PID import PID
from graphPlotter import qtPlotter as plotGraph


class ServoController:

	def __init__(self, xOffset=1570, yOffset=1420, seperator='#', stopByte='$', platePiviotLen=8, servoArmLen=3.2, cameraDistance=45):
		"""
			Initialize a new Servo Controller
			:param platePiviotLen: Distance between connected servo rod and piviot
			:param servoArmLen: Length of the servo arm not the push rod
		"""
		self.arduinoConnected = False
		self.xOffset = xOffset
		self.yOffset = yOffset
		self.seperator = seperator
		self.stopByte = stopByte
		self.platePiviotLen = platePiviotLen
		self.servoArmLen = servoArmLen
		self.cameraDistance = cameraDistance
		self.xOffsetAngle = self.angleMap(xOffset)
		self.yOffsetAngle = self.angleMap(yOffset)
		self.xPlateAngle = 0
		self.yPlateAngle = 0
		self.frameCenter = ()
		self.ballPose = ()

		try:
			self.arduino = serial.Serial(port='COM3', baudrate=115200, write_timeout=0.01)
			self.arduinoConnected = True
		except serial.SerialException:
			print(" Unable to Open Serial Port ".center(50, '■'), end='\n\n')

	def sendArduino(self, xPid=0, yPid=0):
		xServoUs = int(xPid + self.xOffset)
		yServoUs = int(yPid + self.yOffset)
		#print(f"xPid: {xServoUs} yPid: {yServoUs}")
		
		if self.arduinoConnected:
			cmd = f"{xServoUs}{self.seperator}{yServoUs}{self.stopByte}"
			self.arduino.write(cmd.encode('utf-8'))
			self.arduino.flush()

		servoXangle = self.angleMap(xServoUs) - self.xOffsetAngle
		servoYangle = self.angleMap(yServoUs) - self.yOffsetAngle
		xDisplacement = self.servoArmLen * math.sin(servoXangle)
		yDisplacement = self.servoArmLen * math.sin(servoYangle)
		self.xPlateAngle = math.asin(xDisplacement/self.platePiviotLen)
		self.yPlateAngle = math.asin(yDisplacement/self.platePiviotLen)

	def errorMap(self, error, axis):
		if not self.frameCenter:
			return error
		if axis == "x":
			rError = self.frameCenter[0] - self.ballPose[0] 
			rPlateAngle = self.yPlateAngle
			otherError = abs((self.frameCenter[1] - self.ballPose[1])*math.sin(self.yPlateAngle))/self.cameraDistance
			ownError = error/math.cos(self.xPlateAngle)
		else:
			rError = self.frameCenter[1] - self.ballPose[1]
			rPlateAngle = self.xPlateAngle
			otherError = abs((self.frameCenter[0] - self.ballPose[0])*math.sin(self.xPlateAngle))/self.cameraDistance
			ownError = error/math.cos(self.yPlateAngle)
		if error == 0:
			return error
		if rPlateAngle > 0:
			if rError > 0:
				error_ = ownError #+ ((error/abs(error)) * otherError)
			else:
				error_ = ownError #+  ((error/abs(error)) * - otherError)
		else:
			if rError > 0:
				error_ = ownError #+  ((error/abs(error)) * - otherError)
			else:
				error_ = ownError #+ ((error/abs(error)) * otherError)
		
		#print(self.xPlateAngle*180/math.pi, self.yPlateAngle*180/math.pi, error, error_, axis)
		return error_

	@staticmethod
	def angleMap(val):
		if val <= 2500 and val >= 500:
			return 0 + ( (val-500) * ( (math.pi - 0)/(2500-500) ))
		return 0

	def closeSerialPort(self):
		if self.arduinoConnected:
			self.arduino.close()
			print(" Closed Serial Port ".center(50, '■'))


class SetDispController(threading.Thread):

	def __init__(self):
		super().__init__()
		self.lock = threading.Lock()
		self.setLock = threading.Lock()
		self._frame = None
		self._ballPose = ()
		self.lastId = -1
		self.mode = 0
		self.twoPi = 6.28
		self.r = 200
		self.f = 0.1
		self.graphShown = 0
		self.rad = 0
		self.frameCenter = ()
		self._setpoints = ()
		self.terminated = 0
		self.daemon = True
		if showInterface:
			self.start()

	def run(self):
		global done
		while self.frame is None and not self.terminated:
			sleep(0.01)
		if self.terminated:
			return

		cv2.namedWindow('Processed Video Feed', cv2.WINDOW_NORMAL)
		height, width = self.frame.shape[:2]
		cv2.resizeWindow('Processed Video Feed', int(width*1.1), int(height*1.1))
		cv2.setMouseCallback('Processed Video Feed', self.setSetpoints)
		if DEBUG:
			cv2.createTrackbar('xKp(x100)', 'Processed Video Feed', 0, 500, lambda x: self.setTuning(x/100, "xKp"))
			cv2.createTrackbar('xKi(x100)', 'Processed Video Feed', 0, 100, lambda x: self.setTuning(x/100, "xKi"))
			cv2.createTrackbar('xKd(x100)', 'Processed Video Feed', 0, 1500, lambda x: self.setTuning(x/100, "xKd"))
			xKp, xKi, xKd = xAxisPid.tunings
			cv2.setTrackbarPos('xKp(x100)', 'Processed Video Feed', int(xKp*100))
			cv2.setTrackbarPos('xKi(x100)', 'Processed Video Feed', int(xKi*100))
			cv2.setTrackbarPos('xKd(x100)', 'Processed Video Feed', int(xKd*100))

			cv2.createTrackbar('yKp(x100)', 'Processed Video Feed', 0, 500, lambda x: self.setTuning(x/100, "yKp"))
			cv2.createTrackbar('yKi(x100)', 'Processed Video Feed', 0, 100, lambda x: self.setTuning(x/100, "yKi"))
			cv2.createTrackbar('yKd(x100)', 'Processed Video Feed', 0, 1500, lambda x: self.setTuning(x/100, "yKd"))
			yKp, yKi, yKd = yAxisPid.tunings
			cv2.setTrackbarPos('yKp(x100)', 'Processed Video Feed', int(yKp*100))
			cv2.setTrackbarPos('yKi(x100)', 'Processed Video Feed', int(yKi*100))
			cv2.setTrackbarPos('yKd(x100)', 'Processed Video Feed', int(yKd*100))

		while not self.terminated:
			frame = self.frame # to reduce lock time
			ballPose = self.ballPose

			if self.mode == 1: # draw circle mode
				self.setAllSetpoints(self.getCircleCorr())
				cv2.circle(frame, self.frameCenter, self.r ,(255,255,255),1)
			elif self.mode == 2: # draw 8 mode
				self.setAllSetpoints(self.get8Corr())
				cv2.circle(frame, (self.frameCenter[0]-self.r, self.frameCenter[1]), self.r ,(255,255,255),1)
				cv2.circle(frame, (self.frameCenter[0]+self.r, self.frameCenter[1]), self.r ,(255,255,255),1)

			cv2.circle(frame, self.setpoints, 4 ,(0, 255, 0), -1)

			if ballPose: # draw circle around the ball and arrowed line b/w ball and setpoint
				cv2.circle(frame, ballPose[0], ballPose[1], (0, 255, 255), 2)
				cv2.arrowedLine(frame, ballPose[0], self.setpoints, (0, 0, 255), 2)

			cv2.imshow('Processed Video Feed', frame)
			key = cv2.waitKey(1) & 0xFF

			if key == ord('q'):
				done = 1
				self.terminated = 1
				plotTerminate.value = 1
			elif key == ord('o'):
				self.mode = 1
				self.r = 100
				self.setAllSetpoints(self.getCircleCorr(reset=True))
			elif key == ord('8'):
				self.mode = 2
				self.r = 90
				self.setAllSetpoints(self.get8Corr(reset=True))
			elif key == ord('r'):
				self.mode = 0
				self.setAllSetpoints(self.frameCenter)
			elif key == ord('g'):
				if self.graphShown:
					plotTerminate.value = 1
					self.graphShown = 0
				else:
					self.graphShown = 1
					plotTerminate.value = 0
					args = (setpointPlot, ballPosePlot, plotTerminate)
					kwargs = {'identifier': 'GRAPH', 'maxLimit': (max(width, height)) + 50}
					Process(target=plotGraph, args=args, kwargs=kwargs).start()

			elif key == ord('f'):
				self.f *= -1
			elif key == ord('-'):
				if self.f >= 0:
					self.f -= 0.1
				else:
					self.f += 0.1
			elif key == ord('+'):
				if -3 < self.f < 3:
					if self.f >= 0:
						self.f += 0.1
					else:
						self.f -= 0.1

		cv2.destroyAllWindows()
		cv2.waitKey(1)

	def getCircleCorr(self, radOffset=0, reset=False, xCenterOffset=0, yCenterOffset=0):
		#if -self.twoPi < self.rad < self.twoPi:
		#else:
		if not (-self.twoPi + radOffset < self.rad < self.twoPi + radOffset) or reset:
			self.initialTime = perf_counter()
			self.rad = radOffset
		else:
			self.rad = self.twoPi*self.f*(perf_counter() - self.initialTime) + radOffset

		centre = self.frameCenter[0] + xCenterOffset, self.frameCenter[1] + yCenterOffset
		return int( centre[0] + self.r*np.cos(self.rad)), int(centre[1] + self.r*np.sin(self.rad))

	def get8Corr(self, reset=False):
		if reset:
			self.step = 0
			self.f = 0.1
		if not self.step:
			if -self.twoPi < self.rad < self.twoPi:
				x, y = self.getCircleCorr(reset=reset, xCenterOffset=-self.r)
			else:
				self.step = 1
				self.f *= -1
				x, y = self.getCircleCorr(reset=True, radOffset=self.twoPi/2, xCenterOffset=self.r)
		else:
			if -self.twoPi/2  < self.rad < self.twoPi*3/2:
				x, y = self.getCircleCorr(reset=reset, radOffset=self.twoPi/2, xCenterOffset=self.r)
			else:
				self.step = 0
				self.f *= -1
				x, y = self.getCircleCorr(reset=True, xCenterOffset=-self.r)
		return x, y

	@property
	def frame(self):
		with self.lock:
			return self._frame

	@property
	def ballPose(self):
		with self.lock:
			return self._ballPose

	@property
	def setpoints(self):
		with self.setLock:
			return self._setpoints

	def setAllSetpoints(self, coordinate):
		with self.setLock:
			self._setpoints = coordinate
		xAxisPid.setpoint, yAxisPid.setpoint = coordinate
		setpointPlot[:] = coordinate

	def setFrameNpose(self, frame, ballPose):
		with self.lock:
			self._frame = frame
			self._ballPose = ballPose
			if not self.frameCenter:
				self.frameCenter = (frame.shape[1]//2, frame.shape[0]//2)
				servoCtrl.frameCenter = self.frameCenter
				self.setAllSetpoints(self.frameCenter)

	def setSetpoints(self, event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self.mode = 0
			self.setAllSetpoints((x, y))

	def setTuning(self, pos, parameter):
		if parameter == "xKp":
			xAxisPid.Kp = pos
		elif parameter == "yKp":
			yAxisPid.Kp = pos
		elif parameter == "xKi":
			xAxisPid.Ki = pos
		elif parameter == "yKi":
			yAxisPid.Ki = pos
		elif parameter == "xKd":
			xAxisPid.Kd = pos
		elif parameter == "yKd":
			yAxisPid.Kd = pos

	def checkId(self, currentId):
		if self.lastId > currentId:
			return False
		self.lastId = currentId
		return True


class ImageProcessor(threading.Thread):
	def __init__(self, mat):
		super().__init__()
		self.id = perf_counter()
		self.ballPose = ()
		self.stream = io.BytesIO()
		self.event = threading.Event()
		self.terminated = False
		#self.mat = cv2.getPerspectiveTransform(np.float32([[0,0], [449,0], [449,449], [0,449]]), np.float32([[0,0], [449,0], [449,449], [0,449]]))
		self.mat = mat
		self.lowerLimits = np.array([0, 150, 130])
		self.upperLimits = np.array([32, 255, 255])
		self.kernelOpen = np.ones((5, 5))
		self.kernelClose = np.ones((5, 5))
		self.frame = None
		self.daemon = True
		self.start()

	def run(self):
		# This method runs in a separate thread
		global numNoBallFrames
		while not self.terminated:
			# Wait for an image to be written to the stream
			if self.event.wait(1):
				try:
					if self.mat:
						self.frame = self.frame[self.mat[1][0] : self.mat[1][1], self.mat[0][0] : self.mat[0][1]]
					#self.frame = cv2.resize(self.frame, (480, 640))
					#self.stream.seek(0)
					#self.frame = cv2.imdecode(np.frombuffer(stream.read(), np.uint8), cv2.IMREAD_COLOR)
					self.findBall()
					with idLock:
						if setdispCtrl.checkId(self.id):
							if self.ballPose:
								servoCtrl.ballPose = self.ballPose[0]
								xPid = xAxisPid(self.ballPose[0][0])
								yPid = yAxisPid(self.ballPose[0][1])
								servoCtrl.sendArduino(xPid, yPid)
								ballPosePlot[:] = self.ballPose[0]
								numNoBallFrames = 0
							else:
								if numNoBallFrames >=60: # if Ball is not there for more than 60 frames make plate flat
									servoCtrl.sendArduino(0, 0)
									numNoBallFrames = 0
									xAxisPid.reset(withI=False)
									yAxisPid.reset(withI=False)
								else:
									numNoBallFrames +=1
							#if showInterface:
							setdispCtrl.setFrameNpose(self.frame, self.ballPose)
						else:
							fps.droppedFrames += 1

				except serial.SerialTimeoutException:
					print("Data Not Sent - TIMEOUT!!!!!")
				except Exception as e:
					print(e)
					print(traceback.format_exc())
				finally:
					# Reset the stream and event
					#self.stream.seek(0)
					#self.stream.truncate()
					self.event.clear()
					fps.pTimeUpdate((perf_counter() - self.id)*1000)
					# Return ourselves to the processorPool
					with poolLock:
						processorPool.append(self)

	def findBall(self):

		hsvFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsvFrame, self.lowerLimits, self.upperLimits)

		maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernelOpen)
		maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, self.kernelClose)
		contours, _ = cv2.findContours(maskClose, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		#self.frame = maskClose
		if contours:
			ball = max(contours, key = cv2.contourArea)
			if cv2.contourArea(ball) <= 0:
				self.ballPose = ()
				return False
			((x,y), radius) = cv2.minEnclosingCircle(ball)
			if radius < 12 or radius > 60:
				self.ballPose = ()
				return False			
			self.ballPose = ((int(x), int(y)), int(radius))

			#cv2.drawContours(self.frame, ball, -1, (255,255,255), 3)
			#M = cv2.moments(ball)
			#self.ballPose = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			#print(self.ballPose)
			return True
		self.ballPose = ()#((320, 240),10)
		return False


class FPS():
	def __init__(self, avgFrames=250):
		self.nFrames = 0
		self.avgFrames = avgFrames
		self.pTimeVals = []
		self.initialTime = perf_counter()
		self._droppedFrames = 0

		self.pLock = threading.Lock()
		self.dLock = threading.Lock()

	def fpsUpdate(self):
		self.nFrames += 1

	def getFPS(self):
		return int(self.nFrames/(perf_counter() - self.initialTime))

	def pTimeUpdate(self, pTime):
		with self.pLock:
			self.pTimeVals.append(pTime)

	def getpTime(self):
		with self.pLock:
			return round(sum(self.pTimeVals)/len(self.pTimeVals), 4), round(max(self.pTimeVals), 4)

	def ready(self):
		return self.nFrames >= self.avgFrames

	@property
	def droppedFrames(self):
		with self.dLock:
			return self._droppedFrames

	@droppedFrames.setter
	def droppedFrames(self, droppedFrames):
		with self.dLock:
			self._droppedFrames = droppedFrames

	def reset(self):
		with self.pLock:
			self.pTimeVals = []
		self.droppedFrames = 0
		self.nFrames = 0
		self.initialTime = perf_counter()

def findTable(cap, idsPresent):
	shown = 0
	mat = None
	threshold = 5
	while cap.isOpened():
		ret, frame = cap.read()
		grey_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		upper_bound = (threshold, threshold, threshold)
		grey_frame = 255 - cv2.inRange(frame, (0, 0, 0), upper_bound)
		aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
		arucoParameters = aruco.DetectorParameters_create()
		corners, ids, rejectedImgPoints = aruco.detectMarkers(grey_frame, aruco_dict, parameters=arucoParameters)
		threshold = (threshold + 1) % 256
		if np.in1d(idsPresent, ids).all():
			# if ids is not None:
			# 	print(ids[:, 0])
#		else:
			frame = aruco.drawDetectedMarkers(frame, corners, ids)
			ids = list(ids[:, 0])
			#print(ids)
			index = []
			for _id in idsPresent:
				index.append(ids.index(_id))

			TL = [corners[index[0]][0][0][0],corners[index[0]][0][0][1]]
			TR = [corners[index[1]][0][1][0],corners[index[1]][0][1][1]]
			BR = [corners[index[2]][0][2][0],corners[index[2]][0][2][1]]
			BL = [corners[index[3]][0][3][0],corners[index[3]][0][3][1]]
			points = np.array([TL, TR, BR, BL], dtype="int32")
			mat = [[min(points[:, 0]), max(points[:, 0])], [min(points[:, 1]), max(points[:, 1])]]
		if mat:
			frame = frame[mat[1][0] : mat[1][1], mat[0][0] : mat[0][1]]
		cv2.imshow('table', frame)
		key = cv2.waitKey(1) & 0xFF
		if key == ord('q'):
			break
	cv2.destroyAllWindows()
	return mat

if __name__ == '__main__':

	# !!!!!!!!!!!!!!! Configs !!!!!!!!!!!!!!!!!
	DEBUG = True
	videoSrc = 0
	showInterface = 1
	numImageProcessors = 4

	numNoBallFrames = 0
	poolLock = threading.Lock()
	idLock = threading.Lock()
	fps = FPS()

	cap = cv2.VideoCapture(videoSrc)
	cap.set(3, 640)
	cap.set(4, 480)
	mat = findTable(cap, [0, 1, 2, 3])

	xAxisPid = PID("x", Kp=1.70, Ki=0.08, Kd=1.26)
	xAxisPid.output_limits = -750, 750

	yAxisPid = PID("y", Kp=1.85, Ki=0.1, Kd=1.36)
	yAxisPid.output_limits = -750, 750

	xAxisPid.sample_time, yAxisPid.sample_time = 1/32, 1/32

	done = 0
	plotTerminate = Value('i', 0)
	setpointPlot = Array('i', [0, 0])
	ballPosePlot = Array('i', [0, 0])

	servoCtrl = ServoController()
	xAxisPid.error_map = yAxisPid.error_map = servoCtrl.errorMap

	processorPool = [ImageProcessor(mat) for _ in range(numImageProcessors)]
	setdispCtrl = SetDispController()
	allThreads = processorPool[::-1]
	if showInterface:
		allThreads.insert(0, setdispCtrl)

	isStarved = 'N'
	try:
		while cap.isOpened() and not done:
			with poolLock:
				if processorPool:
					processor = processorPool.pop()
				else:
					processor = None
			if processor:
				ret, processor.frame = cap.read()
				if not ret:
					break
				processor.id = perf_counter()
				processor.event.set()
				st_time = perf_counter()
				fps.fpsUpdate()
				if fps.ready():
					print(f"\rFPS: {fps.getFPS()} | Ptime: {fps.getpTime()} ms | fDropped: {fps.droppedFrames}/{fps.avgFrames} | {isStarved}".ljust(65), end='')
					fps.reset()
					isStarved = 'N'
			else:
				# When the processorPool is isStarved, wait a while for it to refill
				isStarved = 'Y'
				sleep(0.01)
	except KeyboardInterrupt:
		pass

	cap.release()
	plotTerminate.value = 1
	print()
	servoCtrl.closeSerialPort()

	for thread in allThreads:
		thread.terminated = True
		thread.join(2)
		print(thread)
	print(" Closed All ".center(50, '■'))
