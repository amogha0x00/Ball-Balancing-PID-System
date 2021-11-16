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
import traceback
import json
from iot import IOT
import picamera
###### Written in this Folder ######
from PID import PID

class ServoController:

	def __init__(self, x_offset=1570, y_offset=1420, seperator='#', stopByte='$', plate_piviot_len=8, servo_arm_len=3.2, camera_distance=45):
		"""
			Initialize a new Servo Controller
			:param plate_piviot_len: Distance between connected servo rod and piviot
			:param servo_arm_len: Length of the servo arm not the push rod
		"""
		self.arduino_connected = False
		self.x_offset = x_offset
		self.y_offset = y_offset
		self.seperator = seperator
		self.stopByte = stopByte
		self.plate_piviot_len = plate_piviot_len
		self.servo_arm_len = servo_arm_len
		self.camera_distance = camera_distance
		self.x_offsetAngle = self.angle_map(x_offset)
		self.y_offsetAngle = self.angle_map(y_offset)
		self.x_plate_angle = 0
		self.y_plate_angle = 0
		self.frame_center = ()
		self.ball_pose = ()

		try:
			self.arduino = serial.Serial(port='COM3', baudrate=115200, write_timeout=0.01)
			self.arduino_connected = True
		except serial.SerialException:
			print(" Unable to Open Serial Port ".center(50, '■'), end='\n\n')

	def send_arduino(self, x_pid=0, y_pid=0):
		x_servo_us = int(x_pid + self.x_offset)
		y_servo_us = int(y_pid + self.y_offset)
		#print(f"x_pid: {x_servo_us} y_pid: {y_servo_us}")
		
		if self.arduino_connected:
			cmd = f"{x_servo_us}{self.seperator}{y_servo_us}{self.stopByte}"
			self.arduino.write(cmd.encode('utf-8'))
			self.arduino.flush()

		servo_x_angle = self.angle_map(x_servo_us) - self.x_offsetAngle
		servo_y_angle = self.angle_map(y_servo_us) - self.y_offsetAngle
		x_displacement = self.servo_arm_len * math.sin(servo_x_angle)
		y_displacement = self.servo_arm_len * math.sin(servo_y_angle)
		self.x_plate_angle = math.asin(x_displacement/self.plate_piviot_len)
		self.y_plate_angle = math.asin(y_displacement/self.plate_piviot_len)

	def error_map(self, error, axis):
		if not self.frame_center:
			return error
		if axis == "x":
			r_error = self.frame_center[0] - self.ball_pose[0] 
			r_plate_angle = self.y_plate_angle
			other_error = abs((self.frame_center[1] - self.ball_pose[1])*math.sin(self.y_plate_angle))/self.camera_distance
			own_error = error/math.cos(self.x_plate_angle)
		else:
			r_error = self.frame_center[1] - self.ball_pose[1]
			r_plate_angle = self.x_plate_angle
			other_error = abs((self.frame_center[0] - self.ball_pose[0])*math.sin(self.x_plate_angle))/self.camera_distance
			own_error = error/math.cos(self.y_plate_angle)
		if error == 0:
			return error
		if r_plate_angle > 0:
			if r_error > 0:
				error_ = own_error #+ ((error/abs(error)) * other_error)
			else:
				error_ = own_error #+  ((error/abs(error)) * - other_error)
		else:
			if r_error > 0:
				error_ = own_error #+  ((error/abs(error)) * - other_error)
			else:
				error_ = own_error #+ ((error/abs(error)) * other_error)
		
		#print(self.x_plate_angle*180/math.pi, self.y_plate_angle*180/math.pi, error, error_, axis)
		return error_

	@staticmethod
	def angle_map(val):
		if val <= 2500 and val >= 500:
			return 0 + ( (val-500) * ( (math.pi - 0)/(2500-500) ))
		return 0

	def close_serial_port(self):
		if self.arduino_connected:
			self.arduino.close()
			print(" Closed Serial Port ".center(50, '■'))


class SetMqttController(threading.Thread):
	def __init__(self):
		super().__init__()
		self.lock = threading.Lock()
		self.set_lock = threading.Lock()
		self._ball_pose = ()
		self.last_id = -1
		self.mode = 0
		self.two_pi = 6.28
		self.r = 200
		self.f = 0.1
		self.graphShown = 0
		self.rad = 0
		self.frame_center = ()
		self._setpoints = (320, 240)
		self.terminated = 0
		self.daemon = True

		self.iot = IOT('localhost')
		self.iot.mqtt_subscribe_thread_start(self.cmd_key_callback, "keyCmd", 2)
		self.iot.mqtt_subscribe_thread_start(self.cmd_set_callback, "setPointCmd", 2)
		self.start()

	def run(self):
		global done
		while not (self.ball_pose or self.terminated):
			sleep(0.01)

		while not self.terminated:
			ball_pose = self.ball_pose

			if self.mode == 1: # draw circle mode
				self.set_all_setpoints(self.get_circle_corr())
			elif self.mode == 2: # draw 8 mode
				self.set_all_setpoints(self.get_8_corr())

			if ball_pose:
				self.iot.mqtt_publish_reuse_client('ball_set_pose', json.dumps({"setpoint": self.setpoints, "ball_pose": ball_pose}), 0, 5)

	def get_circle_corr(self, rad_offset=0, reset=False, xCenterOffset=0, yCenterOffset=0):

		if not (-self.two_pi + rad_offset < self.rad < self.two_pi + rad_offset) or reset:
			self.initial_time = perf_counter()
			self.rad = rad_offset
		else:
			self.rad = self.two_pi*self.f*(perf_counter() - self.initial_time) + rad_offset

		centre = self.frame_center[0] + xCenterOffset, self.frame_center[1] + yCenterOffset
		return int( centre[0] + self.r*np.cos(self.rad)), int(centre[1] + self.r*np.sin(self.rad))

	def get_8_corr(self, reset=False):

		if reset:
			self.step = 0
			self.f = 0.1
		if not self.step:
			if -self.two_pi < self.rad < self.two_pi:
				x, y = self.get_circle_corr(reset=reset, xCenterOffset=-self.r)
			else:
				self.step = 1
				self.f *= -1
				x, y = self.get_circle_corr(reset=True, rad_offset=self.two_pi/2, xCenterOffset=self.r)
		else:
			if -self.two_pi/2  < self.rad < self.two_pi*3/2:
				x, y = self.get_circle_corr(reset=reset, rad_offset=self.two_pi/2, xCenterOffset=self.r)
			else:
				self.step = 0
				self.f *= -1
				x, y = self.get_circle_corr(reset=True, xCenterOffset=-self.r)
		return x, y

	def cmd_key_callback(self, client, userdata, msg):

		key = json.loads(msg.payload.decode('UTF-8'))
		if key == 'q':
			done = 1
			self.terminated = 1
		elif key == 'o':
			self.mode = 1
			self.r = 100
			self.set_all_setpoints(self.get_circle_corr(reset=True))
		elif key == '8':
			self.mode = 2
			self.r = 90
			self.set_all_setpoints(self.get_8_corr(reset=True))
		elif key == 'r':
			self.mode = 0
			self.set_all_setpoints(self.frame_center)
		elif key == 'f':
			self.f *= -1
		elif key == '-':
			if self.f >= 0:
				self.f -= 0.1
			else:
				self.f += 0.1
		elif key == '+':
			if -3 < self.f < 3:
				if self.f >= 0:
					self.f += 0.1
				else:
					self.f -= 0.1

	def cmd_set_callback(self, client, userdata, msg):
		setpoints = json.loads(msg.payload.decode('UTF-8'))
		self.set_all_setpoints(setpoints)

	@property
	def ball_pose(self):
		with self.lock:
			return self._ball_pose

	@property
	def setpoints(self):
		with self.set_lock:
			return self._setpoints

	def set_all_setpoints(self, coordinate):
		with self.set_lock:
			self._setpoints = coordinate
		x_axis_pid.setpoint, y_axis_pid.setpoint = coordinate

	@ball_pose.setter
	def ball_pose(self, ball_pose):
		with self.lock:
			self._ball_pose = ball_pose

	def set_tuning(self, pos, parameter):
		if parameter == "xKp":
			x_axis_pid.Kp = pos
		elif parameter == "yKp":
			y_axis_pid.Kp = pos
		elif parameter == "xKi":
			x_axis_pid.Ki = pos
		elif parameter == "yKi":
			y_axis_pid.Ki = pos
		elif parameter == "xKd":
			x_axis_pid.Kd = pos
		elif parameter == "yKd":
			y_axis_pid.Kd = pos

	def check_id(self, current_id):
		if self.last_id > current_id:
			return False
		self.last_id = current_id
		return True


class ImageProcessor(threading.Thread):
	def __init__(self, mat):
		super().__init__()
		self.id = perf_counter()
		self.ball_pose = ()
		self.stream = io.BytesIO()
		self.event = threading.Event()
		self.terminated = False
		#self.mat = cv2.getPerspectiveTransform(np.float32([[0,0], [449,0], [449,449], [0,449]]), np.float32([[0,0], [449,0], [449,449], [0,449]]))
		self.mat = mat
		self.lower_limits = np.array([0, 0, 130])
		self.upper_limits = np.array([255, 255, 255])
		self.kernel_open = np.ones((5, 5))
		self.kernel_close = np.ones((5, 5))
		self.frame = None
		self.daemon = True
		self.start()

	def run(self):
		# This method runs in a separate thread
		global num_no_ball_frames
		while not self.terminated:
			# Wait for an image to be written to the stream
			if self.event.wait(1):
				try:
					self.stream.seek(0)
					self.frame = cv2.imdecode(np.frombuffer(self.stream.read(), np.uint8), cv2.IMREAD_COLOR)
					if self.mat:
						self.frame = self.frame[self.mat[1][0] : self.mat[1][1], self.mat[0][0] : self.mat[0][1]]
					#cv2.imwrite('im.jpg', self.frame)
					#print(self.frame)
					self.findBall()
					with id_lock:
						if set_mqtt_ctrl.check_id(self.id):
							if self.ball_pose:
								servo_ctrl.ball_pose = self.ball_pose[0]
								x_pid = x_axis_pid(self.ball_pose[0][0])
								y_pid = y_axis_pid(self.ball_pose[0][1])
								servo_ctrl.send_arduino(x_pid, y_pid)
								num_no_ball_frames = 0
							else:
								if num_no_ball_frames >=90: # if Ball is not there for more than 90 frames make plate flat
									servo_ctrl.send_arduino(0, 0)
									num_no_ball_frames = 0
									x_axis_pid.reset()
									y_axis_pid.reset()
								else:
									num_no_ball_frames +=1
							set_mqtt_ctrl.ball_pose = self.ball_pose
						else:
							fps.dropped_frames += 1

				except serial.SerialTimeoutException:
					print("Data Not Sent - TIMEOUT!!!!!")
				except Exception as e:
					print(e)
					print(traceback.format_exc())
				finally:
					# Reset the stream and event
					self.stream.seek(0)
					self.stream.truncate()
					self.event.clear()
					fps.ptime_update((perf_counter() - self.id)*1000)
					# Return ourselves to the processor_pool
					with pool_lock:
						processor_pool.append(self)

	def findBall(self):

		hsvFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsvFrame, self.lower_limits, self.upper_limits)

		mask_open = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
		mask_close = cv2.morphologyEx(mask_open, cv2.MORPH_CLOSE, self.kernel_close)
		contours, _ = cv2.findContours(mask_close, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
		#self.frame = mask_close
		if contours:
			ball = max(contours, key = cv2.contourArea)
			if cv2.contourArea(ball) <= 0:
				self.ball_pose = ()
				return False
			((x,y), radius) = cv2.minEnclosingCircle(ball)
			if radius < 12 or radius > 60:
				self.ball_pose = ()
				#return False			
			self.ball_pose = ((int(x), int(y)), int(radius))
			#print(self.ball_pose)
			#cv2.drawContours(self.frame, ball, -1, (255,255,255), 3)
			#M = cv2.moments(ball)
			#self.ball_pose = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			#print(self.ball_pose)
			return True
		self.ball_pose = ()#((320, 240),10)
		return False


class FPS():
	def __init__(self, avg_frames=250):
		self.n_frames = 0
		self.avg_frames = avg_frames
		self.ptime_vals = []
		self.initial_time = perf_counter()
		self._dropped_frames = 0

		self.plock = threading.Lock()
		self.dlock = threading.Lock()

	def fps_update(self):
		self.n_frames += 1

	def get_fps(self):
		return int(self.n_frames/(perf_counter() - self.initial_time))

	def ptime_update(self, pTime):
		with self.plock:
			self.ptime_vals.append(pTime)

	def get_pTime(self):
		with self.plock:
			return round(sum(self.ptime_vals)/len(self.ptime_vals), 4), round(max(self.ptime_vals), 4)

	def ready(self):
		return self.n_frames >= self.avg_frames

	@property
	def dropped_frames(self):
		with self.dlock:
			return self._dropped_frames

	@dropped_frames.setter
	def dropped_frames(self, dropped_frames):
		with self.dlock:
			self._dropped_frames = dropped_frames

	def reset(self):
		with self.plock:
			self.ptime_vals = []
		self.dropped_frames = 0
		self.n_frames = 0
		self.initial_time = perf_counter()

def find_table(ids_present):
	stream = io.BytesIO()
	global mat
	threshold = 5
	while not done:
		yield stream
		stream.seek(0)
		frame = cv2.imdecode(np.frombuffer(stream.read(), np.uint8), cv2.IMREAD_COLOR)
		grey_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		upper_bound = (threshold, threshold, threshold)
		grey_frame = 255 - cv2.inRange(frame, (0, 0, 0), upper_bound)
		aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
		arucoParameters = aruco.DetectorParameters_create()
		corners, ids, rejectedImgPoints = aruco.detectMarkers(grey_frame, aruco_dict, parameters=arucoParameters)
		threshold = (threshold + 1) % 256
		if np.in1d(ids_present, ids).all():
			frame = aruco.drawDetectedMarkers(frame, corners, ids)
			ids = list(ids[:, 0])
			index = []
			for _id in ids_present:
				index.append(ids.index(_id))
			TL = [corners[index[0]][0][0][0], corners[index[0]][0][0][1]]
			TR = [corners[index[1]][0][1][0], corners[index[1]][0][1][1]]
			BR = [corners[index[2]][0][2][0], corners[index[2]][0][2][1]]
			BL = [corners[index[3]][0][3][0], corners[index[3]][0][3][1]]
			points = np.array([TL, TR, BR, BL], dtype="int32")
			mat = ( ( min(points[:, 0]), max(points[:, 0]) ), ( min(points[:, 1]), max(points[:, 1]) ) )
			frame = frame[mat[1][0] : mat[1][1], mat[0][0] : mat[0][1]]
			set_mqtt_ctrl.frame_center = (frame.shape[1]//2, frame.shape[0]//2)
			servo_ctrl.frame_center = set_mqtt_ctrl.frame_center
			set_mqtt_ctrl.set_all_setpoints(set_mqtt_ctrl.frame_center)
			break
		else:
			print(ids)
		stream.seek(0)
		stream.truncate()

	print(" FOUND TABLE ".center(50, '■'))


def streams():
	global done
	is_starved = 'N'
	while not done:
		try:
			with pool_lock:
				if processor_pool:
					processor = processor_pool.pop()
				else:
					processor = None
			if processor:
				yield processor.stream
				processor.id = perf_counter()
				processor.event.set()
			else:
				# When the pool is starved, wait a while for it to refill
				#print("pool is starved")
				is_starved = 'Y'
				sleep(0.02)
			fps.fps_update()
			if fps.ready():
				print(f"\rFPS: {fps.get_fps()} | Ptime: {fps.get_pTime()} ms | fDropped: {fps.dropped_frames}/{fps.avg_frames} | {is_starved}".ljust(65), end='')
				fps.reset()
				is_starved = 'N'

		except KeyboardInterrupt:
			print ("Ctrl-c pressed ...")
			done = 1

if __name__ == '__main__':

	# !!!!!!!!!!!!!!! Configs !!!!!!!!!!!!!!!!!
	DEBUG = True
	video_src = 0
	use_mqtt = 1
	num_image_processors = 4
	done = 0

	num_no_ball_frames = 0
	pool_lock = threading.Lock()
	id_lock = threading.Lock()
	fps = FPS()

	x_axis_pid = PID("x", Kp=1.70, Ki=0.08, Kd=1.26)
	x_axis_pid.output_limits = -750, 750

	y_axis_pid = PID("y", Kp=1.85, Ki=0.1, Kd=1.36)
	y_axis_pid.output_limits = -750, 750

	x_axis_pid.sample_time, y_axis_pid.sample_time = 1/32, 1/32

	servo_ctrl = ServoController()
	
	x_axis_pid.error_map = y_axis_pid.error_map = servo_ctrl.error_map
	mat = None

	set_mqtt_ctrl = SetMqttController()

	with picamera.PiCamera() as camera:
		camera.resolution = (640, 360)
		camera.framerate = 90
		sleep(2)
		# Now fix the values
		#camera.shutter_speed = 2720 #int(camera.exposure_speed/4)
		#print(str(camera.shutter_speed))
		#camera.exposure_mode = 'off'
		#g = camera.awb_gains
		#camera.awb_mode = 'off'
		#camera.awb_gains = g
		camera.capture_sequence(find_table([0, 1, 2, 3]), use_video_port=True)
		processor_pool = [ImageProcessor(mat) for _ in range(num_image_processors)]
		all_threads = processor_pool[::-1]
		if use_mqtt:
			all_threads.insert(0, set_mqtt_ctrl)
		sleep(2)
		camera.capture_sequence(streams(), use_video_port=True)


	servo_ctrl.close_serial_port()

	for thread in all_threads:
		thread.terminated = True
		thread.join(2)
		print(thread)
	print(" Closed All ".center(50, '■'))
