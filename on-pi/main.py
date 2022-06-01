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
import picamera
###### Written in this Folder ######
from iot import MQTT
from PID import PID

class ServoController:

	def __init__(self, x_offset=1535, y_offset=1410, seperator='#', stopByte='$', plate_piviot_len=8, servo_arm_len=3.2, camera_distance=45):
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
		self.ball_pose = ()

		try:
			self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, write_timeout=0.03)
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
		#print(self.x_plate_angle*180/math.pi, self.y_plate_angle*180/math.pi, error, axis)

		if axis == "x":
			return error/math.cos(self.x_plate_angle)
		return error/math.cos(self.y_plate_angle)


	@staticmethod
	def angle_map(val):
		if val <= 2500 and val >= 500:
			return 0 + ( (val-500) * ( (math.pi - 0)/(2500-500) ))
		return 0

	def close_serial_port(self):
		if self.arduino_connected:
			self.arduino.close()
			print('\n')
			print(" Closed Serial Port ".center(50, '■'))


class SetMqttController(threading.Thread):
	def __init__(self):
		super().__init__()
		self.ball_lock = threading.Lock()
		self.set_lock = threading.Lock()
		self.pid_lock = threading.Lock()
		self._ball_pose = ()
		self.last_id = -1
		self.mode = 0
		self.two_pi = 6.28
		self.r = 200
		self.f = 0.1
		self.rad = 0
		self.frame_center = ()
		self.frame_size = ()
		self._setpoints = (320, 240)
		self._pid_values = ()
		self.terminated = 0
		self.daemon = True

		self.mqtt = MQTT('localhost')
		self.mqtt.subscribe_thread_start(self.cmd_key_callback, "key_cmd", 2)
		self.start()

	def run(self):
		threading.Thread(target=self.setpoint_change_thread, args=(1/15,), daemon=True).start()
		while not self.terminated:
			ball_pose = self.ball_pose
			self.mqtt.publish_reuse_client('ball_set_pose', json.dumps({"setpoint": self.setpoints, "ball_pose": ball_pose, "pid_values": self.pid_values}), 0, None)

	def setpoint_change_thread(self, secs):
		while not self.terminated:
			if self.mode == 1: # draw circle mode
				self.set_all_setpoints(self.get_circle_corr())
			elif self.mode == 2: # draw 8 mode
				self.set_all_setpoints(self.get_8_corr())
			sleep(secs)

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
		global done
		key = json.loads(msg.payload.decode('UTF-8'))

		if key == 'q':
			done = 1
			self.terminated = 1
		elif key == 'frame_size':
			threading.Thread(target=set_mqtt_ctrl.mqtt.publish, args=('frame_size', json.dumps(self.frame_size), 2)).start()
		elif key == 'o':
			self.mode = 1
			self.r = 100
			self.set_all_setpoints(self.get_circle_corr(reset=True))
		elif key == '8':
			self.mode = 2
			self.r = 90
			self.set_all_setpoints(self.get_8_corr(reset=True))
		elif key[0] == 'S':
			self.mode = 0
			self.set_all_setpoints(tuple(json.loads(key[1:])))
		elif key[0] == 'T':
			parameter = key[1:5]
			pos = float(key[5:])
			self.set_tuning(pos, parameter)
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

	@property
	def ball_pose(self):
		with self.ball_lock:
			return self._ball_pose

	@property
	def setpoints(self):
		with self.set_lock:
			return self._setpoints

	@property
	def pid_values(self):
		with self.pid_lock:
			return self._pid_values

	@pid_values.setter
	def pid_values(self, pid):
		with self.pid_lock:
			self._pid_values = pid

	def set_all_setpoints(self, coordinate):
		with self.set_lock:
			self._setpoints = coordinate
		x_axis_pid.setpoint, y_axis_pid.setpoint = coordinate

	@ball_pose.setter
	def ball_pose(self, ball_pose):
		with self.ball_lock:
			self._ball_pose = ball_pose

	def set_tuning(self, pos, parameter):
		if parameter == "x_Kp":
			x_axis_pid.Kp = pos
		elif parameter == "y_Kp":
			y_axis_pid.Kp = pos
		elif parameter == "x_Ki":
			x_axis_pid.Ki = pos
		elif parameter == "y_Ki":
			y_axis_pid.Ki = pos
		elif parameter == "x_Kd":
			x_axis_pid.Kd = pos
		elif parameter == "y_Kd":
			y_axis_pid.Kd = pos

	def check_id(self, current_id):
		if self.last_id > current_id:
			return False
		self.last_id = current_id
		return True


class ImageProcessor(threading.Thread):
	num_no_ball_frames = 0
	def __init__(self, mat, pool_handler):
		super().__init__()
		self.id = perf_counter()
		self.ball_pose = ()
		self.stream = io.BytesIO()
		self.event = threading.Event()
		self.terminated = False
		self.mat = mat
		self.lower_limits = np.array([0, 140, 50])
		self.upper_limits = np.array([32, 255, 255])
		self.kernel_open = np.ones((5, 5))
		self.kernel_close = np.ones((5, 5))
		self.frame = None
		self.daemon = True
		self.pool_handler = pool_handler
		self.start()

	def run(self):
		# This method runs in a separate thread
		while not self.terminated:
			# Wait for an image to be written to the stream
			if self.event.wait(1):
				try:
					self.stream.seek(0)
					self.frame = cv2.imdecode(np.frombuffer(self.stream.read(), np.uint8), cv2.IMREAD_COLOR)
					if self.mat:
						self.frame = self.frame[self.mat[1][0] : self.mat[1][1], self.mat[0][0] : self.mat[0][1]]
					self.findBall()
					with id_lock:
						if set_mqtt_ctrl.check_id(self.id):
							if self.ball_pose:
								servo_ctrl.ball_pose = self.ball_pose[0]
								x_pid = x_axis_pid(self.ball_pose[0][0])
								y_pid = y_axis_pid(self.ball_pose[0][1])
								servo_ctrl.send_arduino(x_pid, y_pid)
								ImageProcessor.num_no_ball_frames = 0
								set_mqtt_ctrl.pid_values = (x_pid, y_pid)
							else:
								if ImageProcessor.num_no_ball_frames >=90: # if Ball is not there for more than 90 frames make plate flat
									servo_ctrl.send_arduino(0, 0)
									ImageProcessor.num_no_ball_frames = 0
									x_axis_pid.reset()
									y_axis_pid.reset()
								else:
									ImageProcessor.num_no_ball_frames +=1
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
					with self.pool_handler.lock:
						self.pool_handler.pool.append(self)

	def findBall(self):

		hsvFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsvFrame, self.lower_limits, self.upper_limits)

		mask_open = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
		mask_close = cv2.morphologyEx(mask_open, cv2.MORPH_CLOSE, self.kernel_close)
		contours, _ = cv2.findContours(mask_close, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
		if contours:
			ball = max(contours, key = cv2.contourArea)
			if cv2.contourArea(ball) <= 0:
				self.ball_pose = ()
				return False
			((x,y), radius) = cv2.minEnclosingCircle(ball)
			if radius < 5 or radius > 40:
				self.ball_pose = ()
				return False
			self.ball_pose = ((int(x), int(y)), int(radius))
			return True
		self.ball_pose = ()
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
			return round(sum(self.ptime_vals)/len(self.ptime_vals), 3), round(max(self.ptime_vals), 3)

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
	ids_points_dict = {}
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

		if np.in1d(ids_present, ids).any():
			ids = list(ids[:, 0])
			for index, _id in enumerate(ids):
				if _id in ids_present:
					corner = ids_present.index(_id)
					ids_points_dict[_id] = [corners[index][0][corner][0], corners[index][0][corner][1]]
			print('\r', ids_points_dict.keys(), end='')

		if np.in1d(ids_present, tuple(ids_points_dict.keys())).all():
			TL = ids_points_dict[ids_present[0]]
			TR = ids_points_dict[ids_present[1]]
			BR = ids_points_dict[ids_present[2]]
			BL = ids_points_dict[ids_present[3]]
			points = np.array([TL, TR, BR, BL], dtype="int32")

			mat = ( ( min(points[:, 0]), max(points[:, 0]) ), ( min(points[:, 1]), max(points[:, 1]) ) )
			frame = frame[mat[1][0] : mat[1][1], mat[0][0] : mat[0][1]]

			frame_size = frame.shape[1], frame.shape[0]
			set_mqtt_ctrl.frame_size = frame_size
			set_mqtt_ctrl.frame_center = frame_size[0]//2, frame_size[1]//2
			set_mqtt_ctrl.mqtt.publish('frame_size', json.dumps(frame_size), 2)

			set_mqtt_ctrl.set_all_setpoints(set_mqtt_ctrl.frame_center)
			break

		stream.seek(0)
		stream.truncate()

	print('\n')
	print(" FOUND TABLE ".center(50, '■'))


class ProcessOutput(object):
	def __init__(self, num_image_processors):
		self.lock = threading.Lock()
		self.pool = [ImageProcessor(mat, self) for _ in range(num_image_processors)]
		self.processor = None
		self.times_starved = 0

	def write(self, buf):
		if buf.startswith(b'\xff\xd8'):
			with self.lock:
				if self.pool:
					self.processor = self.pool.pop()
				else:
					self.times_starved += 1
					self.processor = None

			if self.processor:
				self.processor.stream.write(buf)
				self.processor.id = perf_counter()
				self.processor.event.set()
				fps.fps_update()

		if fps.ready():
			curr_fps = fps.get_fps()
			fDropped = int((fps.dropped_frames/fps.avg_frames)*curr_fps)
			print(f"\rFPS: {curr_fps} | Ptime: {fps.get_pTime()} ms | fDropped: {fDropped}/{curr_fps} | {self.times_starved}".ljust(65), end='')
			fps.reset()
			self.times_starved = 0

	def flush(self):
		self.processor.terminated = True
		for proc in self.pool:
			proc.terminated = True
			proc.join(2)
			print(proc)


if __name__ == '__main__':

	# !!!!!!!!!!!!!!! Configs !!!!!!!!!!!!!!!!!
	DEBUG = True
	aruco_marker_order = [0, 1, 2, 3]
	video_src = 0
	use_mqtt = 1
	num_image_processors = 4
	done = 0

	id_lock = threading.Lock()
	fps = FPS()

	x_axis_pid = PID("x", Kp=1.70, Ki=0.08, Kd=1.26, exp_filter_alpha=0.85)
	x_axis_pid.output_limits = -750, 750

	y_axis_pid = PID("y", Kp=1.85, Ki=0.1, Kd=1.36, exp_filter_alpha=0.9)
	y_axis_pid.output_limits = -750, 750

	x_axis_pid.sample_time, y_axis_pid.sample_time = 1/80, 1/80

	servo_ctrl = ServoController()
	
	x_axis_pid.error_map = y_axis_pid.error_map = servo_ctrl.error_map
	mat = None

	set_mqtt_ctrl = SetMqttController()

	with picamera.PiCamera() as camera:
		camera.resolution = (640, 480)
		camera.framerate = 90
		sleep(2)
		camera.capture_sequence(find_table(aruco_marker_order), use_video_port=True)
		sleep(2)
		output = ProcessOutput(num_image_processors)
		camera.start_recording(output, format='mjpeg')
		while not done:
			camera.wait_recording(1)
		camera.stop_recording()

	servo_ctrl.close_serial_port()
	set_mqtt_ctrl.terminated = True
	print(" Closed All ".center(50, '■'))
