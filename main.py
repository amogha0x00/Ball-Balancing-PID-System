#!/usr/bin/env python3

"""
	Author : Amoghavarsha S G
"""
import cv2
import numpy as np
import threading
from time import sleep, perf_counter
from multiprocessing import Process, Array, Value
import json
import subprocess
import platform
from iot import IOT
import yaml
import pickle
# import traceback

class SetDispMqttController:
	def __init__(self):
		with open('broker_config.yaml', 'r') as file_open:
			broker_url = yaml.load(file_open, yaml.SafeLoader)['broker_url']
		self.iot = IOT(broker_url=broker_url)

		with open('ball_imgs.pkl', 'rb') as outp:
			self.ball_imgs = pickle.load(outp)

		self.lock = threading.Lock()
		self.set_lock = threading.Lock()
		self.table_img = cv2.imread("background.jpg", -1)
		self.return_img = np.array(self.table_img)
		self.table_height, self.table_width = self.table_img.shape[:2] 
		self.frame_center = (self.table_width//2, self.table_height//2)
		self._setpoint = ()
		self._ball_pose = ()
		self.graphShown = 0
		self.terminated = 0
		self.mode = 0
		self.setpoint = self.frame_center

		self.iot.mqtt_subscribe_thread_start(self.pose_callback, "ball_set_pose", 0)
		self.iot.mqtt_subscribe_thread_start(self.frame_size_callback, "frame_size", 2)

	def run(self):
		while not (self.ball_pose or self.terminated):
			sleep(0.01)

		if self.terminated:
			return

		cv2.namedWindow('Processed Video Feed', cv2.WINDOW_NORMAL)
		height, width = self.return_img.shape[:2]
		cv2.resizeWindow('Processed Video Feed', int(width*1.1), int(height*1.1))
		cv2.setMouseCallback('Processed Video Feed', self.set_setpoint)
		if DEBUG:
			cv2.createTrackbar('x_Kp(x100)', 'Processed Video Feed', 0, 500, lambda x: self.set_tuning(x/100, "x_Kp"))
			cv2.createTrackbar('x_Ki(x100)', 'Processed Video Feed', 0, 100, lambda x: self.set_tuning(x/100, "x_Ki"))
			cv2.createTrackbar('x_Kd(x100)', 'Processed Video Feed', 0, 1500, lambda x: self.set_tuning(x/100, "x_Kd"))
			x_Kp, x_Ki, x_Kd = 1, 0, 0 #xAxisPid.tunings
			cv2.setTrackbarPos('x_Kp(x100)', 'Processed Video Feed', int(x_Kp*100))
			cv2.setTrackbarPos('x_Ki(x100)', 'Processed Video Feed', int(x_Ki*100))
			cv2.setTrackbarPos('x_Kd(x100)', 'Processed Video Feed', int(x_Kd*100))

			cv2.createTrackbar('y_Kp(x100)', 'Processed Video Feed', 0, 500, lambda x: self.set_tuning(x/100, "y_Kp"))
			cv2.createTrackbar('y_Ki(x100)', 'Processed Video Feed', 0, 100, lambda x: self.set_tuning(x/100, "y_Ki"))
			cv2.createTrackbar('y_Kd(x100)', 'Processed Video Feed', 0, 1500, lambda x: self.set_tuning(x/100, "y_Kd"))
			y_Kp, y_Ki, y_Kd = 1, 0, 0#yAxisPid.tunings
			cv2.setTrackbarPos('y_Kp(x100)', 'Processed Video Feed', int(y_Kp*100))
			cv2.setTrackbarPos('y_Ki(x100)', 'Processed Video Feed', int(y_Ki*100))
			cv2.setTrackbarPos('y_Kd(x100)', 'Processed Video Feed', int(y_Kd*100))

		while not self.terminated:
			setpoint = self.setpoint

			if self.ball_pose:
				ball_pose, radius = self.ball_pose
				self.animate(ball_pose, radius)
				# draw circle around the ball and arrowed line b/w ball and setpoint
				cv2.circle(self.return_img, ball_pose, radius, (0, 255, 255), 2)
				cv2.arrowedLine(self.return_img, ball_pose, setpoint, (0, 0, 255), 2)
			else:
				self.return_img = self.return_img = np.array(self.table_img)
			
			cv2.circle(self.return_img, setpoint, 4 ,(0, 255, 0), -1)

			if self.mode == 1:
				cv2.circle(self.return_img, self.frame_center, self.r ,(255,255,255),1)
			elif self.mode == 2: # draw 8 mode
				cv2.circle(self.return_img, (self.frame_center[0]-self.r, self.frame_center[1]), self.r ,(255,255,255),1)
				cv2.circle(self.return_img, (self.frame_center[0]+self.r, self.frame_center[1]), self.r ,(255,255,255),1)

			cv2.imshow('Processed Video Feed', self.return_img)
			key = chr(cv2.waitKey(1) & 0xFF)

			if key in ['q', 'o', '8', 'r', 'f', '-', '+']:
				threading.Thread(target=self.iot.mqtt_publish, args=('key_cmd', json.dumps(key), 2)).start()

			if key == 'q':
				self.terminated = 1
				# plotTerminate.value = 1
			elif key == 'o':
				self.mode = 1
				self.r = 100
				# self.setpoint(self.get_circle_corr(reset=True))
			elif key == '8':
				self.mode = 2
				self.r = 90
				# self.setpoint(self.get8Corr(reset=True))
			elif key == 'r':
				self.mode = 0
				self.setpoint = self.frame_center
			# elif key == 'g':
			# 	if self.graphShown:
			# 		plotTerminate.value = 1
			# 		self.graphShown = 0
			# 	else:
			# 		self.graphShown = 1
			# 		plotTerminate.value = 0
			# 		args = (setpointPlot, ball_posePlot, plotTerminate)
			# 		kwargs = {'identifier': 'GRAPH', 'maxLimit': (max(width, height)) + 50}
			# 		Process(target=plotGraph, args=args, kwargs=kwargs).start()

			# elif key == 'f':
			# 	self.f *= -1
			# 	pass
			# elif key == '-'):
		# 	if self.f >= 0:
			# 		self.f -= 0.1
			# 	else:
			# 		self.f += 0.1
			# elif key == '+'):
		# 	if -3 < self.f < 3:
			# 		if self.f >= 0:
			# 			self.f += 0.1
			# 		else:
			# 			self.f -= 0.1

		cv2.destroyAllWindows()
		cv2.waitKey(1)

	def animate(self, ball_pose, radius=15):
		st_time = perf_counter()
		ball_img = self.ball_imgs[radius]
		ball_height, ball_width = ball_img.shape[:2]
		ball_center = (ball_width//2, ball_height//2)
		
		if (ball_pose[1] - ball_center[1] > - ball_height) and (ball_pose[1] + ball_center[1] < self.table_height + ball_height) and (ball_pose[0] - ball_center[0] > - ball_width) and (ball_pose[0] + ball_center[0] < self.table_width + ball_width): 
			tmin_x, tmin_y, tmax_x, tmax_y = ball_pose[0] - ball_center[0], ball_pose[1] - ball_center[1], ball_pose[0] + ball_center[0], ball_pose[1] + ball_center[1]
			bmin_x, bmin_y, bmax_x, bmax_y = 0, 0, ball_width, ball_height
			if tmin_x < 0:
				bmin_x = abs(tmin_x)
				tmin_x = 0
			if tmax_x > self.table_width:
				bmax_x = ball_width - (tmax_x - self.table_width)
				tmax_x = self.table_width
			if tmin_y < 0:
				bmin_y = abs(tmin_y)
				tmin_y = 0
			if tmax_y > self.table_height:
				bmax_y = ball_height - (tmax_y - self.table_height)
				tmax_y = self.table_height

			self.return_img = np.array(self.table_img)
			return_img_roi = self.return_img[tmin_y:tmax_y, tmin_x:tmax_x]
			cv2.add(return_img_roi, ball_img[bmin_y:bmax_y, bmin_x:bmax_x], dst=return_img_roi)
		else:
			self.return_img = np.array(self.table_img)

		fps.ptime_update((perf_counter() - st_time)*1000)
		fps.fps_update()
		if fps.ready():
			print(f"\rFPS: {fps.get_fps()} | Ptime: {fps.get_ptime()} ms".ljust(65), end='')
			fps.reset()


	@property
	def ball_pose(self):
		with self.lock:
			return self._ball_pose

	@ball_pose.setter
	def ball_pose(self, ball_pose):
		with self.lock:
			self._ball_pose = ball_pose

	def pose_callback(self, client, userdata, msg):
		pose = json.loads(msg.payload.decode('UTF-8'))
		self.ball_pose = pose['ball_pose']
		self.setpoint = pose['setpoint']

	def frame_size_callback(self, client, userdata, msg):
		size = json.loads(msg.payload.decode('UTF-8'))
		print(f"Table Found, {size = }")
		self.table_img = cv2.resize(self.table_img, dsize=size, interpolation=cv2.INTER_AREA)
		self.return_img = np.array(self.table_img)
		self.table_height, self.table_width = self.table_img.shape[:2] 
		self.frame_center = (self.table_width//2, self.table_height//2)
		client.loop_stop()
		client.unsubscribe('frame_size')
		client.disconnect()

	@property
	def setpoint(self):
		with self.set_lock:
			return self._setpoint

	@setpoint.setter
	def setpoint(self, coordinate):
		with self.set_lock:
			self._setpoint = coordinate

	def set_setpoint(self, event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self.mode = 0
			self.setpoint = [x, y]

	def set_tuning(self, pos, parameter):
		pass
		# if parameter == "x_Kp":
		# 	xAxisPid.Kp = pos
		# elif parameter == "y_Kp":
		# 	yAxisPid.Kp = pos
		# elif parameter == "x_Ki":
		# 	xAxisPid.Ki = pos
		# elif parameter == "y_Ki":
		# 	yAxisPid.Ki = pos
		# elif parameter == "x_Kd":
		# 	xAxisPid.Kd = pos
		# elif parameter == "y_Kd":
		# 	yAxisPid.Kd = pos


class FPS():
	def __init__(self, avg_frames=30):
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

	def get_ptime(self):
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

def get_circle_corr():
	i = 0
	j = 0
	f = 1
	r = 200
	rad = 0
	initial_time = perf_counter()
	two_pi = 2*3.14
	centre = set_disp_mqtt_ctrl.frame_center
	while 1:
		if fps.ready():
			print(f"\rFPS: {fps.get_fps()} | Ptime: {fps.get_ptime()} ms".ljust(65), end='')
			fps.reset()

		if not (-two_pi < rad < two_pi):
			initial_time = perf_counter()
			rad = 0
		else:
			rad = two_pi*f*(perf_counter() - initial_time)
		sleep(1/40)
		set_disp_mqtt_ctrl.ball_pose = (int(centre[0] + r*np.cos(rad)), int(centre[1] + r*np.sin(rad))), 25


if __name__ == '__main__':

	# !!!!!!!!!!!!!!! Configs !!!!!!!!!!!!!!!!!
	DEBUG = False
	show_interface = 1
	fps = FPS()
	if platform.system() == 'Windows':
		Process(target=subprocess.run, args=[['py', 'plot_graph.py']]).start()
	set_disp_mqtt_ctrl = SetDispMqttController()

	# set_disp_mqtt_ctrl.ball_pose = (0, 0), 25
	# threading.Thread(target=get_circle_corr, daemon=True).start()
	set_disp_mqtt_ctrl.run()
	print('\n')
	print('EXITING'.center(50, '#'))
	# with open('animated_data.pkl', 'wb') as outp:
	#     pickle.dump(set_disp_mqtt_ctrl.animated, outp, pickle.HIGHEST_PROTOCOL)