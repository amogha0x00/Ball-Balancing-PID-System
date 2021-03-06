"""
	Author : Amoghavarsha S G
"""

import threading
from graph_plotter import qt_plotter
from multiprocessing import Array, Value
from iot import MQTT
import json


def pose_callback(client, userdata, msg):
	pose = json.loads(msg.payload.decode('UTF-8'))
	ball_pose = pose['ball_pose']
	pid_values = pose['pid_values']
	if ball_pose:
		ball_pose_plot[:] = ball_pose[0]
	if pid_values:
		pid_plot[:] = pid_values
	setpoint_plot[:] = pose['setpoint']

def key_callback(client, userdata, msg):
	key = json.loads(msg.payload.decode('UTF-8'))
	if key == 'q':
		plot_terminate.value = 1


if __name__ == '__main__':

	broker_url = "raspberrypi.local"

	mqtt = MQTT(broker_url=broker_url)
	pose_topic = 'ball_set_pose'
	key_cmd_topic = "key_cmd"

	plot_terminate = Value('i', 0)
	setpoint_plot = Array('i', [0, 0])
	ball_pose_plot = Array('i', [0, 0])
	pid_plot = Array('d', [0, 0])

	mqtt.subscribe_thread_start(pose_callback, pose_topic, 0)
	mqtt.subscribe_thread_start(key_callback, key_cmd_topic, 2)

	qt_plotter(setpoint_plot, ball_pose_plot, pid_plot, plot_terminate)
