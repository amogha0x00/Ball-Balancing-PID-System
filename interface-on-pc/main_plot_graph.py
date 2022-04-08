"""
	Author : Amoghavarsha S G
"""

import threading
from graph_plotter import qt_plotter
from multiprocessing import Array, Value
from iot import IOT
import json


def pose_callback(client, userdata, msg):
	# print(json.loads(msg.payload.decode('UTF-8')))
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

def initiate_mqtt_communication(iot_obj, pose_callback, key_callback, pose_topic, key_cmd_topic):
	iot_obj.mqtt_subscribe_thread_start(pose_callback, pose_topic, 0)
	iot_obj.mqtt_subscribe_thread_start(key_callback, key_cmd_topic, 2)


if __name__ == '__main__':

	broker_url = "raspberrypi.local"

	iot = IOT(broker_url=broker_url)
	pose_topic = 'ball_set_pose'
	key_cmd_topic = "key_cmd"

	plot_terminate = Value('i', 0)
	setpoint_plot = Array('i', [0, 0])
	ball_pose_plot = Array('i', [0, 0])
	pid_plot = Array('d', [0, 0])

	threading.Thread(target=initiate_mqtt_communication, args=(iot, pose_callback, key_callback, pose_topic, key_cmd_topic), daemon=True).start()
	qt_plotter(setpoint_plot, ball_pose_plot, pid_plot, plot_terminate)
