"""
	Author : Amoghavarsha S G
"""

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


if __name__ == '__main__':

	broker_url = "raspberrypi.local"

	iot = IOT(broker_url=broker_url)
	sub_topic = 'ball_set_pose'
	qos = 0

	plot_terminate = Value('i', 0)
	setpoint_plot = Array('i', [0, 0])
	ball_pose_plot = Array('i', [0, 0])
	pid_plot = Array('d', [0, 0])

	iot.mqtt_subscribe_thread_start(pose_callback, sub_topic, qos)
	iot.mqtt_subscribe_thread_start(key_callback, 'key_cmd', 2)
	qt_plotter(setpoint_plot, ball_pose_plot, pid_plot, plot_terminate)
