"""
	Author : Amoghavarsha S G
"""

from graph_plotter import qt_plotter
from multiprocessing import Process, Array, Value
from time import sleep
import json
from iot import IOT
import yaml


def pose_callback(client, userdata, msg):
	# print(json.loads(msg.payload.decode('UTF-8')))
	pose = json.loads(msg.payload.decode('UTF-8'))
	ball_pose = pose['ball_pose']
	if ball_pose:
		ball_pose_plot[:] = ball_pose[0]
	setpoint_plot[:] = pose['setpoint']

def key_callback(client, userdata, msg):
	key = json.loads(msg.payload.decode('UTF-8'))
	if key == 'q':
		plot_terminate.value = 1


if __name__ == '__main__':

	with open('broker_config.yaml', 'r') as config_file:
		broker_url = yaml.load(config_file, yaml.SafeLoader)['broker_url']

	iot = IOT(broker_url=broker_url)
	sub_topic = 'ball_set_pose'
	qos = 0

	plot_terminate = Value('i', 0)
	setpoint_plot = Array('i', [0, 0])
	ball_pose_plot = Array('i', [0, 0])

	iot.mqtt_subscribe_thread_start(pose_callback, sub_topic, qos)
	iot.mqtt_subscribe_thread_start(key_callback, 'key_cmd', 2)
	qt_plotter(setpoint_plot, ball_pose_plot, plot_terminate)
