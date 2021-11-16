from graph_plotter import qt_plotter
from multiprocessing import Process, Array, Value
from time import sleep
import json
from iot import IOT
import yaml

def callback(client, userdata, msg):
	# print(json.loads(msg.payload.decode('UTF-8')))
	pose = json.loads(msg.payload.decode('UTF-8'))
	ball_pose_plot[:] = pose['ball_pose'][0]
	setpoint_plot[:] = pose['setpoint']

if __name__ == '__main__':

	with open('broker_config.yaml', 'r') as config_file:
		broker_url = yaml.load(config_file, yaml.SafeLoader)['broker_url']

	iot = IOT(broker_url=broker_url)
	sub_topic = 'ball_set_pose'
	qos = 0

	plot_terminate = Value('i', 0)
	setpoint_plot = Array('i', [0, 0])
	ball_pose_plot = Array('i', [0, 0])

	iot.mqtt_subscribe_thread_start(callback, sub_topic, qos)
	qt_plotter(setpoint_plot, ball_pose_plot, plot_terminate)
