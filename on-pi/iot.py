"""
	Author: Amoghavarsha S G
"""

import paho.mqtt.client as mqtt
from time import sleep

class IOT:
	def __init__(self, broker_url, broker_port=1883):
		self.broker_url = broker_url
		self.broker_port = broker_port
		self.pub_client_exits = False

	@staticmethod
	def on_connect(client, userdata, flags, rc, sub_topic, qos):
		print(f"Connected With Result Code {rc}")
		(rc, mid) = client.subscribe(sub_topic, qos=qos)
		if not rc:
			print('subscribed to topic!')
		return rc

	@staticmethod
	def sub_on_connect(func, sub_topic, qos):
		def wrapper(client, userdata, flags, rc):
			return func(client, userdata, flags, rc, sub_topic, qos)
		return wrapper

	def mqtt_subscribe_thread_start(self, on_message, sub_topic, qos):
		"""
			This function subscribes to any mqtt topic given on "sub_topic" and makes "on_message" as on_message cb function
		"""
		try:
			client = mqtt.Client()
			on_connect = IOT.sub_on_connect(IOT.on_connect, sub_topic, qos)
			client.on_connect = on_connect
			client.on_message = on_message
			client.connect(self.broker_url, self.broker_port)
			client.loop_start()
			return 0
		except Exception as e:
			print(e)
			return -1

	def mqtt_publish_reuse_client(self, pub_topic, payload, qos, timeout=None):
		try:
			if not self.pub_client_exits:
				self.pub_client = mqtt.Client()
				self.pub_client.connect(self.broker_url, self.broker_port)
				self.pub_client.loop_start()
				self.pub_client_exits = True

			pub_info = self.pub_client.publish(pub_topic, payload, qos=qos, retain=False)
			if timeout:
				sleep(timeout)
			else:
				pub_info.wait_for_publish()
			return pub_info[0]
		except Exception as e:
			print(e)
			return -1

	def mqtt_publish(self, pub_topic, payload, qos):
		"""
			This function publish's on any mqtt topic given on "pub_topic" with message given as "payload"
		"""
		try:
			client = mqtt.Client()
			client.connect(self.broker_url, self.broker_port)
			client.loop_start()
			pub_info = client.publish(pub_topic, payload, qos=qos, retain=False)
			pub_info.wait_for_publish()
			client.loop_stop()
			client.disconnect()
			return pub_info[0]
		except Exception as e:
			print(e)
			return -1