#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rospy import Publisher
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String
from ardrone_autonomy.msg import Navdata
import time
import random
from ADRC import ADRC
from PID import PIDRegulator


class ReferenceValueGenerator(object):
	def __init__(self, x_low, x_high, y_low, y_high, z_low, z_high, \
			rot_z_low=0, rot_z_high=0, change_period=3):
		self.last_change_time = time.time()
		self.change_per = change_period
		self.x_ref = 0
		self.x_low = x_low
		self.x_high = x_high
		self.y_ref = 0
		self.y_low = y_low
		self.y_high = y_high
		self.z_ref = 1
		self.z_low = z_low
		self.z_high = z_high
		self.rot_z_ref = 0
		self.rot_z_low = rot_z_low
		self.rot_z_high = rot_z_high

	def generate(self):
		time_from_pos_change = time.time() - self.last_change_time
		if time_from_pos_change >= self.change_per:
			self.x_ref = random.uniform(self.x_low, self.x_high)
			self.y_ref = random.uniform(self.y_low, self.y_high)
			self.z_ref = random.uniform(self.z_low, self.z_high)
			self.rot_z_ref = random.uniform(self.rot_z_low, self.rot_z_high)
			#aktualizujemy czas
			self.last_change_time += self.change_per


class ParameterScan(object):
	def __init__(self):
		self.sonar_height = 0
		self.vx = 0
		self.vy = 0
		self.vz = 0
		self.x_pos = 0
		self.y_pos = 0
		self.z_pos = 0
		self.z_rotation = 0

	# dana funkcja uruchamia się po otrzymaniu przerwania z danymi
	def sonar_callback(self, msg):
		# pobieranie wysokosci z czujnika
		self.sonar_height = msg.range

	# dana funkcja uruchamia się po otrzymaniu przerwania z danymi
	def nav_callback(self, msg):
		self.x_velocity = msg.vx
		self.y_velocity = msg.vy
		self.z_velocity = msg.vz
		self.z_rotation = msg.rotZ
		# integrujemy po prędkości, dzielimy przez 200 bo mamy 200 przerywań
		# na sekundę, wynik otrzymujemy w mm; dzielimy przez 1000 by dostać w m
		self.x_pos += self.x_velocity / 1000 / 200
		self.y_pos += self.y_velocity / 1000 / 200
		self.z_pos += self.z_velocity / 1000 / 200


def publish_plot_data(*args):
	for arg in args:
		(publisher, des_val, val) = arg
		plotdata = publisher.data_class
		plotdata = str(des_val) + ',' + str(val)
		publisher.publish(plotdata)


def main():
	# inicjalizacja nodu sterującego i jego publikującego topiku
	rospy.init_node('drone_controller')
	control_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	x_plt_pub = rospy.Publisher('x_plotdata', String, queue_size=10)
	y_plt_pub = rospy.Publisher('y_plotdata', String, queue_size=10)
	z_plt_pub = rospy.Publisher('z_plotdata', String, queue_size=10)
	rot_z_plt_pub = rospy.Publisher('rot_z_plotdata', \
		String, queue_size=10)
	#pos_generator = ReferenceValueGenerator(change_period=5, x_low=-2,
	#	x_high=2, y_low=-2, y_high=2, z_low=0.3, z_high=2.8)
	pos_generator = ReferenceValueGenerator(change_period=30, x_low=-2,
		x_high=2, y_low=-2, y_high=2, z_low=-0.7, z_high=1, rot_z_low=0, rot_z_high=0)

	#x_controller = PIDRegulator(p=1, i=0.17, d=8)
	#y_controller = PIDRegulator(p=1, i=0.17, d=8)
	#z_controller = PIDRegulator(p=1, i=0.17, d=8)
	rot_z_controller = PIDRegulator(p=0.15, i=0.001, d=0.07, history_len=10)
	x_controller = ADRC(w0=16, b0=1.8, w_n=2)
	y_controller = ADRC(w0=16, b0=1.8, w_n=2)
	z_controller = ADRC(w0=17, b0=0.3, w_n=4)
	#rot_z_controller = ADRC(w0=1.7, b0=1.8, w_n=0.2)

	param_reader = ParameterScan()
	# inicjalizacja nasłuchu czyjnika wysokości w funkcji przerywania
	rospy.Subscriber('/sonar_height', Range, param_reader.sonar_callback)
	# inicjalizacja nasłuchu czyjnika wysokości w funkcji przerywania
	rospy.Subscriber('/ardrone/navdata', Navdata, param_reader.nav_callback)
	# inicjalizacja częstotliwości regulatora
	rate = rospy.Rate(100)

	while True: #not rospy.is_shutdown:
		# wysyłanie danych sterujących wysokością
		vel_data = Twist()
		#vel_data.linear.x = x_controller.regulate(
		#	deviation=pos_generator.x_ref-param_reader.x_pos)
		vel_data.linear.x = x_controller.regulate(
			y=param_reader.x_pos, yref=pos_generator.x_ref, sat_high=1.0, sat_low=-1.0)
		vel_data.linear.y = y_controller.regulate(
			y=param_reader.y_pos, yref=pos_generator.y_ref, sat_high=1.0, sat_low=-1.0)
		#vel_data.linear.y = y_controller.regulate(
		#	deviation=pos_generator.y_ref-param_reader.y_pos)
		#vel_data.linear.z = z_controller.regulate(
		#	deviation=pos_generator.z_ref-param_reader.sonar_height)
		vel_data.linear.z = z_controller.regulate(
			y=param_reader.z_pos, yref=pos_generator.z_ref, sat_high=1.0, sat_low=-1.0)
		vel_data.angular.z = rot_z_controller.regulate(
			deviation=pos_generator.rot_z_ref-param_reader.z_rotation)
		#vel_data.angular.z = rot_z_controller.regulate(
		#	y=param_reader.z_rotation, yref=pos_generator.rot_z_ref, sat_high=1.0, sat_low=-1.0)
		print(vel_data)
		control_publisher.publish(vel_data)
		# wysyłanie danych na nod wykresu
		publish_plot_data((x_plt_pub, pos_generator.x_ref, param_reader.x_pos),
			(y_plt_pub, pos_generator.y_ref, param_reader.y_pos),
			(z_plt_pub, pos_generator.z_ref, param_reader.z_pos),
			(rot_z_plt_pub, pos_generator.rot_z_ref, param_reader.z_rotation))
		# generacja nowej pozycji zadanej
		pos_generator.generate()
		#spanie do określonego interwału
		rate.sleep()
	#rospy.spin()


try:
    main()
except rospy.ROSInterruptException:
    pass
