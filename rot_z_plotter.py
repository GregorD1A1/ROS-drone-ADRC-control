#!/usr/bin/env python
# -*- coding: utf-8 -*-

# dany plik rysuje wykres
import rospy
from std_msgs.msg import String
import time
import matplotlib.pyplot as plt

t_plt_values = []
def_rot_z_poses = []
rot_z_poses = []
t_start = 0

# dana funkcja uruchamia się po otrzymaniu przerwania z danymi
def callback(data):
	# sublimacja danych z wiadomości
	predefined_rot_z, rot_z = str(data.data).split(',')
	# dodawanie danych do listów
	t_plt_values.append(time.time()-t_start)
	def_rot_z_poses.append(predefined_rot_z)
	rot_z_poses.append(rot_z)


def plotter():
	global t_start
	print('to okno rysuje wykres')
	rospy.init_node('rot_z_plotter', anonymous=True)
	rospy.Subscriber('rot_z_plotdata', String, callback)
	# czas startowy
	t_start = time.time()
	# inicjalizacja częstotliwości odświeżania
	rate = rospy.Rate(10)
	while True:
		# rysowanie wykresu
		plt.figure(2)
		plt.clf()
		plt.ylabel('kat_z')
		plt.xlabel('czas')
		plt.title('wykres katu z')
		plt.plot(t_plt_values, def_rot_z_poses)
		plt.plot(t_plt_values, rot_z_poses)
		plt.pause(0.001)
		# spranie do następnego
		rate.sleep()

if __name__ == '__main__':
	plotter()
