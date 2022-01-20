#!/usr/bin/env python
# -*- coding: utf-8 -*-

# dany plik rysuje wykres
import rospy
from std_msgs.msg import String
import time
import matplotlib.pyplot as plt

t_plt_values = []
def_x_poses = []
x_poses = []
t_start = 0

# dana funkcja uruchamia się po otrzymaniu przerwania z danymi
def callback(data):
	# sublimacja danych z wiadomości
	predefined_x, x = str(data.data).split(',')
	# dodawanie danych do listów
	t_plt_values.append(time.time()-t_start)
	def_x_poses.append(predefined_x)
	x_poses.append(x)


def plotter():
	global t_start
	print('to okno rysuje wykres')
	rospy.init_node('x_plotter', anonymous=True)
	rospy.Subscriber('x_plotdata', String, callback)
	# czas startowy
	t_start = time.time()
	# inicjalizacja częstotliwości odświeżania
	rate = rospy.Rate(10)
	while True:
		# rysowanie wykresu
		plt.figure(2)
		plt.clf()
		plt.ylabel('x_pozycja')
		plt.xlabel('czas')
		plt.title('wykres x')
		plt.plot(t_plt_values, def_x_poses)
		plt.plot(t_plt_values, x_poses)
		plt.pause(0.001)
		# spranie do następnego
		rate.sleep()

if __name__ == '__main__':
	plotter()
