#!/usr/bin/env python
# -*- coding: utf-8 -*-

# dany plik rysuje wykres
import rospy
from std_msgs.msg import String
import time
import matplotlib.pyplot as plt

x_plt_values = []
def_heights = []
heights = []
t_start = 0

# dana funkcja uruchamia się po otrzymaniu przerwania z danymi
def callback(data):
	# sublimacja danych z wiadomości
	predefined_height, height = str(data.data).split(',')
	# dodawanie danych do listów
	x_plt_values.append(time.time()-t_start)
	def_heights.append(predefined_height)
	heights.append(height)


def plotter():
	global t_start
	print('to okno rysuje wykres')
	rospy.init_node('z_plotter', anonymous=True)
	rospy.Subscriber('z_plotdata', String, callback)
	# czas startowy
	t_start = time.time()
	# inicjalizacja częstotliwości odświeżania
	rate = rospy.Rate(10)
	while True:
		# rysowanie wykresu
		plt.figure(2)
		plt.clf()
		plt.ylabel('wysokosc')
		plt.xlabel('czas')
		plt.title('wykres wysokosci')
		plt.plot(x_plt_values, def_heights)
		plt.plot(x_plt_values, heights)
		plt.pause(0.001)
		# spranie do następnego
		rate.sleep()

if __name__ == '__main__':
	plotter()
