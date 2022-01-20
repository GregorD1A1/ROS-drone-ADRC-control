#!/usr/bin/env python
# -*- coding: utf-8 -*-


class PIDRegulator(object):
	def __init__(self, p, i=0, d=0, history_len=5):
		self.deviation_history = [0] * history_len
		self.p = p
		self.i = i
		self.d = d

	def regulate(self, deviation):
		# obliczenia do częśći I
		self.deviation_history.append(deviation)
		del self.deviation_history[0]
		integral = sum(self.deviation_history)
		# obliczenia częśći D
		deriative = self.deviation_history[-1] - self.deviation_history[-2]
		# końcowe obliczenie
		control = deviation * self.p + integral * self.i + deriative * self.d
		return control
