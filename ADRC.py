#!/usr/bin/env python
# -*- coding: utf-8 -*-

class ADRC(object):
    # w0 – pulsacja graniczna pasma przenoszenia obserwatora,
    # w_n –żądana pulsacja drgań nietłumionych układu zamkniętego,
    # sigma – żądane tłumienie układu.
    def __init__(self, w0, b0, w_n, sigma=1, time_delta=0.01):
        # wartość sterowania z poprzedniego kroku
        self.u_p = 0
        # iksy z poprzedniego kroku
        self.x1_p = 0
        self.x2_p = 0
        self.x3_p = 0
        # wskaźnik jakości regulacji
        self.J = 0

        # parametry adrc
        # krok czasowy
        self.time_delta = time_delta
        self.b0 = b0

        # Parametry obserwatora
        self.b1 = 3 * w0
        self.b2 = 3 * w0**2
        self.b3 = w0**3

        # Nastawy regulatora
        self.kp = w_n * w_n
        self.kd = 2 * sigma * w_n

        self.deviation_history = [0] * 100

    # y - pozycja rzeczywista, yref - pozycja referencyjna,
    # saturacja opcjonalna, dlatego podawana jako kwargs
    def regulate(self, y, yref, sat_high, sat_low):
        # błąd estymacji
        err = y - self.x1_p
        # dyskretne rownania stanu (obliczanie estymat stanu)
        x1_t = self.x1_p + self.time_delta * (self.x2_p + self.b1 * err)
        x2_t = self.x2_p + \
            self.time_delta * (self.x3_p + self.b0 * self.u_p + self.b2 * err)
        x3_t = self.x3_p + self.time_delta * self.b3 * err
        # Uchyb regulacji
        e = yref - x1_t
        # Wskaznik jakosci
        self.J = self.J + e * e * self.time_delta
        # Prawo sterowania
        u = (1/self.b0) * (e*self.kp - self.kd*x2_t - x3_t)
        # zapisywanie parametrów na następny krok
        self.x1_p, self.x2_p, self.x3_p = x1_t, x2_t, x3_t


        self.deviation_history.append(abs(u))
        del self.deviation_history[0]

        srednia = sum(self.deviation_history)/100
        print('wartość sygnału sterującego przed saguracją:{}'.format(srednia))
        # saturacja
        if u > sat_high:
            u = sat_high
        elif u < sat_low:
            u = sat_low
        # zapisywanie u
        self.u_p = u
        return u
