#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de communication en i2c avec la carte Arduino Mega du robot T-Quad
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 1.2.0 - 31/10/2017
##################################################################################

import smbus
import struct

class Mega(object):
    def __init__(self, hostname = "pcduino"):
        if (hostname == "pcduino"):
            self.bus = smbus.SMBus(2)
        elif (hostname == "raspberrypi"):
            self.bus = smbus.SMBus(1)
        else:
            # pcDuino par défaut
            self.bus = smbus.SMBus(2)

    def command(self, cmd, byte_array):
        bytes = [cmd] + byte_array
        self.bus.write_i2c_block_data(20, 1, bytes)
        while(not self.command_done()):
            pass

    def command_done(self):
        self.bus.write_byte(20,0)
        return self.bus.read_byte(20) == 0

    def read_unpack(self, size, format):
        self.bus.write_byte(20,2)
        byte_list = []
        for n in range(0,size):
            byte_list.append(self.bus.read_byte(20))
        return struct.unpack(format,bytes(bytearray(byte_list)))

    def shiftu2s8(self, x):
        return x - 128

    def moteurArriereDroit(self, value):
        self.command(1, map(ord, list(struct.pack('h', value))))

    def moteurArriereGauche(self, value):
        self.command(2, map(ord, list(struct.pack('h', value))))

    def moteurAvantDroit(self, value):
        self.command(3, map(ord, list(struct.pack('h', value))))

    def moteurAvantGauche(self, value):
        self.command(4, map(ord, list(struct.pack('h', value))))

    def moteursArriere(self, ard, arg):
        self.command(5, map(ord, list(struct.pack('hh', ard, arg))))

    def moteursAvant(self, avd, avg):
        self.command(6, map(ord, list(struct.pack('hh', avd, avg))))

    def moteursCRC(self, crc_ar, crc_av):
        self.command(14, map(ord, list(struct.pack('hh', crc_ar, crc_av))))

    def read_codeursArriereDeltaPos(self):
        self.command(7, [])
        codeursArriereDeltaPos = self.read_unpack(2, "H")[0]
        codeurArriereDroitDeltaPos = self.shiftu2s8(codeursArriereDeltaPos >> 8)
        codeurArriereGaucheDeltaPos = self.shiftu2s8(0xFF & codeursArriereDeltaPos)
        return [codeurArriereDroitDeltaPos, codeurArriereGaucheDeltaPos]

    def read_codeursAvantDeltaPos(self):
        self.command(8, [])
        codeursAvantDeltaPos = self.read_unpack(2, "H")[0]
        codeurAvantDroitDeltaPos = self.shiftu2s8(codeursAvantDeltaPos >> 8)
        codeurAvantGaucheDeltaPos = self.shiftu2s8(0xFF & codeursAvantDeltaPos)
        return [codeurAvantDroitDeltaPos, codeurAvantGaucheDeltaPos]

    def read_codeursDeltaPos(self):
        self.command(9, [])
        codeursDeltaPos = self.read_unpack(4, "I")[0]
        codeurArriereDroitDeltaPos = self.shiftu2s8(codeursDeltaPos >> 24)
        codeurArriereGaucheDeltaPos = self.shiftu2s8(0xFF & (codeursDeltaPos >> 16))
        codeurAvantDroitDeltaPos = self.shiftu2s8(0xFF & (codeursDeltaPos >> 8))
        codeurAvantGaucheDeltaPos = self.shiftu2s8(0xFF & codeursDeltaPos)
        return [codeurArriereDroitDeltaPos, codeurArriereGaucheDeltaPos, codeurAvantDroitDeltaPos, codeurAvantGaucheDeltaPos]

    def line_read(self, port):
        self.command(10, [port])
        return self.read_unpack(2, "H")[0]

    def read_battery_millivolts(self):
        self.command(11, [])
        return self.read_unpack(2, "H")[0]

    def read_SW(self):
        self.command(12, [])
        return self.read_unpack(1, "B")[0]

    def analog_read(self, port):
        self.command(13, [port])
        return self.read_unpack(2, "H")[0]

    def firmwareOK(self):
        self.command(15, [])
        if self.read_unpack(2, "H")[0] == 1:
            return True
        else:
            return False

    def read_distance(self):
        self.command(16, [])
        return self.read_unpack(2, "H")[0]

    def test_read8(self):
        try:
            self.read_unpack(8, "cccccccc")
        except:
            print "Erreur test_read8"

    def test_write8(self):
        try:
            self.bus.write_i2c_block_data(20, 0, [0,0,0,0,0,0,0,0])
        except:
            print "Erreur test_write8"

