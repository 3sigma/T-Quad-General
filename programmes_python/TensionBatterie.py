#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de mesure de la décharge de la batterie du robot T-Quad,
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 1.1.1 - 30/01/2017
##################################################################################

# Importe les fonctions Arduino pour Python
from pyduino import *

# Imports Généraux
import time, sched
import os
import threading
import signal
import json
import sys

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Pour le serveur de socket
import tornado.httpserver
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
import tornado.websocket
import tornado.template

# Nom de l'hostname (utilisé ensuite pour savoir sur quel système
# tourne ce programme)
hostname = socket.gethostname()

# Imports pour la communication i2c avec l'Arduino Mega
from mega import Mega
mega = Mega(hostname = hostname)

T0 = time.time()
dt = 0.01
i = 0
tprec = time.time()
tdebut = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)

idecimLectureTension = 0
decimLectureTension = 6000
decimErreurLectureTension = 10

# Mesure de la tension de la batterie
lectureTensionOK = False
tensionAlim = 7.4
while not lectureTensionOK:
    try:
        tensionAlim = float(mega.read_battery_millivolts()) / 1000.
        lectureTensionOK = True
    except:
        print("Erreur lecture tension")


#--- setup --- 
def setup():
    pass
    
# -- fin setup -- 
 
# -- loop -- 
def loop():
    global i, T0
    i = i+1
    s.enterabs( T0 + (i * dt), 1, MesureTension, ())
    s.run()
# -- fin loop --

def MesureTension():
    global tdebut, idecimLectureTension, decimLectureTension, decimErreurLectureTension, tensionAlim
    
    tdebut = time.time()
            
    # Lecture de la tension d'alimentation
    if idecimLectureTension >= decimLectureTension:
        try:
            tensionAlim = float(mega.read_battery_millivolts()) / 1000.
            idecimLectureTension = 0
        except:
            # On recommence la lecture dans decimErreurLectureTension * dt
            idecimLectureTension = idecimLectureTension - decimErreurLectureTension
            #print("Erreur lecture tension dans Loop")
    else:
        idecimLectureTension = idecimLectureTension + 1    
    
        
    #print time.time() - tdebut

        
def emitData():
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    #delay(5000)
    while not noLoop: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 60000)
        self.callback.start()
    

    def on_message(self, message):
        pass

    def on_close(self):
        global socketOK
        print 'connection closed...'
        socketOK = False

    def sendToSocket(self):
        global tensionAlim
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({'Temps':("%.2f" % tcourant), \
                                'tensionAlim':("%.2f" % tensionAlim), \
                                'Raw':("%.2f" % tcourant) \
                                + "," + ("%.2f" % tensionAlim) \
                                })
                                
        if socketOK:
            try:
                self.write_message(aEnvoyer)
            except:
                pass
            
    def check_origin(self, origin):
        # Voir http://www.tornadoweb.org/en/stable/websocket.html#tornado.websocket.WebSocketHandler.check_origin
        # et http://www.arundhaj.com/blog/tornado-error-during-websocket-handshake.html
        return True        

    
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
    
application = tornado.web.Application([
    (r'/ws', WSHandler)
])

def startTornado():
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9090)
    tornado.ioloop.IOLoop.instance().start()


# Gestion du CTRL-C
def signal_handler(signal, frame):
    print 'Sortie du programme'
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

#--- obligatoire pour lancement du code -- 
if __name__=="__main__": # pour rendre le code executable 
    setup() # appelle la fonction setup
    print "Setup done."
    
    th = threading.Thread(None, emitData, None, (), {})
    th.daemon = True
    th.start()
    
    print "Starting Tornado."
    try:
        print "Connect to ws://" + get_ip_address('eth0') + ":9090/ws with Ethernet."
    except:
        pass
        
    try:
        print "Connect to ws://" + get_ip_address('wlan0') + ":9090/ws with Wifi."
    except:
        pass
    socketOK = False
    startTornado()


