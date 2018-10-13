#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme d'affichage des adresses IP et de la tension batterie du robot T-Quad
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 1.1.3 - 23/02/2017
##################################################################################

# Import généraux
from time import sleep
import signal
import sys
import numpy as np

# Imports pour l'écran OLED
from oled.serial import i2c
from oled.device import ssd1306
from oled.render import canvas
from PIL import ImageFont, ImageDraw

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Nom de l'hostname (utilisé ensuite pour savoir sur quel système
# tourne ce programme)
hostname = socket.gethostname()

# Imports pour la communication i2c avec l'Arduino Mega
from mega import Mega
mega = Mega(hostname = hostname)

# Tableau des pourcentages de temps restant en fonction de la tension batterie mesurée
# Il y a 200 points, correspondant aux tensions comprises entre 6.40 et 8.33 V avec 
# 2 décimales
pourcentages = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, \
    1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,4,4,4,4,4,5,5,5,6,6,6,7,7,7,8,9,9,10,11,11,12, \
    13,14,15,16,18,19,20,22,23,25,26,28,29,31,32,33,35,36,38,39,41,42,43,44,46,47,48,49,50,51,53,54,55,56, \
    57,58,59,60,61,62,62,63,64,65,66,67,67,68,69,70,71,71,72,73,73,74,75,76,76,77,78,78,79,79,80,81,81,82,82, \
    83,84,84,85,85,86,86,87,87,88,88,89,89,90,90,91,91,92,92,93,93,94,94,95,95,95,96,96,97,97,98,98,98,99,99,100,100]
# Pour éviter que le pourcentage n'augmente à cause du bruit de mesure
pourcentagePrec = 100

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

# Initialisation de l'écran OLED
if (hostname == "pcduino"):
    serial  = i2c(port=2, address=0x3C)
    font_path = '/usr/share/fonts/truetype/ttf-dejavu/DejaVuSans.ttf'
elif (hostname == "raspberrypi"):
    serial  = i2c(port=1, address=0x3C)
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
else:
    # pcDuino par défaut
    serial  = i2c(port=2, address=0x3C)
    font_path = '/usr/share/fonts/truetype/ttf-dejavu/DejaVuSans.ttf'

ssd1306Started = False
while not ssd1306Started:
    try:
        device = ssd1306(serial)
        ssd1306Started = True
    except:
        sleep(1)
        print "Probleme demarrage ssd1306"
        pass

font12 = ImageFont.truetype(font_path, 12)
font20 = ImageFont.truetype(font_path, 20)
font40 = ImageFont.truetype(font_path, 40)

# Gestion des interruptions
def signal_handler(signal, frame):
    global device
    print "Arret du programme"
    with canvas(device) as draw:
        draw.text((0, 5), "", font=font12, fill="white")
    sys.exit(0)


pasDeWifi = False
try:
    adresseWifi = get_ip_address('wlan0')
except:
    pasDeWifi = True

pasDEthernet = False
try:
    adresseEthernet = get_ip_address('eth0')
except:
    pasDEthernet = True


       
while True:
    # Empêche les interruptions par CTRL-C et kill pendant la période de gestion de l'i2c
    # Si l'écran est interrompu à ce moment-là, il reste dans un état indéfini et
    # il est nécessaire de faire une extinction / réallumage du robot
    s1 = signal.signal(signal.SIGINT, signal.SIG_IGN)
    s2 = signal.signal(signal.SIGTERM, signal.SIG_IGN)
    try:
        with canvas(device) as draw:
            if pasDeWifi:
                draw.text((0, 5), "Pas de Wifi", font=font12, fill="white")
            else:
                draw.text((0, 5), "Wifi:", font=font12, fill="white")
                draw.text((0, 20), adresseWifi, font=font12, fill="white")
            if pasDEthernet:
                draw.text((0, 35), "Pas d'Ethernet", font=font12, fill="white")
            else:
                draw.text((0, 35), "Ethernet:", font=font12, fill="white")
                draw.text((0, 50), adresseEthernet, font=font12, fill="white")
    except:
        print "Erreur affichage adresses réseau"
        pass

    # Réactive les interruptions
    signal.signal(signal.SIGINT, s1)
    signal.signal(signal.SIGTERM, s2)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    sleep(5)
    
    # Empêche les interruptions par CTRL-C et kill pendant la période de gestion de l'i2c
    s1 = signal.signal(signal.SIGINT, signal.SIG_IGN)
    s2 = signal.signal(signal.SIGTERM, signal.SIG_IGN)
    try:
        VBat = float(mega.read_battery_millivolts()) / 1000.
        # Saturation de VBat pour que reste comprise entre 6.40 et 8.33
        VBatSat = np.clip(VBat, 6.4, 8.33)
        # Récupération de l'indice dans le tableau des pourcentages
        iPourcent = int(round((VBatSat-6.4)*100))
        # Récupération du pourcentage ; on fait en sorte qu'il n'augmente jamais
        pourcentage = min(pourcentages[iPourcent], pourcentagePrec)
        pourcentagePrec = pourcentage
        # Affichage
        with canvas(device) as draw:
            draw.text((0, 5), "Bat:", font=font12, fill="white")
            draw.text((70, 5), ("%d" % pourcentage) + " %", font=font12, fill="white")
            draw.text((0, 20), "%.1f" % VBat, font=font40, fill="white")
            draw.text((75, 37), "V", font=font20, fill="white")
    except:
        print "Erreur affichage tension batterie"
        pass
            
    # Réactive les interruptions
    signal.signal(signal.SIGINT, s1)
    signal.signal(signal.SIGTERM, s2)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    sleep(5)
            
            
            
            
