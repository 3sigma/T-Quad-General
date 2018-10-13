#!/usr/bin/python
# -*- coding: utf-8 -*-

######################################################################################
# Programme d'affichage de la distance et de l'�tat des interrupteurs du robot T-Quad
# disponible � l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 1.1.2 - 30/01/2017
######################################################################################

# Importe les fonctions Arduino pour Python
from pyduino import *

# Import g�n�raux
from time import sleep
import signal
import sys

# Imports pour l'�cran OLED
from oled.serial import i2c
from oled.device import ssd1306
from oled.render import canvas
from PIL import ImageFont, ImageDraw

# Nom de l'hostname (utilis� ensuite pour savoir sur quel syst�me
# tourne ce programme)
hostname = socket.gethostname()

# Imports pour la communication i2c avec l'Arduino Mega
from mega import Mega
mega = Mega(hostname = hostname)

# Capteur de distance
pulse_start = 0
pulse_end = 0
pulse_duration = 0
last_pulse_duration = 0
distance = 0

if (hostname == "pcduino"):
    trig = 10
    echo = 13
    # Initialisation
    pinMode(trig, OUTPUT)
    pinMode(echo, INPUT)
elif (hostname == "raspberrypi"):
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    trig = 3 # GPIO22
    echo = 23
    # Initialisation
    pinMode(trig, OUTPUT)
    GPIO.setup(echo,GPIO.IN)
else:
    # pcDuino par d�faut
    trig = 10
    echo = 13
    # Initialisation
    pinMode(trig, OUTPUT)
    pinMode(echo, INPUT)
    
digitalWrite(trig, LOW)
print "Attente du capteur de distance"
time.sleep(2)

digitalWrite(trig, HIGH)
time.sleep(0.00001)
digitalWrite(trig, LOW)

# Initialisation de l'�cran OLED
if (hostname == "pcduino"):
    serial  = i2c(port=2, address=0x3C)
    font_path = '/usr/share/fonts/truetype/ttf-dejavu/DejaVuSans.ttf'
elif (hostname == "raspberrypi"):
    serial  = i2c(port=1, address=0x3C)
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
else:
    # pcDuino par d�faut
    serial  = i2c(port=2, address=0x3C)
    font_path = '/usr/share/fonts/truetype/ttf-dejavu/DejaVuSans.ttf'

device = ssd1306(serial)

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


       
while True:
    # Emp�che les interruptions par CTRL-C et kill pendant la p�riode de gestion de l'i2c
    # Si l'�cran est interrompu � ce moment-l�, il reste dans un �tat ind�fini et
    # il est n�cessaire de faire une extinction / r�allumage du robot
    s1 = signal.signal(signal.SIGINT, signal.SIG_IGN)
    s2 = signal.signal(signal.SIGTERM, signal.SIG_IGN)

    # Lecture des interrupteurs
    try:
        strEtatSW = bin(mega.read_SW())[2:].zfill(2) # Vaut 0b10 si SW1=1 et SW2=0
        # Extraction des 2 bits repr�sentant l'�tat des interrupteurs
        strEtatSW1 = strEtatSW[0]
        strEtatSW2 = strEtatSW[1]
    except:
        print("Erreur lecture interrupteurs")

    # Calcul de la distance mesur�e par le capteur ultrason
    digitalWrite(trig, HIGH)
    time.sleep(0.00001)
    digitalWrite(trig, LOW)
    
    if (hostname == "pcduino"):
        pulse_duration = 0
        while (digitalRead(echo) == 0):
            pulse_start = time.time()

        while (digitalRead(echo) == 1) and (pulse_duration < 0.01166):
            pulse_end = time.time()
            last_pulse_duration = pulse_duration
            pulse_duration = pulse_end - pulse_start
            
    elif (hostname == "raspberrypi"):
        pulse_duration = 0
        while (GPIO.input(echo) == 0):
            pulse_start = time.time()

        while (GPIO.input(echo) == 1) and (pulse_duration < 0.01166):
            pulse_end = time.time()
            last_pulse_duration = pulse_duration
            pulse_duration = pulse_end - pulse_start
            
    else:
        pulse_duration = 0
        while (digitalRead(echo) == 0):
            pulse_start = time.time()

        while (digitalRead(echo) == 1) and (pulse_duration < 0.01166):
            pulse_end = time.time()
            last_pulse_duration = pulse_duration
            pulse_duration = pulse_end - pulse_start
                
    distance = last_pulse_duration * 17150
    distance = round(distance, 0)

    try:
        with canvas(device) as draw:
            draw.text((0, 5), "SW1: " + strEtatSW1, font=font12, fill="white")
            draw.text((0, 20), "SW2: " + strEtatSW2, font=font12, fill="white")
            draw.text((0, 35), ("Dist.: %d cm" % distance), font=font12, fill="white")
    except:
        print "Erreur affichage"
        pass

    # R�active les interruptions
    signal.signal(signal.SIGINT, s1)
    signal.signal(signal.SIGTERM, s2)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    sleep(0.5)
                
            
            
            
