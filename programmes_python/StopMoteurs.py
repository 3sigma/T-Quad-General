#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme d'arrêt des moteurs du robot T-Quad
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 1.1.2 - 30/01/2017
##################################################################################

# Import pour la détection du hostname
import socket

# Nom de l'hostname (utilisé ensuite pour savoir sur quel système
# tourne ce programme)
hostname = socket.gethostname()

# Imports pour la communication i2c avec l'Arduino Mega
from mega import Mega
mega = Mega(hostname = hostname)

tensionAlim = 7.4

def CommandeMoteurs(commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    global tensionAlim
    
    # L'ensemble pont en H + moteur pourrait ne pas être linéaire
    tensionArriereDroit = commandeArriereDroit
    tensionArriereGauche = commandeArriereGauche
    tensionAvantDroit = commandeAvantDroit
    tensionAvantGauche = commandeAvantGauche

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int_ArriereDroit = int(255 * tensionArriereDroit / tensionAlim)
    tension_int_ArriereGauche = int(255 * tensionArriereGauche / tensionAlim)
    tension_int_AvantDroit = int(255 * tensionAvantDroit / tensionAlim)
    tension_int_AvantGauche = int(255 * tensionAvantGauche / tensionAlim)

    # Saturation par sécurité
    if (tension_int_ArriereDroit > 255):
        tension_int_ArriereDroit = 255

    if (tension_int_ArriereDroit < -255):
        tension_int_ArriereDroit = -255

    if (tension_int_ArriereGauche > 255):
        tension_int_ArriereGauche = 255

    if (tension_int_ArriereGauche < -255):
        tension_int_ArriereGauche = -255

    if (tension_int_AvantDroit > 255):
        tension_int_AvantDroit = 255

    if (tension_int_AvantDroit < -255):
        tension_int_AvantDroit = -255

    if (tension_int_AvantGauche > 255):
        tension_int_AvantGauche = 255

    if (tension_int_AvantGauche < -255):
        tension_int_AvantGauche = -255

    # Commande PWM
    try:
        mega.moteursArriere(-tension_int_ArriereDroit, tension_int_ArriereGauche)
        mega.moteursAvant(-tension_int_AvantDroit, tension_int_AvantGauche)
    except:
        pass
        #print "Erreur moteurs"


CommandeMoteurs(0, 0, 0, 0)
        
