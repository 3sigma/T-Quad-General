/******************************************************************************
Firmware de l'Arduino Mega des robots T-Quad avec carte de communication
(pcDuino, Raspberry Pi,...), disponibles à l'adresse:
http://boutique.3sigma.fr/12-robots
Conçu pour la version de matériel 1.2 mais fonctionne également
avec la version de matériel 1.1

Auteur: 3Sigma
Version 1.2.0 - 31/10/2017
*******************************************************************************/

// Inclusion d'une bibliothèque permettant l'exécution à cadence fixe
// d'une partie du programme. Télécharger à l'adresse http://www.3sigma.fr/telechargements/FlexiTimer2.zip
// et décompresser dans le sous-répertoire « libraries » de votre installation Arduino
// Pour plus de détails, voir les pages (en anglais): 
// http://www.arduino.cc/playground/Main/FlexiTimer2
// https://github.com/wimleers/flexitimer2
#include <FlexiTimer2.h>

// Inclusion d'une bibliothèque permettant de lire et d'écrire plus rapidement sur les entrées-sorties digitales.
// Télécharger à l'adresse http://www.3sigma.fr/telechargements/digitalWriteFast.zip
// et décompresser dans le sous-répertoire « libraries » de votre installation Arduino
#include <digitalWriteFast.h> 

// Inclusion d'une bibliothèque permettant de gérer les interruptions externes
// et le "PinChange Interrupts"
#include <EnableInterrupt.h>

// Inclusion d'une bibliothèque pour la gestion du capteur de distance
#include <NewPing.h>

// Inclusion de la bibliothèque permettant de gérer l'i2c
#include "Slave.h"
Slave slave;

// Etat des interrupteurs
int SW1State;
int SW2State;

// Valeur des capteurs de suivi de ligne
float lineSensorValue1 = 0; 
float lineSensorValue2 = 0;
float lineSensorValue3 = 0;

// Moyenne glissante sur les mesures des codeurs
// Attention: augmenter cette valeur conduirait à faire de nombreux changements
// dans le type des données et également du côté des programmes en Python
#define Nmoy 1

// Définitions et déclarations pour le codeur incrémental du moteur arrière droit
#define codeurArriereDroitPinA 2
#define codeurArriereDroitPinB 8
volatile int8_t ticksCodeurArriereDroit = 0;
int8_t codeurArriereDroitDeltaPos;
uint8_t indiceTicksCodeurArriereDroit = 0;
int8_t ticksCodeurArriereDroitTab[Nmoy];

// Définitions et déclarations pour le codeur incrémental du moteur arrière gauche
#define codeurArriereGauchePinA 3
#define codeurArriereGauchePinB 9
volatile int8_t ticksCodeurArriereGauche = 0;
int8_t codeurArriereGaucheDeltaPos;
uint8_t indiceTicksCodeurArriereGauche = 0;
int8_t ticksCodeurArriereGaucheTab[Nmoy];

// Définitions et déclarations pour le codeur incrémental du moteur avant droit
#define codeurAvantDroitPinA 19
#define codeurAvantDroitPinB 49
volatile int8_t ticksCodeurAvantDroit = 0;
int8_t codeurAvantDroitDeltaPos;
uint8_t indiceTicksCodeurAvantDroit = 0;
int8_t ticksCodeurAvantDroitTab[Nmoy];

// Définitions et déclarations pour le codeur incrémental du moteur avant gauche
#define codeurAvantGauchePinA 18
#define codeurAvantGauchePinB 23
volatile int8_t ticksCodeurAvantGauche = 0;
int8_t codeurAvantGaucheDeltaPos;
uint8_t indiceTicksCodeurAvantGauche = 0;
int8_t ticksCodeurAvantGaucheTab[Nmoy];

// Définitions et déclarations pour les moteurs à courant continu.
#define directionMoteurArriereDroit  4
#define directionBMoteurArriereDroit  36
#define pwmMoteurArriereDroit  5
#define mesurePWM_MoteurArriereDroit  13
#define mesureCourant_MoteurArriereDroit 0

#define directionMoteurArriereGauche  7
#define directionBMoteurArriereGauche  34
#define pwmMoteurArriereGauche  6
#define mesurePWM_MoteurArriereGauche  12
#define mesureCourant_MoteurArriereGauche 1

#define directionMoteurAvantDroit  30
#define directionBMoteurAvantDroit  33
#define pwmMoteurAvantDroit  44
#define mesurePWM_MoteurAvantDroit  15
#define mesureCourant_MoteurAvantDroit 8

#define directionMoteurAvantGauche  32
#define directionBMoteurAvantGauche  31
#define pwmMoteurAvantGauche  46
#define mesurePWM_MoteurAvantGauche  14
#define mesureCourant_MoteurAvantGauche 9

// Interrupteurs
#define SW1 48
#define SW2 47

// Définitions et déclarations pour le capteur de distance
#define TRIGGER_PIN  11  // Envoi de l'impulsion ultrason
#define ECHO_PIN     10  // Réception de l'écho
#define MAX_DISTANCE 200 // Distance max en cm
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Certaines parties du programme sont exécutées à cadence fixe grâce à la bibliothèque FlexiTimer2.
// Cadence d'échantillonnage en ms
#define CADENCE_MS 10
volatile double dt = CADENCE_MS/1000.;

// Déclarations pour la commande des moteurs
double omegaArriereDroit = 0.;
double omegaArriereGauche = 0.;
double omegaAvantDroit = 0.;
double omegaAvantGauche = 0.;

int16_t commandeArriereDroit = 0;
int16_t commandeArriereGauche = 0;
int16_t commandeAvantDroit = 0;
int16_t commandeAvantGauche = 0;
int16_t commandeArriereDroitPrec = 0;
int16_t commandeArriereGauchePrec = 0;
int16_t commandeAvantDroitPrec = 0;
int16_t commandeAvantGauchePrec = 0;

float tensionAlim = 7.4;


// Initialisations
void setup(void) {
  
  // Initialisation du bus I2C
  slave.init(20);

  // Interrupteurs
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  
  // Codeur incrémental moteur arrière droit
  pinMode(codeurArriereDroitPinA, INPUT_PULLUP);
  pinMode(codeurArriereDroitPinB, INPUT_PULLUP);
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurArriereDroitPinA (définie à la fin du programme)
  enableInterrupt(codeurArriereDroitPinA, GestionInterruptionCodeurArriereDroitPinA, CHANGE);

  // Codeur incrémental moteur arrière gauche
  pinMode(codeurArriereGauchePinA, INPUT_PULLUP);
  pinMode(codeurArriereGauchePinB, INPUT_PULLUP);
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurArriereGauchePinA (définie à la fin du programme)
  enableInterrupt(codeurArriereGauchePinA, GestionInterruptionCodeurArriereGauchePinA, CHANGE);

  // Codeur incrémental moteur avant droit
  pinMode(codeurAvantDroitPinA, INPUT_PULLUP);
  pinMode(codeurAvantDroitPinB, INPUT_PULLUP);
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurAvantDroitPinA (définie à la fin du programme)
  enableInterrupt(codeurAvantDroitPinA, GestionInterruptionCodeurAvantDroitPinA, CHANGE);

  // Codeur incrémental moteur avant gauche
  pinMode(codeurAvantGauchePinA, INPUT_PULLUP);
  pinMode(codeurAvantGauchePinB, INPUT_PULLUP);
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurAvantGauchePinA (définie à la fin du programme)
  enableInterrupt(codeurAvantGauchePinA, GestionInterruptionCodeurAvantGauchePinA, CHANGE);

  // Moteur à courant continu arrière droit
  pinMode(directionMoteurArriereDroit, OUTPUT);
  pinMode(directionBMoteurArriereDroit, OUTPUT);
  pinMode(pwmMoteurArriereDroit, OUTPUT);
  pinMode(mesurePWM_MoteurArriereDroit, INPUT);

  // Moteur à courant continu arrière gauche
  pinMode(directionMoteurArriereGauche, OUTPUT);
  pinMode(directionBMoteurArriereGauche, OUTPUT);
  pinMode(pwmMoteurArriereGauche, OUTPUT);
  pinMode(mesurePWM_MoteurArriereGauche, INPUT);

  // Moteur à courant continu avant droit
  pinMode(directionMoteurAvantDroit, OUTPUT);
  pinMode(directionBMoteurAvantDroit, OUTPUT);
  pinMode(pwmMoteurAvantDroit, OUTPUT);
  pinMode(mesurePWM_MoteurAvantDroit, INPUT);

  // Moteur à courant continu avant gauche
  pinMode(directionMoteurAvantGauche, OUTPUT);
  pinMode(directionBMoteurAvantGauche, OUTPUT);
  pinMode(pwmMoteurAvantGauche, OUTPUT);
  pinMode(mesurePWM_MoteurAvantGauche, INPUT);
  
  Serial.begin(115200);
//  while (!Serial) {
//    ; // Attente de la connexion du port série. Requis pour l'USB natif
//  }
  
  // Fréquence PWM de 31 kHz
  // Broche 5 (Timer 3)
  TCCR3B = (TCCR3B & 0xF8) | 0x01;
  // Broche 6 (Timer 4)
  TCCR4B = (TCCR4B & 0xF8) | 0x01;
  // Broches 44 et 46 (Timer 5)
  TCCR5B = (TCCR5B & 0xF8) | 0x01;

  // Fréquence i2c de 200 kHz pour le pcDuino
  TWBR = ((F_CPU / 200000L) - 16) / 2;
  
  /* Le programme principal est constitué:
     - de la traditionnelle boucle "loop" des programmes Arduino, dans laquelle on lit 
       en permanence la liaison série pour récupérer les nouvelles consignes de mouvement
     - de la fonction "isrt", dont l'exécution est définie à cadence fixe par les deux instructions
       ci-dessous. La bonne méthode pour exécuter une fonction à cadence fixe, c'est de l'exécuter sur interruption
       (la boucle "loop" est alors interrompue régulièrement, à la cadence voulue, par un timer).
       Une mauvaise méthode serait d'utiliser la fonction Arduino "delay". En effet, la fonction "delay" définit
       un délai entre la fin des instructions qui la précèdent et le début des instructions qui la suivent, mais si
       on ne connait pas le temps d'exécution de ces instructions (ou si celui-ci est inconnu), on n'obtiendra
       jamais une exécution à cadence fixe.
       Il est nécessaire d'exécuter la fonction "isrt" à cadence fixe car cette fonction:
       - doit renvoyer des données à cadence fixe sur la liaison série, pour monitoring
       - calcule l'asservissement de verticalité et de mouvement, conçu pour fonctionner à une cadence bien spécifiée
     
  */
  FlexiTimer2::set(CADENCE_MS, 1/1000., isrt); // résolution timer = 1 ms
  FlexiTimer2::start();

}

void setMoteurArriereDroit(int16_t value) {
  CommandeMoteur(value, directionMoteurArriereDroit, directionBMoteurArriereDroit, pwmMoteurArriereDroit);
}

void setMoteurArriereGauche(int16_t value) {
  CommandeMoteur(value, directionMoteurArriereGauche, directionBMoteurArriereGauche, pwmMoteurArriereGauche);
}

void setMoteurAvantDroit(int16_t value) {
  CommandeMoteur(value, directionMoteurAvantDroit, directionBMoteurAvantDroit, pwmMoteurAvantDroit);
}

void setMoteurAvantGauche(int16_t value) {
  CommandeMoteur(value, directionMoteurAvantGauche, directionBMoteurAvantGauche, pwmMoteurAvantGauche);
}

void setMoteursArriere(int16_t ard, int16_t arg) {
  commandeArriereDroit = ard;
  commandeArriereGauche = arg;
}

void setMoteursAvant(int16_t avd, int16_t avg) {
  commandeAvantDroit = avd;
  commandeAvantGauche = avg;
}

void setMoteurs(int16_t crc_ar, int16_t crc_av) {
  if (crc_ar == commandeArriereDroit + commandeArriereGauche) {
    CommandeMoteur(commandeArriereDroit, directionMoteurArriereDroit, directionBMoteurArriereDroit, pwmMoteurArriereDroit);
    CommandeMoteur(commandeArriereGauche, directionMoteurArriereGauche, directionBMoteurArriereGauche, pwmMoteurArriereGauche);
    commandeArriereDroitPrec = commandeArriereDroit;
    commandeArriereGauchePrec = commandeArriereGauche;
  }
  else {
    CommandeMoteur(commandeArriereDroitPrec, directionMoteurArriereDroit, directionBMoteurArriereDroit, pwmMoteurArriereDroit);
    CommandeMoteur(commandeArriereGauchePrec, directionMoteurArriereGauche, directionBMoteurArriereGauche, pwmMoteurArriereGauche);
  }
  
  if (crc_av == commandeAvantDroit + commandeAvantGauche) {
    CommandeMoteur(commandeAvantDroit, directionMoteurAvantDroit, directionBMoteurAvantDroit, pwmMoteurAvantDroit);
    CommandeMoteur(commandeAvantGauche, directionMoteurAvantGauche, directionBMoteurAvantGauche, pwmMoteurAvantGauche);
    commandeAvantDroitPrec = commandeAvantDroit;
    commandeAvantGauchePrec = commandeAvantGauche;
  }
  else {
    CommandeMoteur(commandeAvantDroitPrec, directionMoteurAvantDroit, directionBMoteurAvantDroit, pwmMoteurAvantDroit);
    CommandeMoteur(commandeAvantGauchePrec, directionMoteurAvantGauche, directionBMoteurAvantGauche, pwmMoteurAvantGauche);
  }
}

uint16_t getCodeursArriereDeltaPos() {
  return ((uint16_t)(codeurArriereDroitDeltaPos + 128) << 8) + (uint16_t)(codeurArriereGaucheDeltaPos + 128);
}

uint16_t getCodeursAvantDeltaPos() {
  return ((uint16_t)(codeurAvantDroitDeltaPos + 128) << 8) + (uint16_t)(codeurAvantGaucheDeltaPos + 128);
}

uint32_t getCodeursDeltaPos() {
  return ((uint32_t)(codeurArriereDroitDeltaPos + 128) << 24) + ((uint32_t)(codeurArriereGaucheDeltaPos + 128) << 16) + ((uint32_t)(codeurAvantDroitDeltaPos + 128) << 8) + (uint32_t)(codeurAvantGaucheDeltaPos + 128);
}

int8_t getCodeurArriereDroitDeltaPos() {
  return codeurArriereDroitDeltaPos;
}

int8_t getCodeurArriereGaucheDeltaPos() {
  return codeurArriereGaucheDeltaPos;
}

int8_t getCodeurAvantDroitDeltaPos() {
  return codeurAvantDroitDeltaPos;
}

int8_t getCodeurAvantGaucheDeltaPos() {
  return codeurAvantGaucheDeltaPos;
}

uint16_t getLineSensor(uint8_t lineSensor) {
  return (int)(5000.0 * (float)analogRead(lineSensor + 1) / 1024.);
}

uint16_t getBatteryVoltage() {
  return (int)(1000. * 3. * (5. * (float)analogRead(5) / 1024.));
}

uint8_t getSW() {
  uint8_t SW1State = digitalRead(SW1);
  uint8_t SW2State = digitalRead(SW2);
  // On retourne les valeurs sous forme de 2 bits
  return (uint8_t)(SW1State << 1) + SW2State;
}

uint16_t getAnalogValue(uint8_t analogPin) {
  return analogRead(analogPin);
}

uint16_t firmwareOK() {
  return 1;
}

uint16_t getDistance() {
  return sonar.ping_cm();
}

void checkCommands() {
  slave.checkCommand(1, setMoteurArriereDroit);
  slave.checkCommand(2, setMoteurArriereGauche);
  slave.checkCommand(3, setMoteurAvantDroit);
  slave.checkCommand(4, setMoteurAvantGauche);
  slave.checkCommand(5, setMoteursArriere);
  slave.checkCommand(6, setMoteursAvant);
  slave.checkCommand(7, getCodeursArriereDeltaPos);
  slave.checkCommand(8, getCodeursAvantDeltaPos);
  slave.checkCommand(9, getCodeursDeltaPos);
  slave.checkCommand(10, getLineSensor);
  slave.checkCommand(11, getBatteryVoltage);
  slave.checkCommand(12, getSW);
  slave.checkCommand(13, getAnalogValue);
  slave.checkCommand(14, setMoteurs);
  slave.checkCommand(15, firmwareOK);
  slave.checkCommand(16, getDistance);
}


// Boucle principale
void loop() {
  if(slave.commandReady()) {
    checkCommands();
    slave.commandDone();
  }
}

// Fonction excutée sur interruption
void isrt() {
  
  // On calcule une moyenne glissante de la vitesse sur les Nmoy derniers échantillons
  // Nombre de ticks codeur depuis la dernière fois: correspond à l'angle de rotation du moteur pendant la période d'échantillonnage
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurArriereDroitPinA et GestionInterruptionCodeurArriereDroitPinB,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  for (int i=0; i<Nmoy; i++) {
    ticksCodeurArriereDroitTab[i] += ticksCodeurArriereDroit;
  }  
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeurArriereDroit = 0;

  // Pour l'échantillon courant, calcule de l'angle de rotation du moteur pendant la période d'échantillonnage
  codeurArriereDroitDeltaPos = ticksCodeurArriereDroitTab[indiceTicksCodeurArriereDroit];
  // Remise à zéro du compteur d'impulsion codeur pour l'échantillon courant
  ticksCodeurArriereDroitTab[indiceTicksCodeurArriereDroit] = 0;
  
  // Mise à jour de l'indice d'échantillon courant
  indiceTicksCodeurArriereDroit++;
  if (indiceTicksCodeurArriereDroit==Nmoy) {
    indiceTicksCodeurArriereDroit = 0;
  }
  
  // On calcule une moyenne glissante de la vitesse sur les Nmoy derniers échantillons
  // Nombre de ticks codeur depuis la dernière fois: correspond à l'angle de rotation du moteur pendant la période d'échantillonnage
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurArriereGauchePinA et GestionInterruptionCodeurArriereGauchePinB,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  for (int i=0; i<Nmoy; i++) {
    ticksCodeurArriereGaucheTab[i] += ticksCodeurArriereGauche;
  }  
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeurArriereGauche = 0;

  // Pour l'échantillon courant, calcule de l'angle de rotation du moteur pendant la période d'échantillonnage
  codeurArriereGaucheDeltaPos = ticksCodeurArriereGaucheTab[indiceTicksCodeurArriereGauche];
  // Remise à zéro du compteur d'impulsion codeur pour l'échantillon courant
  ticksCodeurArriereGaucheTab[indiceTicksCodeurArriereGauche] = 0;
  
  // Mise à jour de l'indice d'échantillon courant
  indiceTicksCodeurArriereGauche++;
  if (indiceTicksCodeurArriereGauche==Nmoy) {
    indiceTicksCodeurArriereGauche = 0;
  }

  // On calcule une moyenne glissante de la vitesse sur les Nmoy derniers échantillons
  // Nombre de ticks codeur depuis la dernière fois: correspond à l'angle de rotation du moteur pendant la période d'échantillonnage
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurAvantDroitPinA et GestionInterruptionCodeurAvantDroitPinB,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  for (int i=0; i<Nmoy; i++) {
    ticksCodeurAvantDroitTab[i] += ticksCodeurAvantDroit;
  }  
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeurAvantDroit = 0;

  // Pour l'échantillon courant, calcule de l'angle de rotation du moteur pendant la période d'échantillonnage
  codeurAvantDroitDeltaPos = ticksCodeurAvantDroitTab[indiceTicksCodeurAvantDroit];
  // Remise à zéro du compteur d'impulsion codeur pour l'échantillon courant
  ticksCodeurAvantDroitTab[indiceTicksCodeurAvantDroit] = 0;
  
  // Mise à jour de l'indice d'échantillon courant
  indiceTicksCodeurAvantDroit++;
  if (indiceTicksCodeurAvantDroit==Nmoy) {
    indiceTicksCodeurAvantDroit = 0;
  }

  // On calcule une moyenne glissante de la vitesse sur les Nmoy derniers échantillons
  // Nombre de ticks codeur depuis la dernière fois: correspond à l'angle de rotation du moteur pendant la période d'échantillonnage
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurAvantGauchePinA et GestionInterruptionCodeurAvantGauchePinB,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  for (int i=0; i<Nmoy; i++) {
    ticksCodeurAvantGaucheTab[i] += ticksCodeurAvantGauche;
  }  
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeurAvantGauche = 0;

  // Pour l'échantillon courant, calcule de l'angle de rotation du moteur pendant la période d'échantillonnage
  codeurAvantGaucheDeltaPos = ticksCodeurAvantGaucheTab[indiceTicksCodeurAvantGauche];
  // Remise à zéro du compteur d'impulsion codeur pour l'échantillon courant
  ticksCodeurAvantGaucheTab[indiceTicksCodeurAvantGauche] = 0;
  
  // Mise à jour de l'indice d'échantillon courant
  indiceTicksCodeurAvantGauche++;
  if (indiceTicksCodeurAvantGauche==Nmoy) {
    indiceTicksCodeurAvantGauche = 0;
  }

}

void CommandeMoteur(int tension, int directionMoteur, int directionBMoteur, int pwmMoteur) {
  // Cette fonction calcule et envoi les signaux PWM au pont en H
  // en fonction des tensions de commande et d'alimentation

  // Saturation par sécurité
  if (tension>255) {
    tension = 255;
  }
  if (tension<-255) {
    tension = -255;
  }
  
  // Commande PWM
  if (tension>=0) {
    digitalWrite(directionMoteur, HIGH);
    digitalWrite(directionBMoteur, LOW);
    analogWrite(pwmMoteur, tension);
  }
  if (tension<0) {
    digitalWrite(directionMoteur, LOW);
    digitalWrite(directionBMoteur, HIGH);
    analogWrite(pwmMoteur, -tension);
  }
}

void GestionInterruptionCodeurArriereDroitPinA() {  
  // Routine de service d'interruption attachée à la voie A du codeur incrémental arrière droit
  // On utilise la fonction digitalReadFast2 de la librairie digitalWriteFast
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (digitalReadFast2(codeurArriereDroitPinA) == digitalReadFast2(codeurArriereDroitPinB)) {
    ticksCodeurArriereDroit++;
  }
  else {
    ticksCodeurArriereDroit--;
  }
}


void GestionInterruptionCodeurArriereGauchePinA() {  
  // Routine de service d'interruption attachée à la voie A du codeur incrémental arrière gauche
  // On utilise la fonction digitalReadFast2 de la librairie digitalWriteFast
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (digitalReadFast2(codeurArriereGauchePinA) == digitalReadFast2(codeurArriereGauchePinB)) {
    ticksCodeurArriereGauche++;
  }
  else {
    ticksCodeurArriereGauche--;
  }
}


void GestionInterruptionCodeurAvantDroitPinA() {  
  // Routine de service d'interruption attachée à la voie A du codeur incrémental arrière droit
  // On utilise la fonction digitalReadFast2 de la librairie digitalWriteFast
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (digitalReadFast2(codeurAvantDroitPinA) == digitalReadFast2(codeurAvantDroitPinB)) {
    ticksCodeurAvantDroit++;
  }
  else {
    ticksCodeurAvantDroit--;
  }
}


void GestionInterruptionCodeurAvantGauchePinA() {  
  // Routine de service d'interruption attachée à la voie A du codeur incrémental arrière gauche
  // On utilise la fonction digitalReadFast2 de la librairie digitalWriteFast
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (digitalReadFast2(codeurAvantGauchePinA) == digitalReadFast2(codeurAvantGauchePinB)) {
    ticksCodeurAvantGauche++;
  }
  else {
    ticksCodeurAvantGauche--;
  }
}

