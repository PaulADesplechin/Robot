#include <AFMotor.h>

const int trigPin = 9;
const int echoPin = 10;

AF_DCMotor moteurGauche(3);
AF_DCMotor moteurDroit(4);

const int VITESSE_MAX = 255;
const int VITESSE_MIN = 200;
const int SEUIL_DETECTION_CM = 90;
const int SEUIL_ALERTE_CM = 50;
const int SEUIL_CRITIQUE_CM = 25;

enum EtatRobot {
  AVANCE_DROIT,
  EVITER_OBSTACLE
};

EtatRobot etat = AVANCE_DROIT;

unsigned long tempsDebut = 0;
bool tournerDroite = true;
long distancePrecedente = 400;
long historiqueDistance[10];
int indexHistorique = 0;
int nbMesuresValides = 0;
int compteurObstacle = 0;

long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(12);
  digitalWrite(trigPin, LOW);

  long duree = pulseIn(echoPin, HIGH, 35000);
  
  if (duree == 0) {
    return 400;
  }
  
  long distance = (duree * 0.034 / 2);
  
  if (distance < 2 || distance > 400) {
    return 400;
  }
  
  return distance;
}

long getDistanceFiltree() {
  long mesures[5];
  
  for (int i = 0; i < 5; i++) {
    mesures[i] = getDistance();
    if (i < 4) delayMicroseconds(3000);
  }
  
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4 - i; j++) {
      if (mesures[j] > mesures[j + 1]) {
        long temp = mesures[j];
        mesures[j] = mesures[j + 1];
        mesures[j + 1] = temp;
      }
    }
  }
  
  long mediane = mesures[2];
  long moyenne = 0;
  int count = 0;
  
  for (int i = 0; i < 5; i++) {
    if (abs(mesures[i] - mediane) < 20) {
      moyenne += mesures[i];
      count++;
    }
  }
  
  if (count > 0) {
    return moyenne / count;
  }
  
  return mediane;
}

void avancerDroit(int vitesseGauche, int vitesseDroite) {
  moteurGauche.setSpeed(vitesseGauche);
  moteurDroit.setSpeed(vitesseDroite);
  moteurGauche.run(FORWARD);
  moteurDroit.run(FORWARD);
}

void reculer() {
  moteurGauche.setSpeed(220);
  moteurDroit.setSpeed(220);
  moteurGauche.run(BACKWARD);
  moteurDroit.run(BACKWARD);
}

void tournerAngle(bool droite, int angleDegres) {
  int vitesse = 220;
  moteurGauche.setSpeed(vitesse);
  moteurDroit.setSpeed(vitesse);
  
  if (droite) {
    moteurGauche.run(FORWARD);
    moteurDroit.run(BACKWARD);
  } else {
    moteurGauche.run(BACKWARD);
    moteurDroit.run(FORWARD);
  }
  
  unsigned long duree = (angleDegres * 4);
  delay(duree);
  
  moteurGauche.run(RELEASE);
  moteurDroit.run(RELEASE);
  delay(30);
}

void arret() {
  moteurGauche.run(RELEASE);
  moteurDroit.run(RELEASE);
}

int calculerVitesseAdaptative(long distance) {
  if (distance < 0 || distance > SEUIL_DETECTION_CM) {
    return VITESSE_MAX;
  } else if (distance < SEUIL_CRITIQUE_CM) {
    return VITESSE_MIN;
  } else if (distance < SEUIL_ALERTE_CM) {
    float ratio = (float)(distance - SEUIL_CRITIQUE_CM) / (SEUIL_ALERTE_CM - SEUIL_CRITIQUE_CM);
    return VITESSE_MIN + (int)(ratio * (VITESSE_MAX - VITESSE_MIN) * 0.3);
  } else {
    float ratio = (float)(distance - SEUIL_ALERTE_CM) / (SEUIL_DETECTION_CM - SEUIL_ALERTE_CM);
    return VITESSE_MIN + (int)(ratio * (VITESSE_MAX - VITESSE_MIN) * 0.5) + (VITESSE_MAX - VITESSE_MIN) * 0.5;
  }
}

int calculerAngleRotation(long distance) {
  if (distance >= SEUIL_DETECTION_CM || distance < 0) {
    return 0;
  } else if (distance < SEUIL_CRITIQUE_CM) {
    return 90 + (compteurObstacle * 10);
  } else if (distance < SEUIL_ALERTE_CM) {
    float ratio = (float)(SEUIL_ALERTE_CM - distance) / (SEUIL_ALERTE_CM - SEUIL_CRITIQUE_CM);
    return 60 + (int)(ratio * 30);
  } else {
    float ratio = (float)(SEUIL_DETECTION_CM - distance) / (SEUIL_DETECTION_CM - SEUIL_ALERTE_CM);
    return (int)(ratio * 60);
  }
}

float calculerTendance(long distance) {
  if (nbMesuresValides < 4) {
    return 0.0;
  }
  
  float somme = 0;
  int count = 0;
  for (int i = 1; i < nbMesuresValides && i < 8; i++) {
    int idx1 = (indexHistorique - i - 1 + 10) % 10;
    int idx2 = (indexHistorique - i + 10) % 10;
    if (historiqueDistance[idx1] > 0 && historiqueDistance[idx2] > 0 && historiqueDistance[idx1] < 400 && historiqueDistance[idx2] < 400) {
      somme += (historiqueDistance[idx2] - historiqueDistance[idx1]);
      count++;
    }
  }
  
  return count > 0 ? (somme / count) : 0.0;
}

long detecterChuteBrutale(long distanceActuelle, long distanceMoyenne) {
  if (distancePrecedente > 0 && distancePrecedente < 400 && distanceActuelle < 400) {
    long chute = distancePrecedente - distanceActuelle;
    if (chute > 15 && distanceActuelle < SEUIL_DETECTION_CM) {
      return distanceActuelle;
    }
  }
  return distanceMoyenne;
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  moteurGauche.setSpeed(VITESSE_MAX);
  moteurDroit.setSpeed(VITESSE_MAX);
  arret();
  delay(1000);
  
  for (int i = 0; i < 10; i++) {
    historiqueDistance[i] = 400;
  }
  
  Serial.println("Robot haute performance pret");
}

void loop() {
  long distance = getDistanceFiltree();
  
  if (distance < 2 || distance > 400) {
    if (distancePrecedente > 0 && distancePrecedente < 400) {
      distance = distancePrecedente;
    } else {
      distance = 400;
    }
  }
  
  historiqueDistance[indexHistorique] = distance;
  indexHistorique = (indexHistorique + 1) % 10;
  if (nbMesuresValides < 10) nbMesuresValides++;

  long distanceMoyenne = 0;
  int count = 0;
  for (int i = 0; i < nbMesuresValides && i < 7; i++) {
    int idx = (indexHistorique - i - 1 + 10) % 10;
    if (historiqueDistance[idx] > 0 && historiqueDistance[idx] < 400) {
      distanceMoyenne += historiqueDistance[idx];
      count++;
    }
  }
  if (count > 0) {
    distanceMoyenne /= count;
  } else {
    distanceMoyenne = distance;
  }

  float tendance = calculerTendance(distance);
  long distancePredite = distanceMoyenne + (long)(tendance * 2.0);
  
  if (distancePredite < 0) distancePredite = distanceMoyenne;
  if (distancePredite > 400) distancePredite = 400;

  long distanceUtilisee = detecterChuteBrutale(distance, distanceMoyenne);
  if (distanceUtilisee == distanceMoyenne) {
    distanceUtilisee = distancePredite > 0 ? distancePredite : distanceMoyenne;
  }

  long variation = distance - distancePrecedente;

  Serial.print("Dist:");
  Serial.print(distance);
  Serial.print(" moy:");
  Serial.print(distanceMoyenne);
  Serial.print(" pred:");
  Serial.print(distancePredite);
  Serial.print(" use:");
  Serial.print(distanceUtilisee);
  if (variation != 0) {
    Serial.print(" var:");
    if (variation > 0) Serial.print("+");
    Serial.print(variation);
  }

  distancePrecedente = distance;

  if (etat == AVANCE_DROIT) {
    if (distanceUtilisee < SEUIL_CRITIQUE_CM && distanceUtilisee > 1) {
      Serial.println(" -> OBSTACLE CRITIQUE!");
      compteurObstacle++;
      etat = EVITER_OBSTACLE;
      tempsDebut = millis();
      arret();
      delay(30);
    } else if (distanceUtilisee < SEUIL_ALERTE_CM && distanceUtilisee > SEUIL_CRITIQUE_CM) {
      Serial.print(" -> CORRECTION FORTE (angle:");
      int angle = calculerAngleRotation(distanceUtilisee);
      Serial.print(angle);
      Serial.println("deg)");
      
      int vitesse = calculerVitesseAdaptative(distanceUtilisee);
      int correction = (angle * vitesse) / 120;
      if (correction < 8) correction = 8;
      if (correction > vitesse - VITESSE_MIN) correction = vitesse - VITESSE_MIN;
      
      if (tournerDroite) {
        avancerDroit(vitesse, max(VITESSE_MIN, vitesse - correction));
      } else {
        avancerDroit(max(VITESSE_MIN, vitesse - correction), vitesse);
      }
    } else if (distanceUtilisee < SEUIL_DETECTION_CM && distanceUtilisee > SEUIL_ALERTE_CM) {
      Serial.print(" -> PREPARATION (angle:");
      int angle = calculerAngleRotation(distanceUtilisee);
      Serial.print(angle);
      Serial.println("deg)");
      
      int vitesse = calculerVitesseAdaptative(distanceUtilisee);
      int correction = (angle * vitesse) / 250;
      if (correction < 5) correction = 5;
      
      if (tournerDroite) {
        avancerDroit(vitesse, vitesse - correction);
      } else {
        avancerDroit(vitesse - correction, vitesse);
      }
    } else {
      Serial.println(" -> AVANCE DROIT");
      compteurObstacle = 0;
      int vitesse = calculerVitesseAdaptative(distanceUtilisee);
      avancerDroit(vitesse, vitesse);
    }
  } else if (etat == EVITER_OBSTACLE) {
    unsigned long tempsEcoule = millis() - tempsDebut;

    if (tempsEcoule < 150) {
      reculer();
      Serial.println(" -> RECUL");
    } else if (tempsEcoule < 200) {
      arret();
      Serial.println(" -> ARRET");
    } else {
      int angle = calculerAngleRotation(distanceUtilisee);
      if (angle < 50) angle = 60;
      if (angle > 120) angle = 120;
      
      Serial.print(" -> TOURNE ");
      Serial.print(tournerDroite ? "DROITE" : "GAUCHE");
      Serial.print(" (");
      Serial.print(angle);
      Serial.println("deg)");
      
      tournerAngle(tournerDroite, angle);
      tournerDroite = !tournerDroite;
      etat = AVANCE_DROIT;
    }
  }
  
  delay(8);
}
