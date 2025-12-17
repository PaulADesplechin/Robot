#include <AFMotor.h>

const int trigPin = 9;
const int echoPin = 10;

AF_DCMotor moteurGauche(3);
AF_DCMotor moteurDroit(4);

const int VITESSE_MAX = 255;
const int VITESSE_MIN = 180;
const int SEUIL_DETECTION_CM = 70;
const int SEUIL_ALERTE_CM = 40;
const int SEUIL_CRITIQUE_CM = 22;

enum EtatRobot {
  AVANCE_DROIT,
  CORRECTION_PROGRESSIVE,
  EVITER_OBSTACLE
};

EtatRobot etat = AVANCE_DROIT;

unsigned long tempsDebut = 0;
bool tournerDroite = true;
long distancePrecedente = 0;
long historiqueDistance[7];
int indexHistorique = 0;
int nbMesuresValides = 0;

long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duree = pulseIn(echoPin, HIGH, 30000);
  
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
  long d1 = getDistance();
  delayMicroseconds(5000);
  long d2 = getDistance();
  delayMicroseconds(5000);
  long d3 = getDistance();
  
  long mesures[3] = {d1, d2, d3};
  
  if (mesures[0] > mesures[1]) {
    long temp = mesures[0];
    mesures[0] = mesures[1];
    mesures[1] = temp;
  }
  if (mesures[1] > mesures[2]) {
    long temp = mesures[1];
    mesures[1] = mesures[2];
    mesures[2] = temp;
  }
  if (mesures[0] > mesures[1]) {
    long temp = mesures[0];
    mesures[0] = mesures[1];
    mesures[1] = temp;
  }
  
  return mesures[1];
}

void avancerDroit(int vitesseGauche, int vitesseDroite) {
  moteurGauche.setSpeed(vitesseGauche);
  moteurDroit.setSpeed(vitesseDroite);
  moteurGauche.run(FORWARD);
  moteurDroit.run(FORWARD);
}

void reculer() {
  moteurGauche.setSpeed(200);
  moteurDroit.setSpeed(200);
  moteurGauche.run(BACKWARD);
  moteurDroit.run(BACKWARD);
}

void tournerAngle(bool droite, int angleDegres) {
  int vitesse = 200;
  moteurGauche.setSpeed(vitesse);
  moteurDroit.setSpeed(vitesse);
  
  if (droite) {
    moteurGauche.run(FORWARD);
    moteurDroit.run(BACKWARD);
  } else {
    moteurGauche.run(BACKWARD);
    moteurDroit.run(FORWARD);
  }
  
  unsigned long duree = (angleDegres * 5);
  delay(duree);
  
  moteurGauche.run(RELEASE);
  moteurDroit.run(RELEASE);
  delay(50);
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
    return VITESSE_MIN + (int)(ratio * (VITESSE_MAX - VITESSE_MIN) * 0.4);
  } else {
    float ratio = (float)(distance - SEUIL_ALERTE_CM) / (SEUIL_DETECTION_CM - SEUIL_ALERTE_CM);
    return VITESSE_MIN + (int)(ratio * (VITESSE_MAX - VITESSE_MIN) * 0.6) + (VITESSE_MAX - VITESSE_MIN) * 0.4;
  }
}

int calculerAngleRotation(long distance) {
  if (distance >= SEUIL_DETECTION_CM || distance < 0) {
    return 0;
  } else if (distance < SEUIL_CRITIQUE_CM) {
    return 90;
  } else if (distance < SEUIL_ALERTE_CM) {
    float ratio = (float)(SEUIL_ALERTE_CM - distance) / (SEUIL_ALERTE_CM - SEUIL_CRITIQUE_CM);
    return 50 + (int)(ratio * 40);
  } else {
    float ratio = (float)(SEUIL_DETECTION_CM - distance) / (SEUIL_DETECTION_CM - SEUIL_ALERTE_CM);
    return (int)(ratio * 50);
  }
}

float calculerTendance(long distance) {
  if (nbMesuresValides < 3) {
    return 0.0;
  }
  
  float somme = 0;
  int count = 0;
  for (int i = 1; i < nbMesuresValides && i < 7; i++) {
    int idx1 = (indexHistorique - i - 1 + 7) % 7;
    int idx2 = (indexHistorique - i + 7) % 7;
    if (historiqueDistance[idx1] > 0 && historiqueDistance[idx2] > 0) {
      somme += (historiqueDistance[idx2] - historiqueDistance[idx1]);
      count++;
    }
  }
  
  return count > 0 ? (somme / count) : 0.0;
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  moteurGauche.setSpeed(VITESSE_MAX);
  moteurDroit.setSpeed(VITESSE_MAX);
  arret();
  delay(1000);
  
  for (int i = 0; i < 7; i++) {
    historiqueDistance[i] = -1;
  }
  
  Serial.println("Robot ultra precis pret");
}

void loop() {
  long distance = getDistanceFiltree();
  
  if (distance < 2 || distance > 400) {
    if (distancePrecedente > 0) {
      distance = distancePrecedente;
    } else {
      distance = 400;
    }
  }
  
  historiqueDistance[indexHistorique] = distance;
  indexHistorique = (indexHistorique + 1) % 7;
  if (nbMesuresValides < 7) nbMesuresValides++;

  long distanceMoyenne = 0;
  int count = 0;
  for (int i = 0; i < nbMesuresValides && i < 5; i++) {
    int idx = (indexHistorique - i - 1 + 7) % 7;
    if (historiqueDistance[idx] > 0) {
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
  long distancePredite = distanceMoyenne + (long)(tendance * 1.5);

  long variation = distance - distancePrecedente;

  Serial.print("Dist:");
  Serial.print(distance);
  Serial.print(" moy:");
  Serial.print(distanceMoyenne);
  Serial.print(" pred:");
  Serial.print(distancePredite);
  if (variation != 0) {
    Serial.print(" var:");
    if (variation > 0) Serial.print("+");
    Serial.print(variation);
  }

  distancePrecedente = distance;

  long distanceUtilisee = distancePredite > 0 ? distancePredite : distanceMoyenne;

  if (etat == AVANCE_DROIT) {
    if (distanceUtilisee < SEUIL_CRITIQUE_CM && distanceUtilisee > 1) {
      Serial.println(" -> OBSTACLE CRITIQUE!");
      etat = EVITER_OBSTACLE;
      tempsDebut = millis();
      arret();
      delay(50);
    } else if (distanceUtilisee < SEUIL_ALERTE_CM && distanceUtilisee > SEUIL_CRITIQUE_CM) {
      Serial.print(" -> CORRECTION PROGRESSIVE (angle:");
      int angle = calculerAngleRotation(distanceUtilisee);
      Serial.print(angle);
      Serial.println("deg)");
      
      int vitesse = calculerVitesseAdaptative(distanceUtilisee);
      int correction = (angle * vitesse) / 150;
      if (correction < 5) correction = 5;
      
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
      int correction = (angle * vitesse) / 300;
      if (correction < 3) correction = 3;
      
      if (tournerDroite) {
        avancerDroit(vitesse, vitesse - correction);
      } else {
        avancerDroit(vitesse - correction, vitesse);
      }
    } else {
      Serial.println(" -> AVANCE DROIT");
      int vitesse = calculerVitesseAdaptative(distanceUtilisee);
      avancerDroit(vitesse, vitesse);
    }
  } else if (etat == EVITER_OBSTACLE) {
    unsigned long tempsEcoule = millis() - tempsDebut;

    if (tempsEcoule < 180) {
      reculer();
      Serial.println(" -> RECUL");
    } else if (tempsEcoule < 220) {
      arret();
      Serial.println(" -> ARRET");
    } else {
      int angle = calculerAngleRotation(distanceUtilisee);
      if (angle < 40) angle = 50;
      if (angle > 90) angle = 90;
      
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
  
  delay(10);
}
