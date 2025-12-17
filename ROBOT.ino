#include <AFMotor.h>

const int trigPin = 12;
const int echoPin = 13;

AF_DCMotor moteurGauche(3);
AF_DCMotor moteurDroit(4);

const int VITESSE_MAX = 180;
const int VITESSE_MIN = 165;
const float FACTEUR_COMPENSATION_GAUCHE = 1.0;
const float FACTEUR_COMPENSATION_DROIT = 0.98;
const int SEUIL_DETECTION_CM = 110;
const int SEUIL_ALERTE_CM = 60;
const int SEUIL_CRITIQUE_CM = 30;

enum EtatRobot {
  AVANCE_DROIT,
  CORRECTION_LEGERE,
  CORRECTION_FORTE,
  EVITER_OBSTACLE
};

EtatRobot etat = AVANCE_DROIT;

unsigned long tempsDebut = 0;
bool tournerDroite = true;
long distancePrecedente = 400;
long historiqueDistance[8];
int indexHistorique = 0;
int nbMesuresValides = 0;
int compteurObstacle = 0;
int compteurBlocage = 0;
unsigned long dernierObstacleTemps = 0;
int vitesseActuelleGauche = 0;
int vitesseActuelleDroite = 0;
int vitesseCibleGauche = 0;
int vitesseCibleDroite = 0;

long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(12);
  digitalWrite(trigPin, LOW);

  long duree = pulseIn(echoPin, HIGH, 25000);
  
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
  long mesures[3];
  
  mesures[0] = getDistance();
  delayMicroseconds(1000);
  mesures[1] = getDistance();
  delayMicroseconds(1000);
  mesures[2] = getDistance();
  
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
  vitesseCibleGauche = vitesseGauche;
  vitesseCibleDroite = vitesseDroite;
}

void appliquerVitesseRamp() {
  int diffG = vitesseCibleGauche - vitesseActuelleGauche;
  int diffD = vitesseCibleDroite - vitesseActuelleDroite;
  
  if (abs(diffG) > 3) {
    int step = max(3, abs(diffG) / 3);
    if (diffG > 0) vitesseActuelleGauche = min(vitesseCibleGauche, vitesseActuelleGauche + step);
    else vitesseActuelleGauche = max(vitesseCibleGauche, vitesseActuelleGauche - step);
  } else {
    vitesseActuelleGauche = vitesseCibleGauche;
  }
  
  if (abs(diffD) > 3) {
    int step = max(3, abs(diffD) / 3);
    if (diffD > 0) vitesseActuelleDroite = min(vitesseCibleDroite, vitesseActuelleDroite + step);
    else vitesseActuelleDroite = max(vitesseCibleDroite, vitesseActuelleDroite - step);
  } else {
    vitesseActuelleDroite = vitesseCibleDroite;
  }
  
  if (vitesseCibleGauche == vitesseCibleDroite && etat == AVANCE_DROIT) {
    int vitesseMoyenne = (vitesseActuelleGauche + vitesseActuelleDroite) / 2;
    vitesseActuelleGauche = vitesseMoyenne;
    vitesseActuelleDroite = vitesseMoyenne;
  }
  
  int vitesseGaucheFinale = (int)(vitesseActuelleGauche * FACTEUR_COMPENSATION_GAUCHE);
  int vitesseDroiteFinale = (int)(vitesseActuelleDroite * FACTEUR_COMPENSATION_DROIT);
  
  if (vitesseCibleGauche == vitesseCibleDroite && etat == AVANCE_DROIT) {
    int vitesseMoyenneFinale = (vitesseGaucheFinale + vitesseDroiteFinale) / 2;
    vitesseGaucheFinale = vitesseMoyenneFinale;
    vitesseDroiteFinale = vitesseMoyenneFinale;
  }
  
  moteurGauche.setSpeed(vitesseGaucheFinale);
  moteurDroit.setSpeed(vitesseDroiteFinale);
  moteurGauche.run(FORWARD);
  moteurDroit.run(FORWARD);
}

void reculer() {
  vitesseActuelleGauche = 0;
  vitesseActuelleDroite = 0;
  int vitesseRecul = 175;
  moteurGauche.setSpeed((int)(vitesseRecul * FACTEUR_COMPENSATION_GAUCHE));
  moteurDroit.setSpeed((int)(vitesseRecul * FACTEUR_COMPENSATION_DROIT));
  moteurGauche.run(BACKWARD);
  moteurDroit.run(BACKWARD);
}

void tournerAngle(bool droite, int angleDegres) {
  vitesseActuelleGauche = 0;
  vitesseActuelleDroite = 0;
  int vitesse = 175;
  moteurGauche.setSpeed((int)(vitesse * FACTEUR_COMPENSATION_GAUCHE));
  moteurDroit.setSpeed((int)(vitesse * FACTEUR_COMPENSATION_DROIT));
  
  if (droite) {
    moteurGauche.run(FORWARD);
    moteurDroit.run(BACKWARD);
  } else {
    moteurGauche.run(BACKWARD);
    moteurDroit.run(FORWARD);
  }
  
  unsigned long duree = (angleDegres * 1);
  delay(duree);
  
  moteurGauche.run(RELEASE);
  moteurDroit.run(RELEASE);
  delay(10);
}

void arret() {
  vitesseActuelleGauche = 0;
  vitesseActuelleDroite = 0;
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
    return VITESSE_MIN + (int)(ratio * (VITESSE_MAX - VITESSE_MIN) * 0.25);
  } else {
    float ratio = (float)(distance - SEUIL_ALERTE_CM) / (SEUIL_DETECTION_CM - SEUIL_ALERTE_CM);
    return VITESSE_MIN + (int)(ratio * (VITESSE_MAX - VITESSE_MIN) * 0.4) + (VITESSE_MAX - VITESSE_MIN) * 0.6;
  }
}

int calculerAngleRotation(long distance) {
  if (distance >= SEUIL_DETECTION_CM || distance < 0) {
    return 0;
  }
  
  int angleBase = 0;
  
  if (distance < SEUIL_CRITIQUE_CM) {
    angleBase = 85 + (compteurObstacle * 8) + (compteurBlocage * 15);
    if (angleBase > 180) angleBase = 180;
  } else if (distance < SEUIL_ALERTE_CM) {
    float ratio = (float)(SEUIL_ALERTE_CM - distance) / (SEUIL_ALERTE_CM - SEUIL_CRITIQUE_CM);
    angleBase = 60 + (int)(ratio * 25) + (compteurBlocage * 10);
  } else {
    float ratio = (float)(SEUIL_DETECTION_CM - distance) / (SEUIL_DETECTION_CM - SEUIL_ALERTE_CM);
    angleBase = (int)(ratio * 60);
  }
  
  return angleBase;
}

long calculerDistanceUtilisee(long distance, long distanceMoyenne) {
  if (distancePrecedente > 0 && distancePrecedente < 400 && distance < 400) {
    long chute = distancePrecedente - distance;
    if (chute > 8 && distance < SEUIL_DETECTION_CM) {
      return distance;
    }
  }
  
  return (distanceMoyenne * 2 + distance) / 3;
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  moteurGauche.setSpeed(VITESSE_MAX);
  moteurDroit.setSpeed(VITESSE_MAX);
  arret();
  delay(300);
  
  for (int i = 0; i < 8; i++) {
    historiqueDistance[i] = 400;
  }
  
  Serial.println("Robot optimise pret");
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
  indexHistorique = (indexHistorique + 1) % 8;
  if (nbMesuresValides < 8) nbMesuresValides++;

  long distanceMoyenne = 0;
  int count = 0;
  for (int i = 0; i < nbMesuresValides && i < 5; i++) {
    int idx = (indexHistorique - i - 1 + 8) % 8;
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

  long distanceUtilisee = calculerDistanceUtilisee(distance, distanceMoyenne);
  unsigned long maintenant = millis();

  if (distanceUtilisee < SEUIL_CRITIQUE_CM && distanceUtilisee > 1) {
    if (maintenant - dernierObstacleTemps < 2000) {
      compteurBlocage++;
      if (compteurBlocage > 5) compteurBlocage = 5;
    } else {
      compteurBlocage = max(0, compteurBlocage - 1);
    }
    dernierObstacleTemps = maintenant;
  } else if (distanceUtilisee > SEUIL_DETECTION_CM) {
    compteurBlocage = 0;
  }

  Serial.print("Dist:");
  Serial.print(distance);
  Serial.print(" use:");
  Serial.print(distanceUtilisee);
  Serial.print(" v:");
  Serial.print((vitesseActuelleGauche + vitesseActuelleDroite) / 2);
  if (compteurBlocage > 0) {
    Serial.print(" bloc:");
    Serial.print(compteurBlocage);
  }

  distancePrecedente = distance;

  if (etat == AVANCE_DROIT) {
    if (distanceUtilisee < SEUIL_CRITIQUE_CM && distanceUtilisee > 1) {
      Serial.println(" -> CRITIQUE!");
      compteurObstacle++;
      etat = EVITER_OBSTACLE;
      tempsDebut = maintenant;
      arret();
      delay(8);
    } else if (distanceUtilisee < SEUIL_ALERTE_CM && distanceUtilisee > SEUIL_CRITIQUE_CM) {
      if (etat != CORRECTION_FORTE) {
        Serial.println(" -> CORRECTION FORTE");
        etat = CORRECTION_FORTE;
      }
      
      int angle = calculerAngleRotation(distanceUtilisee);
      int vitesse = calculerVitesseAdaptative(distanceUtilisee);
      int correction = (angle * vitesse) / 90;
      if (correction < 15) correction = 15;
      if (correction > vitesse - VITESSE_MIN) correction = vitesse - VITESSE_MIN;
      
      if (tournerDroite) {
        avancerDroit(vitesse, max(VITESSE_MIN, vitesse - correction));
      } else {
        avancerDroit(max(VITESSE_MIN, vitesse - correction), vitesse);
      }
      appliquerVitesseRamp();
    } else if (distanceUtilisee < SEUIL_DETECTION_CM && distanceUtilisee > SEUIL_ALERTE_CM) {
      if (etat != CORRECTION_LEGERE) {
        Serial.println(" -> CORRECTION LEGERE");
        etat = CORRECTION_LEGERE;
      }
      
      int angle = calculerAngleRotation(distanceUtilisee);
      int vitesse = calculerVitesseAdaptative(distanceUtilisee);
      int correction = (angle * vitesse) / 150;
      if (correction < 10) correction = 10;
      
      if (tournerDroite) {
        avancerDroit(vitesse, vitesse - correction);
      } else {
        avancerDroit(vitesse - correction, vitesse);
      }
      appliquerVitesseRamp();
    } else {
      if (etat != AVANCE_DROIT) {
        Serial.println(" -> AVANCE DROIT");
        etat = AVANCE_DROIT;
        compteurObstacle = 0;
        compteurBlocage = max(0, compteurBlocage - 1);
      }
      int vitesse = calculerVitesseAdaptative(distanceUtilisee);
      vitesseCibleGauche = vitesse;
      vitesseCibleDroite = vitesse;
      vitesseActuelleGauche = vitesse;
      vitesseActuelleDroite = vitesse;
      appliquerVitesseRamp();
    }
  } else if (etat == CORRECTION_LEGERE || etat == CORRECTION_FORTE) {
    if (distanceUtilisee >= SEUIL_DETECTION_CM) {
      etat = AVANCE_DROIT;
      compteurObstacle = 0;
    } else if (distanceUtilisee < SEUIL_CRITIQUE_CM && distanceUtilisee > 1) {
      etat = EVITER_OBSTACLE;
      tempsDebut = maintenant;
      arret();
      delay(8);
    } else {
      int angle = calculerAngleRotation(distanceUtilisee);
      int vitesse = calculerVitesseAdaptative(distanceUtilisee);
      int correction = (etat == CORRECTION_FORTE) ? ((angle * vitesse) / 90) : ((angle * vitesse) / 150);
      if (correction < (etat == CORRECTION_FORTE ? 15 : 10)) correction = (etat == CORRECTION_FORTE ? 15 : 10);
      
      if (tournerDroite) {
        avancerDroit(vitesse, vitesse - correction);
      } else {
        avancerDroit(vitesse - correction, vitesse);
      }
      appliquerVitesseRamp();
    }
  } else if (etat == EVITER_OBSTACLE) {
    unsigned long tempsEcoule = maintenant - tempsDebut;
    int dureeRecul = compteurBlocage > 2 ? 120 : 80;
    int dureeArret = compteurBlocage > 2 ? 140 : 110;

    if (tempsEcoule < dureeRecul) {
      reculer();
      Serial.println(" -> RECUL");
    } else if (tempsEcoule < dureeArret) {
      arret();
      Serial.println(" -> ARRET");
    } else {
      int angle = calculerAngleRotation(distanceUtilisee);
      
      if (compteurBlocage > 3) {
        angle = 180;
        Serial.print(" -> ROTATION COMPLETE ");
      } else if (compteurBlocage > 2) {
        if (angle < 90) angle = 120;
        if (angle > 180) angle = 180;
      } else {
        if (angle < 55) angle = 65;
        if (angle > 120) angle = 120;
      }
      
      Serial.print(" -> TOURNE ");
      Serial.print(tournerDroite ? "DROITE" : "GAUCHE");
      Serial.print(" (");
      Serial.print(angle);
      Serial.println("deg)");
      
      tournerAngle(tournerDroite, angle);
      
      if (compteurBlocage > 2) {
        tournerDroite = !tournerDroite;
      } else {
        tournerDroite = !tournerDroite;
      }
      
      etat = AVANCE_DROIT;
    }
  }
  
  delay(2);
}
