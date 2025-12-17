#include <AFMotor.h>
 
const int trigPin = 9;
const int echoPin = 10;
 
AF_DCMotor moteurGauche(3);
AF_DCMotor moteurDroit(4);
 
const int VITESSE_MAX = 255;
const int VITESSE_MIN = 210;
const int SEUIL_DETECTION_CM = 100;
const int SEUIL_ALERTE_CM = 55;
const int SEUIL_CRITIQUE_CM = 28;

enum EtatRobot {
  AVANCE_DROIT,
  CORRECTION_LEGERE,
  CORRECTION_FORTE,
  EVITER_OBSTACLE
};

EtatRobot etat = AVANCE_DROIT;

unsigned long tempsDebut = 0;
unsigned long dernierChangementEtat = 0;
bool tournerDroite = true;
long distancePrecedente = 400;
long historiqueDistance[12];
int indexHistorique = 0;
int nbMesuresValides = 0;
int compteurObstacle = 0;
int vitesseActuelleGauche = 0;
int vitesseActuelleDroite = 0;
int vitesseCibleGauche = 0;
int vitesseCibleDroite = 0;
float confianceMesure = 1.0;
long dernierObstacleDistance = 400;
unsigned long dernierObstacleTemps = 0;
int patternObstacle = 0;

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
  long mesures[6];
  
  for (int i = 0; i < 6; i++) {
    mesures[i] = getDistance();
    if (i < 5) delayMicroseconds(2500);
  }
  
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5 - i; j++) {
      if (mesures[j] > mesures[j + 1]) {
        long temp = mesures[j];
        mesures[j] = mesures[j + 1];
        mesures[j + 1] = temp;
      }
    }
  }
  
  long mediane = (mesures[2] + mesures[3]) / 2;
  long moyenne = 0;
  int count = 0;
  float ecartTotal = 0;
  
  for (int i = 0; i < 6; i++) {
    long ecart = abs(mesures[i] - mediane);
    ecartTotal += ecart;
    if (ecart < 25) {
      moyenne += mesures[i];
      count++;
    }
  }
  
  confianceMesure = 1.0 - (ecartTotal / (6.0 * 100.0));
  if (confianceMesure < 0.3) confianceMesure = 0.3;
  if (confianceMesure > 1.0) confianceMesure = 1.0;
  
  if (count >= 3) {
    return moyenne / count;
  }
  
  return mediane;
}

void avancerDroit(int vitesseGauche, int vitesseDroite) {
  vitesseCibleGauche = vitesseGauche;
  vitesseCibleDroite = vitesseDroite;
}

void appliquerVitesseRamp() {
  int diffG = vitesseCibleGauche - vitesseActuelleGauche;
  int diffD = vitesseCibleDroite - vitesseActuelleDroite;
  int maxDiff = max(abs(diffG), abs(diffD));
  
  if (maxDiff > 0) {
    int step = max(1, maxDiff / 8);
    if (diffG > 0) vitesseActuelleGauche = min(vitesseCibleGauche, vitesseActuelleGauche + step);
    else if (diffG < 0) vitesseActuelleGauche = max(vitesseCibleGauche, vitesseActuelleGauche - step);
    
    if (diffD > 0) vitesseActuelleDroite = min(vitesseCibleDroite, vitesseActuelleDroite + step);
    else if (diffD < 0) vitesseActuelleDroite = max(vitesseCibleDroite, vitesseActuelleDroite - step);
  }
  
  moteurGauche.setSpeed(vitesseActuelleGauche);
  moteurDroit.setSpeed(vitesseActuelleDroite);
  moteurGauche.run(FORWARD);
  moteurDroit.run(FORWARD);
}
 
void reculer() {
  vitesseActuelleGauche = 0;
  vitesseActuelleDroite = 0;
  moteurGauche.setSpeed(225);
  moteurDroit.setSpeed(225);
  moteurGauche.run(BACKWARD);
  moteurDroit.run(BACKWARD);
}
 
void tournerAngle(bool droite, int angleDegres) {
  vitesseActuelleGauche = 0;
  vitesseActuelleDroite = 0;
  int vitesse = 225;
  moteurGauche.setSpeed(vitesse);
  moteurDroit.setSpeed(vitesse);
  
  if (droite) {
    moteurGauche.run(FORWARD);
    moteurDroit.run(BACKWARD);
  } else {
    moteurGauche.run(BACKWARD);
    moteurDroit.run(FORWARD);
  }
  
  unsigned long duree = (angleDegres * 3);
  delay(duree);
  
  moteurGauche.run(RELEASE);
  moteurDroit.run(RELEASE);
  delay(25);
}
 
void arret() {
  vitesseActuelleGauche = 0;
  vitesseActuelleDroite = 0;
  moteurGauche.run(RELEASE);
  moteurDroit.run(RELEASE);
}
 
int calculerVitesseAdaptative(long distance, float confiance) {
  if (distance < 0 || distance > SEUIL_DETECTION_CM) {
    return VITESSE_MAX;
  } else if (distance < SEUIL_CRITIQUE_CM) {
    return VITESSE_MIN;
  } else if (distance < SEUIL_ALERTE_CM) {
    float ratio = (float)(distance - SEUIL_CRITIQUE_CM) / (SEUIL_ALERTE_CM - SEUIL_CRITIQUE_CM);
    int vitesse = VITESSE_MIN + (int)(ratio * (VITESSE_MAX - VITESSE_MIN) * 0.25);
    return (int)(vitesse * confiance + VITESSE_MIN * (1.0 - confiance));
  } else {
    float ratio = (float)(distance - SEUIL_ALERTE_CM) / (SEUIL_DETECTION_CM - SEUIL_ALERTE_CM);
    int vitesse = VITESSE_MIN + (int)(ratio * (VITESSE_MAX - VITESSE_MIN) * 0.45) + (VITESSE_MAX - VITESSE_MIN) * 0.55;
    return (int)(vitesse * confiance + VITESSE_MAX * (1.0 - confiance));
  }
}

int calculerAngleRotation(long distance, int vitesseActuelle) {
  if (distance >= SEUIL_DETECTION_CM || distance < 0) {
    return 0;
  }
  
  int angleBase = 0;
  int vitesseMoyenne = (vitesseActuelleGauche + vitesseActuelleDroite) / 2;
  float facteurVitesse = 1.0 + ((float)vitesseMoyenne / VITESSE_MAX) * 0.3;
  
  if (distance < SEUIL_CRITIQUE_CM) {
    angleBase = 85 + (compteurObstacle * 8) + (patternObstacle * 5);
    if (angleBase > 130) angleBase = 130;
  } else if (distance < SEUIL_ALERTE_CM) {
    float ratio = (float)(SEUIL_ALERTE_CM - distance) / (SEUIL_ALERTE_CM - SEUIL_CRITIQUE_CM);
    angleBase = 65 + (int)(ratio * 20);
  } else {
    float ratio = (float)(SEUIL_DETECTION_CM - distance) / (SEUIL_DETECTION_CM - SEUIL_ALERTE_CM);
    angleBase = (int)(ratio * 65);
  }
  
  return (int)(angleBase * facteurVitesse);
}

float calculerTendance(long distance) {
  if (nbMesuresValides < 5) {
    return 0.0;
  }
  
  float somme = 0;
  int count = 0;
  float poidsTotal = 0;
  
  for (int i = 1; i < nbMesuresValides && i < 10; i++) {
    int idx1 = (indexHistorique - i - 1 + 12) % 12;
    int idx2 = (indexHistorique - i + 12) % 12;
    if (historiqueDistance[idx1] > 0 && historiqueDistance[idx2] > 0 && 
        historiqueDistance[idx1] < 400 && historiqueDistance[idx2] < 400) {
      float poids = 1.0 / (i + 1);
      somme += (historiqueDistance[idx2] - historiqueDistance[idx1]) * poids;
      poidsTotal += poids;
      count++;
    }
  }
  
  return count > 0 ? (somme / poidsTotal) : 0.0;
}

float calculerAcceleration(long distance) {
  if (nbMesuresValides < 6) {
    return 0.0;
  }
  
  float tendance1 = 0, tendance2 = 0;
  int count1 = 0, count2 = 0;
  
  for (int i = 1; i < 4 && i < nbMesuresValides; i++) {
    int idx1 = (indexHistorique - i - 1 + 12) % 12;
    int idx2 = (indexHistorique - i + 12) % 12;
    if (historiqueDistance[idx1] < 400 && historiqueDistance[idx2] < 400) {
      tendance1 += (historiqueDistance[idx2] - historiqueDistance[idx1]);
      count1++;
    }
  }
  
  for (int i = 4; i < 7 && i < nbMesuresValides; i++) {
    int idx1 = (indexHistorique - i - 1 + 12) % 12;
    int idx2 = (indexHistorique - i + 12) % 12;
    if (historiqueDistance[idx1] < 400 && historiqueDistance[idx2] < 400) {
      tendance2 += (historiqueDistance[idx2] - historiqueDistance[idx1]);
      count2++;
    }
  }
  
  if (count1 > 0 && count2 > 0) {
    tendance1 /= count1;
    tendance2 /= count2;
    return tendance1 - tendance2;
  }
  
  return 0.0;
}

long detecterChuteBrutale(long distanceActuelle, long distanceMoyenne) {
  if (distancePrecedente > 0 && distancePrecedente < 400 && distanceActuelle < 400) {
    long chute = distancePrecedente - distanceActuelle;
    int vitesseMoyenne = (vitesseActuelleGauche + vitesseActuelleDroite) / 2;
    int seuilChute = 12 + (vitesseMoyenne / 20);
    
    if (chute > seuilChute && distanceActuelle < SEUIL_DETECTION_CM) {
      return distanceActuelle;
    }
  }
  return distanceMoyenne;
}

void detecterPatternObstacle(long distance) {
  unsigned long maintenant = millis();
  
  if (distance < SEUIL_ALERTE_CM && distance > 1) {
    if (dernierObstacleTemps > 0 && (maintenant - dernierObstacleTemps) < 2000) {
      long diffDistance = abs(distance - dernierObstacleDistance);
      if (diffDistance < 15) {
        patternObstacle++;
        if (patternObstacle > 5) patternObstacle = 5;
      } else {
        patternObstacle = max(0, patternObstacle - 1);
      }
    }
    dernierObstacleDistance = distance;
    dernierObstacleTemps = maintenant;
  } else {
    if ((maintenant - dernierObstacleTemps) > 3000) {
      patternObstacle = max(0, patternObstacle - 1);
    }
  }
}

long calculerDistanceUtilisee(long distance, long distanceMoyenne, float tendance, float acceleration) {
  long distancePredite = distanceMoyenne + (long)(tendance * 2.5) + (long)(acceleration * 1.5);
  
  if (distancePredite < 0) distancePredite = distanceMoyenne;
  if (distancePredite > 400) distancePredite = 400;
  
  long distanceChute = detecterChuteBrutale(distance, distanceMoyenne);
  if (distanceChute != distanceMoyenne) {
    return distanceChute;
  }
  
  float poidsPrediction = confianceMesure * 0.6;
  float poidsMoyenne = (1.0 - confianceMesure) * 0.3;
  float poidsActuelle = 0.1;
  
  return (long)(distancePredite * poidsPrediction + distanceMoyenne * poidsMoyenne + distance * poidsActuelle);
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  moteurGauche.setSpeed(VITESSE_MAX);
  moteurDroit.setSpeed(VITESSE_MAX);
  arret();
  delay(1000);
  
  for (int i = 0; i < 12; i++) {
    historiqueDistance[i] = 400;
  }
  
  Serial.println("Robot ultra perfectionne pret");
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
  indexHistorique = (indexHistorique + 1) % 12;
  if (nbMesuresValides < 12) nbMesuresValides++;

  long distanceMoyenne = 0;
  int count = 0;
  for (int i = 0; i < nbMesuresValides && i < 8; i++) {
    int idx = (indexHistorique - i - 1 + 12) % 12;
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
  float acceleration = calculerAcceleration(distance);
  long distanceUtilisee = calculerDistanceUtilisee(distance, distanceMoyenne, tendance, acceleration);
  
  detecterPatternObstacle(distanceUtilisee);

  long variation = distance - distancePrecedente;
  unsigned long maintenant = millis();
  
  Serial.print("Dist:");
  Serial.print(distance);
  Serial.print(" use:");
  Serial.print(distanceUtilisee);
  Serial.print(" conf:");
  Serial.print((int)(confianceMesure * 100));
  Serial.print(" pat:");
  Serial.print(patternObstacle);
  if (variation != 0) {
    Serial.print(" var:");
    if (variation > 0) Serial.print("+");
    Serial.print(variation);
  }
  
  distancePrecedente = distance;
  
  if (etat == AVANCE_DROIT) {
    if (distanceUtilisee < SEUIL_CRITIQUE_CM && distanceUtilisee > 1) {
      Serial.println(" -> CRITIQUE!");
      compteurObstacle++;
      etat = EVITER_OBSTACLE;
      tempsDebut = maintenant;
      dernierChangementEtat = maintenant;
    arret();
      delay(25);
    } else if (distanceUtilisee < SEUIL_ALERTE_CM && distanceUtilisee > SEUIL_CRITIQUE_CM) {
      if (etat != CORRECTION_FORTE) {
        Serial.println(" -> CORRECTION FORTE");
        etat = CORRECTION_FORTE;
        dernierChangementEtat = maintenant;
      }
      
      int angle = calculerAngleRotation(distanceUtilisee, (vitesseActuelleGauche + vitesseActuelleDroite) / 2);
      int vitesse = calculerVitesseAdaptative(distanceUtilisee, confianceMesure);
      int correction = (angle * vitesse) / 110;
      if (correction < 10) correction = 10;
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
        dernierChangementEtat = maintenant;
      }
      
      int angle = calculerAngleRotation(distanceUtilisee, (vitesseActuelleGauche + vitesseActuelleDroite) / 2);
      int vitesse = calculerVitesseAdaptative(distanceUtilisee, confianceMesure);
      int correction = (angle * vitesse) / 220;
      if (correction < 6) correction = 6;
      
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
        dernierChangementEtat = maintenant;
        compteurObstacle = 0;
      }
      int vitesse = calculerVitesseAdaptative(distanceUtilisee, confianceMesure);
      avancerDroit(vitesse, vitesse);
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
      delay(25);
    } else {
      int angle = calculerAngleRotation(distanceUtilisee, (vitesseActuelleGauche + vitesseActuelleDroite) / 2);
      int vitesse = calculerVitesseAdaptative(distanceUtilisee, confianceMesure);
      int correction = (etat == CORRECTION_FORTE) ? ((angle * vitesse) / 110) : ((angle * vitesse) / 220);
      if (correction < (etat == CORRECTION_FORTE ? 10 : 6)) correction = (etat == CORRECTION_FORTE ? 10 : 6);
      
      if (tournerDroite) {
        avancerDroit(vitesse, vitesse - correction);
      } else {
        avancerDroit(vitesse - correction, vitesse);
      }
      appliquerVitesseRamp();
    }
  } else if (etat == EVITER_OBSTACLE) {
    unsigned long tempsEcoule = maintenant - tempsDebut;

    if (tempsEcoule < 140) {
      reculer();
      Serial.println(" -> RECUL");
    } else if (tempsEcoule < 190) {
    arret();
      Serial.println(" -> ARRET");
    } else {
      int angle = calculerAngleRotation(distanceUtilisee, VITESSE_MAX);
      if (angle < 55) angle = 65;
      if (angle > 130) angle = 130;
      
      Serial.print(" -> TOURNE ");
      Serial.print(tournerDroite ? "DROITE" : "GAUCHE");
      Serial.print(" (");
      Serial.print(angle);
      Serial.println("deg)");
      
      tournerAngle(tournerDroite, angle);
    tournerDroite = !tournerDroite;
      etat = AVANCE_DROIT;
      dernierChangementEtat = maintenant;
    }
  }
  
  delay(6);
}
