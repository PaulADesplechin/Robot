#include <AFMotor.h>

const int trigPin = 12;
const int echoPin = 13;
AF_DCMotor moteurGauche(3);
AF_DCMotor moteurDroit(4);

const int VITESSE_MAX = 180;
const int VITESSE_MIN = 165;
const float FACTEUR_DROIT = 0.98;
const int SEUIL_DETECTION = 110;
const int SEUIL_ALERTE = 60;
const int SEUIL_CRITIQUE = 30;

enum Etat { AVANCE, CORRECTION, EVITER };
Etat etat = AVANCE;

unsigned long tempsDebut = 0;
bool tournerDroite = true;
long distancePrec = 400;
long historique[5];
int idxHist = 0;
int compteurBlocage = 0;
unsigned long dernierObstacle = 0;
int vitesseG = 0, vitesseD = 0;

long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(12);
  digitalWrite(trigPin, LOW);
  long duree = pulseIn(echoPin, HIGH, 25000);
  if (duree == 0) return 400;
  long dist = (duree * 0.034 / 2);
  return (dist < 2 || dist > 400) ? 400 : dist;
}

long getDistanceFiltree() {
  long m[3];
  m[0] = getDistance();
  delayMicroseconds(1000);
  m[1] = getDistance();
  delayMicroseconds(1000);
  m[2] = getDistance();
  if (m[0] > m[1]) { long t = m[0]; m[0] = m[1]; m[1] = t; }
  if (m[1] > m[2]) { long t = m[1]; m[1] = m[2]; m[2] = t; }
  if (m[0] > m[1]) { long t = m[0]; m[0] = m[1]; m[1] = t; }
  return m[1];
}

void setMoteurs(int g, int d) {
  vitesseG = g;
  vitesseD = d;
  int gFinal = (int)(vitesseG * 1.0);
  int dFinal = (int)(vitesseD * FACTEUR_DROIT);
  if (g == d && etat == AVANCE) {
    int moy = (gFinal + dFinal) / 2;
    gFinal = dFinal = moy;
  }
  moteurGauche.setSpeed(gFinal);
  moteurDroit.setSpeed(dFinal);
  moteurGauche.run(FORWARD);
  moteurDroit.run(FORWARD);
}

void reculer() {
  vitesseG = vitesseD = 0;
  moteurGauche.setSpeed(175);
  moteurDroit.setSpeed((int)(175 * FACTEUR_DROIT));
  moteurGauche.run(BACKWARD);
  moteurDroit.run(BACKWARD);
}

void tourner(bool droite, int angle) {
  vitesseG = vitesseD = 0;
  moteurGauche.setSpeed(175);
  moteurDroit.setSpeed((int)(175 * FACTEUR_DROIT));
  if (droite) {
    moteurGauche.run(FORWARD);
    moteurDroit.run(BACKWARD);
  } else {
    moteurGauche.run(BACKWARD);
    moteurDroit.run(FORWARD);
  }
  delay(angle);
  moteurGauche.run(RELEASE);
  moteurDroit.run(RELEASE);
  delay(10);
}

void arret() {
  vitesseG = vitesseD = 0;
  moteurGauche.run(RELEASE);
  moteurDroit.run(RELEASE);
}

int calculerVitesse(long dist) {
  if (dist > SEUIL_DETECTION) return VITESSE_MAX;
  if (dist < SEUIL_CRITIQUE) return VITESSE_MIN;
  if (dist < SEUIL_ALERTE) {
    float r = (float)(dist - SEUIL_CRITIQUE) / (SEUIL_ALERTE - SEUIL_CRITIQUE);
    return VITESSE_MIN + (int)(r * (VITESSE_MAX - VITESSE_MIN) * 0.3);
  }
  float r = (float)(dist - SEUIL_ALERTE) / (SEUIL_DETECTION - SEUIL_ALERTE);
  return VITESSE_MIN + (int)(r * (VITESSE_MAX - VITESSE_MIN) * 0.7);
}

int calculerAngle(long dist, bool angleDetecte) {
  if (dist >= SEUIL_DETECTION) return 0;
  if (angleDetecte || compteurBlocage > 2) {
    if (compteurBlocage > 3) return 180;
    return compteurBlocage > 2 ? 150 : 120;
  }
  if (dist < SEUIL_CRITIQUE) {
    int a = 90 + compteurBlocage * 15;
    return a > 150 ? 150 : a;
  }
  if (dist < SEUIL_ALERTE) {
    float r = (float)(SEUIL_ALERTE - dist) / (SEUIL_ALERTE - SEUIL_CRITIQUE);
    return 65 + (int)(r * 25) + compteurBlocage * 8;
  }
  float r = (float)(SEUIL_DETECTION - dist) / (SEUIL_DETECTION - SEUIL_ALERTE);
  return (int)(r * 60);
}

bool detecterAngle(long distMoy) {
  if (distMoy >= SEUIL_ALERTE || distMoy <= 1) return false;
  long var = 0;
  int c = 0;
  for (int i = 1; i < 4 && i < idxHist; i++) {
    int idx1 = (idxHist - i - 1 + 5) % 5;
    int idx2 = (idxHist - i + 5) % 5;
    if (historique[idx1] < 400 && historique[idx2] < 400) {
      var += abs(historique[idx1] - historique[idx2]);
      c++;
    }
  }
  return (c > 0 && var / c < 5);
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  moteurGauche.setSpeed(VITESSE_MAX);
  moteurDroit.setSpeed(VITESSE_MAX);
  arret();
  delay(300);
  for (int i = 0; i < 5; i++) historique[i] = 400;
  Serial.println("Robot pret");
}

void loop() {
  long dist = getDistanceFiltree();
  if (dist < 2 || dist > 400) dist = (distancePrec > 0 && distancePrec < 400) ? distancePrec : 400;
  
  historique[idxHist] = dist;
  idxHist = (idxHist + 1) % 5;
  
  long distMoy = 0;
  int c = 0;
  for (int i = 0; i < 5; i++) {
    if (historique[i] > 0 && historique[i] < 400) {
      distMoy += historique[i];
      c++;
    }
  }
  distMoy = c > 0 ? distMoy / c : dist;
  
  long distUse = (distancePrec > 0 && distancePrec < 400 && dist < 400 && distancePrec - dist > 8 && dist < SEUIL_DETECTION) ? dist : (distMoy * 2 + dist) / 3;
  unsigned long maintenant = millis();
  bool angleDetecte = detecterAngle(distMoy);
  
  if (distUse < SEUIL_CRITIQUE && distUse > 1) {
    if (maintenant - dernierObstacle < 1500) {
      compteurBlocage++;
      if (compteurBlocage > 5) compteurBlocage = 5;
    } else {
      compteurBlocage = max(0, compteurBlocage - 1);
    }
    dernierObstacle = maintenant;
  } else if (distUse > SEUIL_DETECTION) {
    compteurBlocage = 0;
  }
  if (angleDetecte) compteurBlocage = max(compteurBlocage, 2);
  
  Serial.print("Dist:"); Serial.print(dist);
  Serial.print(" use:"); Serial.print(distUse);
  Serial.print(" v:"); Serial.print((vitesseG + vitesseD) / 2);
  if (compteurBlocage > 0) { Serial.print(" bloc:"); Serial.print(compteurBlocage); }
  if (angleDetecte) Serial.print(" ANGLE");
  
  distancePrec = dist;
  
  if (etat == AVANCE) {
    if (distUse < SEUIL_CRITIQUE && distUse > 1) {
      Serial.println(" -> CRITIQUE!");
      etat = EVITER;
      tempsDebut = maintenant;
      arret();
      delay(5);
    } else if (distUse < SEUIL_DETECTION) {
      if (etat != CORRECTION) {
        Serial.println(" -> CORRECTION");
        etat = CORRECTION;
      }
      int angle = calculerAngle(distUse, angleDetecte);
      int vit = calculerVitesse(distUse);
      int corr = (angle * vit) / (distUse < SEUIL_ALERTE ? 90 : 150);
      if (corr < (distUse < SEUIL_ALERTE ? 15 : 10)) corr = (distUse < SEUIL_ALERTE ? 15 : 10);
      if (tournerDroite) setMoteurs(vit, max(VITESSE_MIN, vit - corr));
      else setMoteurs(max(VITESSE_MIN, vit - corr), vit);
    } else {
      if (etat != AVANCE) {
        Serial.println(" -> AVANCE");
        etat = AVANCE;
        compteurBlocage = max(0, compteurBlocage - 1);
      }
      int vit = calculerVitesse(distUse);
      setMoteurs(vit, vit);
    }
  } else if (etat == CORRECTION) {
    if (distUse >= SEUIL_DETECTION) {
      etat = AVANCE;
    } else if (distUse < SEUIL_CRITIQUE && distUse > 1) {
      etat = EVITER;
      tempsDebut = maintenant;
      arret();
      delay(5);
    } else {
      int angle = calculerAngle(distUse, angleDetecte);
      int vit = calculerVitesse(distUse);
      int corr = (angle * vit) / (distUse < SEUIL_ALERTE ? 90 : 150);
      if (corr < (distUse < SEUIL_ALERTE ? 15 : 10)) corr = (distUse < SEUIL_ALERTE ? 15 : 10);
      if (tournerDroite) setMoteurs(vit, vit - corr);
      else setMoteurs(vit - corr, vit);
    }
  } else if (etat == EVITER) {
    unsigned long t = maintenant - tempsDebut;
    int dureeRecul = (angleDetecte || compteurBlocage > 2) ? 150 : (compteurBlocage > 1 ? 120 : 80);
    int dureeArret = (angleDetecte || compteurBlocage > 2) ? 120 : (compteurBlocage > 1 ? 110 : 100);
    
    if (t < dureeRecul) {
      reculer();
      Serial.println(" -> RECUL");
    } else if (t < dureeArret) {
      arret();
      Serial.println(" -> ARRET");
    } else {
      int angle = calculerAngle(distUse, angleDetecte);
      if (compteurBlocage > 3) angle = 180;
      else if (compteurBlocage > 2 || angleDetecte) {
        if (angle < 100) angle = 120;
        if (angle > 180) angle = 180;
      } else {
        if (angle < 60) angle = 70;
        if (angle > 150) angle = 150;
      }
      Serial.print(" -> TOURNE "); Serial.print(tournerDroite ? "DROITE" : "GAUCHE");
      Serial.print(" ("); Serial.print(angle); Serial.println("deg)");
      tourner(tournerDroite, angle);
      tournerDroite = !tournerDroite;
      etat = AVANCE;
    }
  }
  
  delay(2);
}
