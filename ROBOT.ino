#include <AFMotor.h>

// ===== CONFIGURATION DES PINS =====
const int trigPin = 12;  // Pin du capteur ultrason (envoi)
const int echoPin = 13;  // Pin du capteur ultrason (réception)

// ===== CONFIGURATION DES MOTEURS =====
AF_DCMotor moteurGauche(3);  // Moteur gauche sur port 3
AF_DCMotor moteurDroit(4);   // Moteur droit sur port 4

// ===== PARAMETRES DE VITESSE =====
const int VITESSE_MAX = 180;        // Vitesse maximale (0-255)
const int VITESSE_MIN = 165;        // Vitesse minimale pour éviter les obstacles
const float FACTEUR_COMPENSATION_GAUCHE = 1.0;   // Ajustement moteur gauche
const float FACTEUR_COMPENSATION_DROIT = 0.98;    // Ajustement moteur droit (légèrement plus lent)

// ===== SEUILS DE DETECTION (en cm) =====
const int SEUIL_DETECTION_CM = 110;   // Distance où on commence à détecter un obstacle
const int SEUIL_ALERTE_CM = 60;       // Distance où on ralentit et corrige la trajectoire
const int SEUIL_CRITIQUE_CM = 30;     // Distance critique : on doit reculer et tourner

// ===== ETATS DU ROBOT =====
enum EtatRobot {
  AVANCE_DROIT,        // Avance tout droit
  CORRECTION_LEGERE,   // Légère correction pour éviter un obstacle lointain
  CORRECTION_FORTE,    // Correction plus importante pour un obstacle proche
  EVITER_OBSTACLE      // Recule et tourne pour éviter un obstacle critique
};

EtatRobot etat = AVANCE_DROIT;  // État actuel du robot

// ===== VARIABLES GLOBALES =====
unsigned long tempsDebut = 0;           // Temps de début d'une manœuvre
bool tournerDroite = true;              // Direction de rotation (true = droite)
long distancePrecedente = 400;          // Dernière distance mesurée
long historiqueDistance[8];            // Historique des 8 dernières mesures
int indexHistorique = 0;                // Index pour l'historique
int nbMesuresValides = 0;               // Nombre de mesures valides dans l'historique
int compteurObstacle = 0;               // Nombre d'obstacles rencontrés
int compteurBlocage = 0;                // Compteur de blocage (si plusieurs obstacles rapides)
unsigned long dernierObstacleTemps = 0; // Temps du dernier obstacle détecté
int vitesseActuelleGauche = 0;          // Vitesse actuelle moteur gauche
int vitesseActuelleDroite = 0;         // Vitesse actuelle moteur droit
int vitesseCibleGauche = 0;            // Vitesse cible moteur gauche
int vitesseCibleDroite = 0;            // Vitesse cible moteur droit
int tentativesMemeDirection = 0;       // Compteur pour éviter les oscillations

// ===== FONCTION : MESURER LA DISTANCE =====
long getDistance() {
  // Envoie une impulsion ultrasonique
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(12);
  digitalWrite(trigPin, LOW);
  
  // Attend le retour de l'écho (max 25ms)
  long duree = pulseIn(echoPin, HIGH, 25000);
  
  // Si pas de retour, retourne 400 (distance invalide)
  if (duree == 0) {
    return 400;
  }
  
  // Calcule la distance : vitesse du son = 0.034 cm/microseconde
  long distance = (duree * 0.034 / 2);
  
  // Vérifie que la distance est valide (entre 2cm et 400cm)
  if (distance < 2 || distance > 400) {
    return 400;
  }
  
  return distance;
}

// ===== FONCTION : MESURE FILTREE (plus précise) =====
long getDistanceFiltree() {
  // Prend 3 mesures
  long mesures[3];
  mesures[0] = getDistance();
  delayMicroseconds(1000);
  mesures[1] = getDistance();
  delayMicroseconds(1000);
  mesures[2] = getDistance();
  
  // Trie les mesures pour trouver la médiane (valeur du milieu)
  // Cela élimine les erreurs de mesure
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
  
  // Retourne la médiane (valeur du milieu)
  return mesures[1];
}

// ===== FONCTION : DEFINIR LA VITESSE DES MOTEURS =====
void avancerDroit(int vitesseGauche, int vitesseDroite) {
  vitesseCibleGauche = vitesseGauche;
  vitesseCibleDroite = vitesseDroite;
}

// ===== FONCTION : APPLIQUER LA VITESSE AVEC RAMPE (mouvement fluide) =====
void appliquerVitesseRamp() {
  // Calcule la différence entre vitesse actuelle et cible
  int diffG = vitesseCibleGauche - vitesseActuelleGauche;
  int diffD = vitesseCibleDroite - vitesseActuelleDroite;
  
  // Ajuste progressivement la vitesse gauche (rampe)
  if (abs(diffG) > 3) {
    int step = max(3, abs(diffG) / 3);  // Pas d'ajustement
    if (diffG > 0) {
      vitesseActuelleGauche = min(vitesseCibleGauche, vitesseActuelleGauche + step);
    } else {
      vitesseActuelleGauche = max(vitesseCibleGauche, vitesseActuelleGauche - step);
    }
  } else {
    vitesseActuelleGauche = vitesseCibleGauche;
  }
  
  // Ajuste progressivement la vitesse droite (rampe)
  if (abs(diffD) > 3) {
    int step = max(3, abs(diffD) / 3);
    if (diffD > 0) {
      vitesseActuelleDroite = min(vitesseCibleDroite, vitesseActuelleDroite + step);
    } else {
      vitesseActuelleDroite = max(vitesseCibleDroite, vitesseActuelleDroite - step);
    }
  } else {
    vitesseActuelleDroite = vitesseCibleDroite;
  }
  
  // Synchronise les moteurs si on avance tout droit
  if (vitesseCibleGauche == vitesseCibleDroite && etat == AVANCE_DROIT) {
    int vitesseMoyenne = (vitesseActuelleGauche + vitesseActuelleDroite) / 2;
    vitesseActuelleGauche = vitesseMoyenne;
    vitesseActuelleDroite = vitesseMoyenne;
  }
  
  // Applique les facteurs de compensation pour équilibrer les moteurs
  int vitesseGaucheFinale = (int)(vitesseActuelleGauche * FACTEUR_COMPENSATION_GAUCHE);
  int vitesseDroiteFinale = (int)(vitesseActuelleDroite * FACTEUR_COMPENSATION_DROIT);
  
  // Synchronise encore une fois après compensation
  if (vitesseCibleGauche == vitesseCibleDroite && etat == AVANCE_DROIT) {
    int vitesseMoyenneFinale = (vitesseGaucheFinale + vitesseDroiteFinale) / 2;
    vitesseGaucheFinale = vitesseMoyenneFinale;
    vitesseDroiteFinale = vitesseMoyenneFinale;
  }
  
  // Applique les vitesses aux moteurs
  moteurGauche.setSpeed(vitesseGaucheFinale);
  moteurDroit.setSpeed(vitesseDroiteFinale);
  moteurGauche.run(FORWARD);
  moteurDroit.run(FORWARD);
}

// ===== FONCTION : RECULER =====
void reculer() {
  vitesseActuelleGauche = 0;
  vitesseActuelleDroite = 0;
  int vitesseRecul = 175;
  
  // Applique la vitesse avec compensation
  moteurGauche.setSpeed((int)(vitesseRecul * FACTEUR_COMPENSATION_GAUCHE));
  moteurDroit.setSpeed((int)(vitesseRecul * FACTEUR_COMPENSATION_DROIT));
  moteurGauche.run(BACKWARD);
  moteurDroit.run(BACKWARD);
}

// ===== FONCTION : TOURNER D'UN ANGLE =====
void tournerAngle(bool droite, int angleDegres) {
  vitesseActuelleGauche = 0;
  vitesseActuelleDroite = 0;
  int vitesse = 175;
  
  // Applique la vitesse avec compensation
  moteurGauche.setSpeed((int)(vitesse * FACTEUR_COMPENSATION_GAUCHE));
  moteurDroit.setSpeed((int)(vitesse * FACTEUR_COMPENSATION_DROIT));
  
  // Fait tourner le robot
  if (droite) {
    moteurGauche.run(FORWARD);   // Moteur gauche avance
    moteurDroit.run(BACKWARD);   // Moteur droit recule
  } else {
    moteurGauche.run(BACKWARD);  // Moteur gauche recule
    moteurDroit.run(FORWARD);    // Moteur droit avance
  }
  
  // Attend le temps nécessaire pour tourner de l'angle demandé
  unsigned long duree = (angleDegres * 1);  // 1ms par degré
  delay(duree);
  
  // Arrête les moteurs
  moteurGauche.run(RELEASE);
  moteurDroit.run(RELEASE);
  delay(10);
}

// ===== FONCTION : ARRETER LES MOTEURS =====
void arret() {
  vitesseActuelleGauche = 0;
  vitesseActuelleDroite = 0;
  moteurGauche.run(RELEASE);
  moteurDroit.run(RELEASE);
}

// ===== FONCTION : CALCULER LA VITESSE SELON LA DISTANCE =====
int calculerVitesseAdaptative(long distance) {
  // Si très loin ou invalide : vitesse max
  if (distance < 0 || distance > SEUIL_DETECTION_CM) {
    return VITESSE_MAX;
  }
  // Si très proche : vitesse min
  else if (distance < SEUIL_CRITIQUE_CM) {
    return VITESSE_MIN;
  }
  // Si zone d'alerte : vitesse intermédiaire
  else if (distance < SEUIL_ALERTE_CM) {
    float ratio = (float)(distance - SEUIL_CRITIQUE_CM) / (SEUIL_ALERTE_CM - SEUIL_CRITIQUE_CM);
    return VITESSE_MIN + (int)(ratio * (VITESSE_MAX - VITESSE_MIN) * 0.25);
  }
  // Si zone de détection : vitesse presque max
  else {
    float ratio = (float)(distance - SEUIL_ALERTE_CM) / (SEUIL_DETECTION_CM - SEUIL_ALERTE_CM);
    return VITESSE_MIN + (int)(ratio * (VITESSE_MAX - VITESSE_MIN) * 0.4) + (VITESSE_MAX - VITESSE_MIN) * 0.6;
  }
}

// ===== FONCTION : DETECTER SI ON EST DANS UN ANGLE RESSERRE =====
bool detecterAngleResserre(long distanceMoyenne, long distance) {
  // Vérifie si on est proche d'un obstacle
  if (distanceMoyenne < SEUIL_ALERTE_CM && distanceMoyenne > 1) {
    // Calcule la variation des mesures récentes
    long variation = 0;
    int count = 0;
    for (int i = 1; i < nbMesuresValides && i < 5; i++) {
      int idx1 = (indexHistorique - i - 1 + 8) % 8;
      int idx2 = (indexHistorique - i + 8) % 8;
      if (historiqueDistance[idx1] < 400 && historiqueDistance[idx2] < 400) {
        variation += abs(historiqueDistance[idx1] - historiqueDistance[idx2]);
        count++;
      }
    }
    // Si la variation est faible (< 5cm), on est probablement dans un angle
    if (count > 0) {
      float variationMoyenne = variation / count;
      if (variationMoyenne < 5 && distanceMoyenne < SEUIL_ALERTE_CM) {
        return true;
      }
    }
  }
  return false;
}

// ===== FONCTION : CALCULER L'ANGLE DE ROTATION =====
int calculerAngleRotation(long distance, bool estAngle) {
  // Si pas d'obstacle : pas de rotation
  if (distance >= SEUIL_DETECTION_CM || distance < 0) {
    return 0;
  }
  
  // Si on est dans un angle ou très bloqué : rotation importante
  if (estAngle || compteurBlocage > 2) {
    if (compteurBlocage > 3) {
      return 180;  // Rotation complète
    } else if (compteurBlocage > 2) {
      return 150;  // Rotation importante
    } else {
      return 120;  // Rotation moyenne
    }
  }
  
  // Calcule l'angle selon la distance
  int angleBase = 0;
  
  if (distance < SEUIL_CRITIQUE_CM) {
    // Zone critique : angle important
    angleBase = 90 + (compteurObstacle * 6) + (compteurBlocage * 12);
    if (angleBase > 150) angleBase = 150;
  } else if (distance < SEUIL_ALERTE_CM) {
    // Zone d'alerte : angle moyen
    float ratio = (float)(SEUIL_ALERTE_CM - distance) / (SEUIL_ALERTE_CM - SEUIL_CRITIQUE_CM);
    angleBase = 65 + (int)(ratio * 25) + (compteurBlocage * 8);
  } else {
    // Zone de détection : angle léger
    float ratio = (float)(SEUIL_DETECTION_CM - distance) / (SEUIL_DETECTION_CM - SEUIL_ALERTE_CM);
    angleBase = (int)(ratio * 60);
  }
  
  return angleBase;
}

// ===== FONCTION : CALCULER LA DISTANCE UTILISEE (filtrée) =====
long calculerDistanceUtilisee(long distance, long distanceMoyenne) {
  // Si chute brutale de distance : utilise la distance actuelle directement
  if (distancePrecedente > 0 && distancePrecedente < 400 && distance < 400) {
    long chute = distancePrecedente - distance;
    if (chute > 8 && distance < SEUIL_DETECTION_CM) {
      return distance;
    }
  }
  
  // Sinon : moyenne pondérée (favorise la moyenne)
  return (distanceMoyenne * 2 + distance) / 3;
}

// ===== SETUP : INITIALISATION =====
void setup() {
  Serial.begin(9600);  // Démarre la communication série pour le debug
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Initialise les moteurs
  moteurGauche.setSpeed(VITESSE_MAX);
  moteurDroit.setSpeed(VITESSE_MAX);
  arret();
  delay(300);
  
  // Initialise l'historique des distances
  for (int i = 0; i < 8; i++) {
    historiqueDistance[i] = 400;
  }
  
  Serial.println("Robot intelligent pret");
}

// ===== LOOP : BOUCLE PRINCIPALE =====
void loop() {
  // 1. MESURE LA DISTANCE
  long distance = getDistanceFiltree();
  
  // Si mesure invalide : utilise la précédente
  if (distance < 2 || distance > 400) {
    if (distancePrecedente > 0 && distancePrecedente < 400) {
      distance = distancePrecedente;
    } else {
      distance = 400;
    }
  }
  
  // 2. MET A JOUR L'HISTORIQUE
  historiqueDistance[indexHistorique] = distance;
  indexHistorique = (indexHistorique + 1) % 8;
  if (nbMesuresValides < 8) nbMesuresValides++;
  
  // 3. CALCULE LA DISTANCE MOYENNE (sur les 5 dernières mesures)
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
  
  // 4. CALCULE LA DISTANCE UTILISEE (filtrée)
  long distanceUtilisee = calculerDistanceUtilisee(distance, distanceMoyenne);
  unsigned long maintenant = millis();
  
  // 5. DETECTE SI ON EST DANS UN ANGLE RESSERRE
  bool estAngleResserre = detecterAngleResserre(distanceMoyenne, distance);
  
  // 6. MET A JOUR LE COMPTEUR DE BLOCAGE
  if (distanceUtilisee < SEUIL_CRITIQUE_CM && distanceUtilisee > 1) {
    // Si obstacle détecté récemment : augmente le compteur
    if (maintenant - dernierObstacleTemps < 1500) {
      compteurBlocage++;
      if (compteurBlocage > 5) compteurBlocage = 5;
    } else {
      compteurBlocage = max(0, compteurBlocage - 1);
    }
    dernierObstacleTemps = maintenant;
  } else if (distanceUtilisee > SEUIL_DETECTION_CM) {
    // Si plus d'obstacle : réinitialise
    compteurBlocage = 0;
    tentativesMemeDirection = 0;
  }
  
  // Si angle détecté : force un niveau de blocage minimum
  if (estAngleResserre) {
    compteurBlocage = max(compteurBlocage, 2);
  }
  
  // 7. AFFICHE LES INFORMATIONS DE DEBUG
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
  if (estAngleResserre) {
    Serial.print(" ANGLE");
  }
  
  distancePrecedente = distance;
  
  // 8. GESTION DES ETATS DU ROBOT
  
  // ETAT : AVANCE DROIT
  if (etat == AVANCE_DROIT) {
    if (distanceUtilisee < SEUIL_CRITIQUE_CM && distanceUtilisee > 1) {
      // Obstacle critique détecté : passe en mode évitement
      Serial.println(" -> CRITIQUE!");
      compteurObstacle++;
      etat = EVITER_OBSTACLE;
      tempsDebut = maintenant;
      arret();
      delay(5);
    } else if (distanceUtilisee < SEUIL_ALERTE_CM && distanceUtilisee > SEUIL_CRITIQUE_CM) {
      // Zone d'alerte : correction forte
      if (etat != CORRECTION_FORTE) {
        Serial.println(" -> CORRECTION FORTE");
        etat = CORRECTION_FORTE;
      }
      
      int angle = calculerAngleRotation(distanceUtilisee, estAngleResserre);
      int vitesse = calculerVitesseAdaptative(distanceUtilisee);
      int correction = (angle * vitesse) / 90;
      if (correction < 15) correction = 15;
      if (correction > vitesse - VITESSE_MIN) correction = vitesse - VITESSE_MIN;
      
      // Tourne légèrement pour éviter l'obstacle
      if (tournerDroite) {
        avancerDroit(vitesse, max(VITESSE_MIN, vitesse - correction));
      } else {
        avancerDroit(max(VITESSE_MIN, vitesse - correction), vitesse);
      }
      appliquerVitesseRamp();
    } else if (distanceUtilisee < SEUIL_DETECTION_CM && distanceUtilisee > SEUIL_ALERTE_CM) {
      // Zone de détection : correction légère
      if (etat != CORRECTION_LEGERE) {
        Serial.println(" -> CORRECTION LEGERE");
        etat = CORRECTION_LEGERE;
      }
      
      int angle = calculerAngleRotation(distanceUtilisee, false);
      int vitesse = calculerVitesseAdaptative(distanceUtilisee);
      int correction = (angle * vitesse) / 150;
      if (correction < 10) correction = 10;
      
      // Tourne très légèrement
      if (tournerDroite) {
        avancerDroit(vitesse, vitesse - correction);
      } else {
        avancerDroit(vitesse - correction, vitesse);
      }
      appliquerVitesseRamp();
    } else {
      // Pas d'obstacle : avance tout droit
      if (etat != AVANCE_DROIT) {
        Serial.println(" -> AVANCE DROIT");
        etat = AVANCE_DROIT;
        compteurObstacle = 0;
        compteurBlocage = max(0, compteurBlocage - 1);
        tentativesMemeDirection = 0;
      }
      int vitesse = calculerVitesseAdaptative(distanceUtilisee);
      vitesseCibleGauche = vitesse;
      vitesseCibleDroite = vitesse;
      vitesseActuelleGauche = vitesse;
      vitesseActuelleDroite = vitesse;
      appliquerVitesseRamp();
    }
  }
  // ETAT : CORRECTION LEGERE OU FORTE
  else if (etat == CORRECTION_LEGERE || etat == CORRECTION_FORTE) {
    if (distanceUtilisee >= SEUIL_DETECTION_CM) {
      // Plus d'obstacle : retour à l'avance droit
      etat = AVANCE_DROIT;
      compteurObstacle = 0;
      tentativesMemeDirection = 0;
    } else if (distanceUtilisee < SEUIL_CRITIQUE_CM && distanceUtilisee > 1) {
      // Obstacle critique : passe en mode évitement
      etat = EVITER_OBSTACLE;
      tempsDebut = maintenant;
      arret();
      delay(5);
    } else {
      // Continue la correction
      int angle = calculerAngleRotation(distanceUtilisee, estAngleResserre);
      int vitesse = calculerVitesseAdaptative(distanceUtilisee);
      int correction = (etat == CORRECTION_FORTE) ? ((angle * vitesse) / 90) : ((angle * vitesse) / 150);
      if (correction < (etat == CORRECTION_FORTE ? 15 : 10)) {
        correction = (etat == CORRECTION_FORTE ? 15 : 10);
      }
      
      if (tournerDroite) {
        avancerDroit(vitesse, vitesse - correction);
      } else {
        avancerDroit(vitesse - correction, vitesse);
      }
      appliquerVitesseRamp();
    }
  }
  // ETAT : EVITER OBSTACLE
  else if (etat == EVITER_OBSTACLE) {
    unsigned long tempsEcoule = maintenant - tempsDebut;
    
    // Détermine les durées selon le niveau de blocage
    int dureeRecul = 80;   // Durée du recul (ms)
    int dureeArret = 100;  // Durée de l'arrêt (ms)
    
    if (estAngleResserre || compteurBlocage > 2) {
      // Angle ou blocage important : recul plus long
      dureeRecul = 150;
      dureeArret = 120;
    } else if (compteurBlocage > 1) {
      // Blocage modéré : recul moyen
      dureeRecul = 120;
      dureeArret = 110;
    }
    
    // Phase 1 : RECULER
    if (tempsEcoule < dureeRecul) {
      reculer();
      Serial.println(" -> RECUL");
    }
    // Phase 2 : ARRETER
    else if (tempsEcoule < dureeArret) {
      arret();
      Serial.println(" -> ARRET");
    }
    // Phase 3 : TOURNER
    else {
      int angle = calculerAngleRotation(distanceUtilisee, estAngleResserre);
      
      // Si plusieurs tentatives dans la même direction : change de direction
      if (tentativesMemeDirection > 2) {
        tournerDroite = !tournerDroite;
        tentativesMemeDirection = 0;
        angle = max(angle, 120);
      }
      
      // Ajuste l'angle selon le niveau de blocage
      if (compteurBlocage > 3) {
        angle = 180;  // Rotation complète
        Serial.print(" -> ROTATION COMPLETE ");
      } else if (compteurBlocage > 2 || estAngleResserre) {
        if (angle < 100) angle = 120;
        if (angle > 180) angle = 180;
      } else {
        if (angle < 60) angle = 70;
        if (angle > 150) angle = 150;
      }
      
      Serial.print(" -> TOURNE ");
      Serial.print(tournerDroite ? "DROITE" : "GAUCHE");
      Serial.print(" (");
      Serial.print(angle);
      Serial.println("deg)");
      
      // Effectue la rotation
      tournerAngle(tournerDroite, angle);
      
      // Met à jour le compteur d'oscillation
      if (angle == dernierAngleRotation && tournerDroite == (dernierAngleRotation > 0)) {
        tentativesMemeDirection++;
      } else {
        tentativesMemeDirection = 0;
      }
      
      dernierAngleRotation = angle;
      // Alterne la direction pour la prochaine fois
      tournerDroite = !tournerDroite;
      etat = AVANCE_DROIT;
    }
  }
  
  // Petite pause pour laisser le temps au système
  delay(2);
}
