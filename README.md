# Robot Éviteur d'Obstacles

## Description

Robot autonome qui détecte et évite automatiquement les obstacles grâce à un capteur ultrason HC-SR04 et des moteurs DC contrôlés par un motor shield utilisant la bibliothèque AFMotor.

Le robot avance en ligne droite et, lorsqu'un obstacle est détecté, il recule, tourne et reprend sa trajectoire.

## Fonctionnalités

- **Détection multi-niveaux** : 3 zones de détection (110cm, 60cm, 30cm)
- **Corrections progressives** : ajustements de trajectoire selon la distance
- **Évitement intelligent** : recul et rotation adaptative
- **Vitesse adaptative** : ajustement automatique selon la distance
- **Détection d'angles** : gestion spéciale pour les angles resserrés

## Matériel Requis

- Arduino Uno
- Motor Shield compatible AFMotor
- 2 moteurs DC avec roues
- Capteur ultrason HC-SR04
- Châssis de robot
- Batterie 7-12V
- Fils de connexion

## Connexions

### Capteur Ultrason HC-SR04
- **Trig** → Pin 12
- **Echo** → Pin 13
- **VCC** → 5V
- **GND** → GND

### Motor Shield
- **Moteur Gauche** → Port M3
- **Moteur Droit** → Port M4

## Bibliothèque

**AFMotor** (Adafruit Motor Shield Library)

Installation via Arduino IDE : Croquis → Inclure une bibliothèque → Gérer les bibliothèques → Rechercher "AFMotor"

Lien : https://github.com/adafruit/Adafruit-Motor-Shield-library

## Configuration

```cpp
const int VITESSE_MAX = 180;
const int VITESSE_MIN = 165;
const int SEUIL_DETECTION = 110;  // cm
const int SEUIL_ALERTE = 60;       // cm
const int SEUIL_CRITIQUE = 30;     // cm
const float FACTEUR_DROIT = 0.98;  // Compensation moteur droit
```

## Dépannage

**Le robot ne détecte pas les obstacles**
- Vérifier les connexions Trig/Echo (pins 12 et 13)

**Un moteur va plus vite que l'autre**
- Ajuster `FACTEUR_DROIT` (0.90 à 1.0)

**Le robot est trop lent/rapide**
- Ajuster `VITESSE_MAX` et `VITESSE_MIN`

**Le robot oscille**
- Augmenter `SEUIL_DETECTION` pour détecter plus tôt
- Ajuster les facteurs de compensation
