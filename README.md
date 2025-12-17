# Robot Éviteur d'Obstacles Ultra-Performant

## GO FAST

Ce projet consiste à réaliser un **robot autonome ultra-performant** qui détecte et évite automatiquement les obstacles grâce à un capteur ultrason HC-SR04 et des moteurs DC contrôlés par un motor shield utilisant la bibliothèque AFMotor.

Le robot utilise un système de détection avancé avec **prédiction**, **corrections progressives** et **évitement intelligent** pour naviguer de manière fluide et réactive.

## Fonctionnalités

### Détection Multi-Niveaux
Le robot utilise **3 zones de détection** pour une réaction progressive :

- **Zone de Préparation** (60-110 cm) : Détection précoce, corrections légères
- **Zone d'Alerte** (30-60 cm) : Corrections fortes, réduction de vitesse
- **Zone Critique** (<30 cm) : Arrêt immédiat, recul et rotation

### Système de Filtrage Avancé
- **4 mesures** effectuées et filtrées par médiane
- **Filtrage intelligent** : élimination des valeurs aberrantes
- **Calcul de confiance** : pondération des décisions selon la fiabilité des mesures
- **Historique de 10 mesures** pour calculer tendances et accélérations

### Prédiction et Anticipation
- **Calcul de tendance** : détection de la direction de mouvement des obstacles
- **Calcul d'accélération** : anticipation des obstacles qui se rapprochent rapidement
- **Distance prédite** : utilisation de la prédiction pour réagir plus tôt
- **Détection de chute brutale** : réaction immédiate aux obstacles fins (pieds de chaise, barreaux)

### Machine à États Intelligente
Le robot utilise **4 états** pour un contrôle précis :

1. **AVANCE_DROIT** : Navigation normale à vitesse maximale
2. **CORRECTION_LEGERE** : Préparation et ajustement progressif (60-110 cm)
3. **CORRECTION_FORTE** : Correction active avec réduction de vitesse (30-60 cm)
4. **EVITER_OBSTACLE** : Recul, arrêt et rotation pour éviter l'obstacle (<30 cm)

### Évitement Intelligent
- **Alternance de direction** : tourne alternativement à droite et à gauche
- **Angles adaptatifs** : angles de rotation ajustés selon la distance et la vitesse
- **Détection de patterns** : reconnaissance des obstacles répétitifs (barreaux, etc.)
- **Compteur d'obstacles** : angles plus grands si plusieurs obstacles consécutifs

### Contrôle de Vitesse Avancé
- **Vitesse adaptative** : ajustement automatique selon la distance
- **Rampe de vitesse** : transitions fluides (pas de changements brusques)
- **Compensation moteurs** : facteurs de correction pour équilibrer les moteurs
- **Vitesse maximale** : 180 (configurable)
- **Vitesse minimale** : 160 (même près des obstacles)

### Performance Optimisée
- **Boucle ultra-rapide** : 3ms entre chaque cycle
- **Réactions immédiates** : recul en 100ms, rotation en 2ms/deg
- **Filtrage rapide** : 4 mesures en ~6ms
- **Calculs optimisés** : historique réduit pour performance maximale

## Matériel Requis

- **Arduino Uno**
- **Motor Shield** compatible AFMotor (Adafruit Motor Shield)
- **2 moteurs DC** avec roues
- **Capteur ultrason HC-SR04**
- **Châssis de robot** avec support pour capteur et moteurs
- **Batterie** 7-12V pour alimenter le shield
- **Fils de connexion** (jumpers)

## Connexions

### Capteur Ultrason HC-SR04
- **Trig** → Pin 9 (Arduino)
- **Echo** → Pin 10 (Arduino)
- **VCC** → 5V
- **GND** → GND

### Motor Shield
- **Moteur Gauche** → Port M3
- **Moteur Droit** → Port M4
- Alimentation via connecteur batterie du shield

## Bibliothèque Nécessaire

### AFMotor
Bibliothèque pour contrôler le Motor Shield Adafruit.

**Installation :**
1. Ouvrir Arduino IDE
2. Aller dans **Croquis** → **Inclure une bibliothèque** → **Gérer les bibliothèques**
3. Rechercher "AFMotor" ou "Adafruit Motor Shield"
4. Installer la bibliothèque **Adafruit Motor Shield Library**

**Lien GitHub :** https://github.com/adafruit/Adafruit-Motor-Shield-library

## Configuration

### Paramètres Principaux

```cpp
const int VITESSE_MAX = 180;        // Vitesse maximale (0-255)
const int VITESSE_MIN = 160;        // Vitesse minimale près des obstacles
const int SEUIL_DETECTION_CM = 110; // Distance de détection précoce
const int SEUIL_ALERTE_CM = 60;     // Distance d'alerte
const int SEUIL_CRITIQUE_CM = 30;   // Distance critique
```

### Compensation Moteurs

Si un moteur va plus vite que l'autre, ajustez les facteurs :

```cpp
const float FACTEUR_COMPENSATION_GAUCHE = 1.0;  // Référence
const float FACTEUR_COMPENSATION_DROIT = 0.95;   // Ajuster selon besoin
```

- **Diminuer** le facteur si le moteur va trop vite
- **Augmenter** le facteur si le moteur va trop lentement

## Fonctionnement Détaillé

### 1. Initialisation
- Configuration des pins et moteurs
- Initialisation de l'historique de distances
- Démarrage en mode **AVANCE_DROIT**

### 2. Boucle Principale (3ms)
- **Lecture capteur** : 4 mesures filtrées (~6ms)
- **Calcul moyenne** : sur les 6 dernières mesures
- **Calcul tendance** : direction de mouvement de l'obstacle
- **Calcul accélération** : vitesse d'approche
- **Prédiction** : distance future estimée
- **Détection patterns** : reconnaissance d'obstacles répétitifs

### 3. Prise de Décision

#### Zone Libre (>110 cm)
- **État** : AVANCE_DROIT
- **Action** : Avance à vitesse maximale (180)

#### Zone de Préparation (60-110 cm)
- **État** : CORRECTION_LEGERE
- **Action** : Corrections légères, légère réduction de vitesse
- **Angle** : 0-60° selon distance

#### Zone d'Alerte (30-60 cm)
- **État** : CORRECTION_FORTE
- **Action** : Corrections fortes, réduction significative de vitesse
- **Angle** : 60-80° selon distance

#### Zone Critique (<30 cm)
- **État** : EVITER_OBSTACLE
- **Action** :
  1. **Arrêt** (15ms)
  2. **Recul** (100ms)
  3. **Arrêt** (40ms)
  4. **Rotation** (angle adaptatif, 2ms/deg)
- **Alternance** : Change de direction à chaque obstacle

### 4. Corrections Progressives
- **Calcul d'angle** : selon distance et vitesse actuelle
- **Calcul de correction** : différence de vitesse entre moteurs
- **Application rampe** : transition fluide vers nouvelle vitesse
- **Compensation** : facteurs appliqués pour équilibrer moteurs

## Optimisations Techniques

### Filtrage des Mesures
- **4 mesures** prises rapidement
- **Tri par médiane** pour éliminer valeurs aberrantes
- **Moyenne** des valeurs cohérentes (écart < 20cm)
- **Calcul de confiance** pour pondérer les décisions

### Prédiction Avancée
- **Tendance** : moyenne pondérée des variations récentes
- **Accélération** : différence entre tendances courtes et longues
- **Distance prédite** : `moyenne + tendance×3.0 + accélération×2.0`
- **Pondération** : 70% prédiction, 20% moyenne, 10% mesure actuelle

### Gestion de la Vitesse
- **Rampe** : transitions par pas de 3-4 unités
- **Vitesse adaptative** : calcul selon distance et confiance
- **Compensation** : facteurs appliqués à chaque moteur
- **Limites** : respect des VITESSE_MIN et VITESSE_MAX

## Debug et Monitoring

Le robot envoie des informations via Serial (9600 baud) :

```
Dist:85 use:82 conf:95 v:175 var:+2 -> CORRECTION LEGERE
```

- **Dist** : Distance mesurée actuelle
- **use** : Distance utilisée pour décision (prédite)
- **conf** : Confiance dans la mesure (0-100)
- **v** : Vitesse moyenne actuelle
- **var** : Variation depuis dernière mesure

## Dépannage

### Le robot ne détecte pas les obstacles
- Vérifier les connexions du capteur (Trig/Echo)
- Vérifier que les pins 9 et 10 sont libres
- Tester le capteur avec un code simple

### Le robot tourne toujours dans le même sens
- Vérifier la variable `tournerDroite` qui alterne
- Vérifier que les moteurs sont bien connectés aux bons ports

### Un moteur va plus vite que l'autre
- Ajuster `FACTEUR_COMPENSATION_GAUCHE` ou `FACTEUR_COMPENSATION_DROIT`
- Tester avec différentes valeurs (0.90 à 1.0)

### Le robot est trop lent/rapide
- Ajuster `VITESSE_MAX` et `VITESSE_MIN`
- Ajuster les seuils de détection (`SEUIL_*`)

### Le robot oscille ou fait des cercles
- Augmenter `SEUIL_DETECTION_CM` pour détecter plus tôt
- Ajuster les facteurs de compensation moteurs
- Vérifier que les roues sont bien fixées

## Structure du Code

- **`getDistance()`** : Lecture brute du capteur
- **`getDistanceFiltree()`** : Filtrage et calcul de confiance
- **`calculerTendance()`** : Calcul de la tendance de distance
- **`calculerAcceleration()`** : Calcul de l'accélération
- **`calculerDistanceUtilisee()`** : Calcul de la distance finale utilisée
- **`calculerVitesseAdaptative()`** : Calcul vitesse selon distance
- **`calculerAngleRotation()`** : Calcul angle de rotation
- **`appliquerVitesseRamp()`** : Application rampe de vitesse
- **`detecterPatternObstacle()`** : Détection patterns répétitifs
- **`detecterChuteBrutale()`** : Détection obstacles fins

## Améliorations Futures Possibles

- Ajout de capteurs latéraux pour détection multi-directionnelle
- Système de mémorisation de parcours
- Communication Bluetooth pour contrôle à distance
- Enregistrement de statistiques (distance parcourue, obstacles évités)
- Mode suivi de ligne combiné avec évitement d'obstacles

## Licence

Ce projet est libre d'utilisation pour l'apprentissage et les projets personnels.

---

**GO FAST**

