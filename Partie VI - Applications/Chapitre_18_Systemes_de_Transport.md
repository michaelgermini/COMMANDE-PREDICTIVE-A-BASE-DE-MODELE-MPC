# Chapitre 18 : Systèmes de Transport

La MPC trouve des applications croissantes dans les systèmes de transport, des véhicules autonomes à l'aéronautique, où la planification de trajectoire et le contrôle précis sont essentiels.

## 18.1 Véhicules autonomes

Les véhicules autonomes représentent l'une des applications les plus prometteuses de la MPC, combinant planification de trajectoire et contrôle en temps réel.

### 18.1.1 Planification de trajectoire

**Problème** : Générer une trajectoire sûre et confortable vers un objectif tout en évitant les obstacles.

**Modèle du véhicule** :

**Modèle cinématique (bicycle model)** :
$$
\begin{cases}
\dot{x} = v \cos(\theta) \\
\dot{y} = v \sin(\theta) \\
\dot{\theta} = \frac{v}{L} \tan(\delta) \\
\dot{v} = a
\end{cases}
$$

où :
- $(x, y)$ : Position
- $\theta$ : Orientation
- $v$ : Vitesse
- $\delta$ : Angle de braquage
- $a$ : Accélération
- $L$ : Empattement

**MPC pour planification** :

**Objectif** :
$$
J = \sum_{i=1}^{N_p} \|p_{k+i|k} - p_{target}\|^2 + \|v - v_{ref}\|^2 + \|\delta\|^2 + \|a\|^2
$$

**Contraintes** :
- **Obstacles** : Distance minimale aux obstacles
- **Limites physiques** : $|\delta| \leq \delta_{max}$, $|a| \leq a_{max}$, $v \geq 0$
- **Confort** : $|\dot{\delta}| \leq \dot{\delta}_{max}$, $|\dot{a}| \leq \dot{a}_{max}$
- **Routage** : Rester sur la route, respecter les voies

**Horizon** : $N_p = 50-100$ (5-10 secondes à 10 Hz)

**Défis** :
- **Obstacles dynamiques** : Autres véhicules en mouvement
- **Prédiction des autres véhicules** : Modèles de comportement
- **Temps réel** : Résolution en < 100 ms

### 18.1.2 Contrôle latéral et longitudinal

**Contrôle latéral** : Suivi de trajectoire (lane keeping)

**Modèle linéarisé** :
$$
\begin{bmatrix}
\dot{e} \\ \dot{\psi} \\ \dot{r}
\end{bmatrix}
=
\begin{bmatrix}
0 & v & 0 \\
0 & 0 & 1 \\
0 & 0 & a_{22}
\end{bmatrix}
\begin{bmatrix}
e \\ \psi \\ r
\end{bmatrix}
+
\begin{bmatrix}
0 \\ 0 \\ b_2
\end{bmatrix}
\delta
$$

où :
- $e$ : Erreur latérale
- $\psi$ : Erreur d'orientation
- $r$ : Vitesse de lacet

**MPC** :
$$
J = \sum_{i=1}^{N_p} \|e_{k+i|k}\|^2 + \|\psi_{k+i|k}\|^2 + \|\delta_{k+i|k}\|^2
$$

**Contrôle longitudinal** : Gestion de la vitesse et distance de sécurité

**MPC adaptative** :
- **Suivi de vitesse** : Si pas de véhicule devant
- **Suivi de distance** : Si véhicule détecté (ACC - Adaptive Cruise Control)

**Formulation ACC** :
$$
J = \sum_{i=1}^{N_p} \|d - d_{safe}\|^2 + \|v - v_{ref}\|^2 + \|a\|^2
$$

où $d$ est la distance au véhicule précédent et $d_{safe} = v \times t_{headway}$.

**Résultats** :
- Précision de suivi : ±10 cm latéralement
- Confort : Amélioration significative vs PID
- Sécurité : Respect garanti des contraintes

## 18.2 Aéronautique et spatial

### 18.2.1 Pilotage automatique

**Problème** : Maintenir l'altitude, la vitesse, l'assiette et suivre une trajectoire.

**Modèle d'avion** (simplifié) :
$$
\begin{cases}
\dot{V} = \frac{T - D}{m} - g \sin(\gamma) \\
\dot{\gamma} = \frac{L - mg \cos(\gamma)}{mV} \\
\dot{h} = V \sin(\gamma) \\
\dot{x} = V \cos(\gamma)
\end{cases}
$$

où :
- $V$ : Vitesse
- $\gamma$ : Angle de montée
- $h$ : Altitude
- $T, D, L$ : Poussée, traînée, portance

**MPC** :
$$
J = \sum_{i=1}^{N_p} \|h - h_{ref}\|^2 + \|V - V_{ref}\|^2 + \|T\|^2
$$

**Contraintes** :
- Altitude : $h_{min} \leq h \leq h_{max}$
- Vitesse : $V_{stall} \leq V \leq V_{max}$
- Poussée : $0 \leq T \leq T_{max}$
- Assiette : $|\theta| \leq \theta_{max}$

**Applications** :
- **Approche et atterrissage** : Trajectoire précise
- **Navigation** : Suivi de waypoints
- **Gestion de carburant** : Optimisation économique

### 18.2.2 Atterrissage de fusées

**Problème** : Atterrir une fusée de manière précise et sûre (ex: SpaceX Falcon 9).

**Défis** :
- **Dynamiques rapides** : Réaction en temps réel nécessaire
- **Contraintes strictes** : Position, vitesse, attitude
- **Non-linéarités** : Aérodynamique, poussée variable
- **Perturbations** : Vent, variations de masse

**Modèle** :
$$
\begin{cases}
\ddot{r} = \frac{T}{m} u - g + \frac{F_{aero}}{m} \\
\dot{m} = -\frac{T}{I_{sp} g_0}
\end{cases}
$$

où :
- $r$ : Position
- $T$ : Poussée
- $u$ : Direction de poussée (vecteur unitaire)
- $m$ : Masse (variable)
- $F_{aero}$ : Forces aérodynamiques

**MPC** :
$$
J = \sum_{i=1}^{N_p} \|r - r_{target}\|^2 + \|v\|^2 + \|\theta - \theta_{vertical}\|^2
$$

**Contraintes** :
- Position : $r_{target} \pm \epsilon$ (précision requise)
- Vitesse : $v \approx 0$ à l'atterrissage
- Attitude : $\theta \approx 0$ (vertical)
- Poussée : $T_{min} \leq T \leq T_{max}$

**Résultats** :
- Précision : Atterrissage à ±1 m
- Fiabilité : > 95% de succès
- Efficacité : Optimisation du carburant

## 18.3 Systèmes ferroviaires

Les systèmes ferroviaires utilisent la MPC pour optimiser la consommation d'énergie, respecter les horaires et améliorer le confort des passagers, tout en respectant les contraintes de sécurité et de signalisation.

### Contrôle de train

**Problème** : Optimiser la vitesse d'un train pour :
- Respecter les horaires (ponctualité)
- Minimiser la consommation d'énergie
- Assurer le confort des passagers
- Respecter les limitations de vitesse et la signalisation

**Contexte** :
- **Trains de voyageurs** : Confort et ponctualité prioritaires
- **Trains de fret** : Économie d'énergie importante
- **Métros** : Fréquence élevée, arrêts fréquents
- **Trains à grande vitesse** : Optimisation complexe avec profils de vitesse variables

**Modèle dynamique** :

Le modèle de base d'un train s'écrit :
$$
\begin{cases}
\dot{s} = v \\
\dot{v} = \frac{F_{traction} - F_{resistance} - F_{brake}}{m}
\end{cases}
$$

où :
- $s$ : Position le long de la voie
- $v$ : Vitesse
- $F_{traction}$ : Force de traction (commande)
- $F_{resistance}$ : Forces de résistance
- $F_{brake}$ : Force de freinage (commande)
- $m$ : Masse du train

**Forces de résistance** :
$$
F_{resistance} = a + b v + c v^2 + F_{grade}(s)
$$

où :
- $a$ : Résistance de roulement (constante)
- $b v$ : Résistance proportionnelle à la vitesse
- $c v^2$ : Résistance aérodynamique (quadratique)
- $F_{grade}(s)$ : Force due au profil de la voie (pentes, courbes)

**Modèle énergétique** :
$$
P_{traction} = F_{traction} \cdot v / \eta_{motor}
$$

où $\eta_{motor}$ est le rendement du moteur.

**Formulation MPC économique** :

**Horizons** :
- $N_p = 100-500$ (selon longueur du trajet, $T_s = 1-10$ s)
- $N_c = 20-50$

**Fonction de coût** :
$$
J = \sum_{i=1}^{N_p} \left( c_{energy}(s_{k+i}) P_{traction,k+i} - c_{regen} P_{regen,k+i} \right) + \alpha \|t_{k+i} - t_{schedule,k+i}\|^2 + \beta \|a_{k+i}\|^2
$$

où :
- $c_{energy}(s)$ : Coût de l'énergie (peut varier selon position)
- $P_{regen}$ : Puissance récupérée par freinage récupératif
- $c_{regen}$ : Valeur de l'énergie récupérée
- $t$ : Temps écoulé depuis le départ
- $\alpha, \beta$ : Pondérations

**Contraintes** :

1. **Vitesse** :
   - $0 \leq v \leq v_{max}(s)$ : Limite selon position (courbes, zones urbaines)
   - $v(s_{stop}) = 0$ : Arrêt aux stations

2. **Accélération** :
   - $|a| \leq a_{max}$ : Confort des passagers (typiquement $|a| \leq 1-1.5$ m/s²)
   - $a_{min} \leq a \leq a_{max}$ : Limites mécaniques

3. **Horaire** :
   - $t(s_{station}) = t_{schedule}(s_{station}) \pm \Delta t$ : Arriver à l'heure (±30 s typiquement)

4. **Signalisation** :
   - Respect des signaux (rouge = arrêt, jaune = ralentir)
   - Distance de sécurité avec train précédent

5. **Énergie** :
   - $0 \leq P_{traction} \leq P_{max}$ : Puissance maximale
   - $0 \leq P_{brake} \leq P_{brake,max}$ : Freinage maximal

**Stratégies d'optimisation** :

1. **Coasting** : Rouler sans traction (économie d'énergie)
   - Utilisé en descente ou avant ralentissement
   - Économie significative d'énergie

2. **Freinage récupératif** : Récupérer l'énergie au freinage
   - Conversion énergie cinétique → électrique
   - Réduction consommation de 10-20%

3. **Anticipation** : Ralentir progressivement avant limitations
   - Évite freinages brusques
   - Améliore confort et économie

4. **Optimisation du profil** : Choisir vitesse optimale selon profil de voie
   - Accélération en montée modérée
   - Récupération en descente

**Implémentation** :

**Prédictions nécessaires** :
- **Profil de voie** : Pentes, courbes (connu a priori)
- **Limitations vitesse** : Selon position (connu)
- **Signalisation** : État des signaux (mesuré en temps réel)
- **Charge** : Masse du train (estimée)

**MPC adaptative** :
- Ajustement selon retard/avance
- Adaptation à la charge réelle
- Réaction aux changements de signalisation

**Résultats obtenus** :
- **Économie d'énergie** : 15-25% (selon profil de ligne)
- **Ponctualité** : Amélioration de 5-10%
- **Confort** : Réduction des à-coups de 30-40%
- **Usure matériel** : Réduction (freinages plus doux)

### Gestion de trafic

**Problème** : Coordonner plusieurs trains sur un réseau pour optimiser le flux global.

**MPC distribuée** :
- Chaque train résout son propre problème MPC
- Coordination pour éviter collisions
- Optimisation globale du réseau

**Objectifs** :
- Minimiser temps de trajet total
- Maximiser débit
- Respecter contraintes de sécurité

**Défis** :
- Communication entre trains
- Prédiction comportement autres trains
- Gestion des conflits (aiguillages, voies uniques)

### Gestion de trafic

**Problème** : Coordonner plusieurs trains sur un réseau pour optimiser le flux.

**MPC distribuée** :
- Chaque train résout son propre problème MPC
- Coordination pour éviter collisions
- Optimisation globale du réseau

## 18.4 Navigation maritime

La navigation maritime moderne utilise la MPC pour optimiser les trajectoires, économiser le carburant et assurer la sécurité, particulièrement importante pour les navires autonomes et l'optimisation des routes commerciales.

### Contrôle de navires

**Problème** : Naviguer un navire de manière :
- **Précise** : Suivre une route définie
- **Économique** : Minimiser consommation de carburant
- **Sûre** : Éviter obstacles, respecter limitations
- **Confortable** : Réduire mouvements (roulis, tangage)

**Contexte** :
- **Navires de commerce** : Optimisation économique cruciale
- **Navires autonomes** : Navigation sans intervention humaine
- **Navires de recherche** : Précision de positionnement
- **Yachts** : Confort et économie

**Modèle de navire** :

Le modèle cinématique et dynamique d'un navire s'écrit :

**Cinématique** (transformation repère) :
$$
\begin{cases}
\dot{x} = u \cos(\psi) - v \sin(\psi) \\
\dot{y} = u \sin(\psi) + v \cos(\psi) \\
\dot{\psi} = r
\end{cases}
$$

**Dynamique** (forces et moments) :
$$
\begin{cases}
\dot{u} = \frac{F_u}{m} \\
\dot{v} = \frac{F_v}{m} \\
\dot{r} = \frac{M}{I}
\end{cases}
$$

où :
- $(x, y)$ : Position dans repère fixe
- $\psi$ : Cap (orientation)
- $(u, v)$ : Vitesses dans repère du navire (surge, sway)
- $r$ : Vitesse de rotation (yaw rate)
- $F_u, F_v$ : Forces longitudinale et latérale
- $M$ : Moment de rotation
- $m, I$ : Masse et moment d'inertie

**Forces** :
$$
\begin{cases}
F_u = F_{prop} - F_{drag}(u) - F_{wave}(t) \\
F_v = F_{rudder}(\delta) - F_{drag}(v) \\
M = M_{rudder}(\delta) + M_{wave}(t)
\end{cases}
$$

où :
- $F_{prop}$ : Poussée des hélices (commande)
- $\delta$ : Angle de barre (commande)
- $F_{wave}, M_{wave}$ : Perturbations (vagues, vent, courants)

**Formulation MPC** :

**Horizons** :
- $N_p = 50-200$ (selon vitesse, $T_s = 1-10$ s)
- $N_c = 10-50$

**Fonction de coût** :
$$
J = \sum_{i=1}^{N_p} \left( \|p_{k+i|k} - p_{waypoint,k+i}\|^2 + c_{fuel} P_{prop,k+i} + \alpha \|\delta_{k+i|k}\|^2 + \beta \|\phi_{k+i|k}\|^2 \right)
$$

où :
- $p$ : Position $(x, y)$
- $p_{waypoint}$ : Waypoints à suivre
- $P_{prop}$ : Puissance propulsion
- $c_{fuel}$ : Coût du carburant
- $\delta$ : Angle de barre
- $\phi$ : Gîte (inclinaison latérale)
- $\alpha, \beta$ : Pondérations

**Contraintes** :

1. **Route** :
   - Suivre waypoints : $p(s_{waypoint}) = p_{waypoint} \pm \epsilon$
   - Distance minimale aux obstacles : $d_{obstacle} \geq d_{safe}$

2. **Vitesse** :
   - $0 \leq v \leq v_{max}$ : Limite mécanique
   - $v_{min} \leq v$ : Vitesse minimale (manœuvrabilité)

3. **Gîte** :
   - $|\phi| \leq \phi_{max}$ : Sécurité (typiquement $|\phi| \leq 15-20°$)
   - Réduction gîte améliore confort et sécurité

4. **Commandes** :
   - $0 \leq P_{prop} \leq P_{max}$ : Puissance maximale
   - $|\delta| \leq \delta_{max}$ : Angle barre maximal (typiquement $±35°$)
   - $|\dot{\delta}| \leq \dot{\delta}_{max}$ : Vitesse de rotation barre

5. **Zone de navigation** :
   - Rester dans les eaux navigables
   - Respecter zones interdites
   - Suivre couloirs de navigation

**Défis spécifiques** :

1. **Perturbations** :
   - **Vagues** : Forces périodiques complexes
   - **Courants** : Déplacement d'eau
   - **Vent** : Force aérodynamique
   - **Prédiction** : Modèles météo-oceanographiques

2. **Dynamiques lentes** :
   - **Inertie importante** : Réponse lente aux commandes
   - **Horizon long nécessaire** : Anticipation importante
   - **Stabilité** : Navire stable mais réactif

3. **Non-linéarités** :
   - **Aérodynamique** : Forces non-linéaires
   - **Hydrodynamique** : Résistance variable avec vitesse
   - **Couplages** : Interactions entre mouvements

4. **Environnement** :
   - **Obstacles dynamiques** : Autres navires
   - **Conditions météo** : Variables et imprévisibles
   - **Réglementation** : Règles de navigation (COLREG)

**Stratégies d'optimisation** :

1. **Routage optimal** :
   - Éviter zones de mauvais temps
   - Profiter des courants favorables
   - Minimiser distance et temps

2. **Économie de carburant** :
   - Vitesse optimale selon conditions
   - Réduction gîte (moins de résistance)
   - Utilisation courants

3. **Sécurité** :
   - Anticipation obstacles
   - Respect distances de sécurité
   - Manœuvres d'évitement

**Applications** :

1. **Navires autonomes** :
   - Navigation sans pilote
   - Décisions automatiques
   - Sécurité renforcée

2. **Optimisation de route** :
   - Éviter mauvais temps
   - Profiter courants favorables
   - Réduction temps et coûts

3. **Économie de carburant** :
   - Routage optimal
   - Gestion vitesse
   - Réduction consommation de 10-20%

4. **Précision de positionnement** :
   - Navigation précise
   - Suivi de route exact
   - Applications scientifiques, militaires

**Résultats** :
- **Économie carburant** : 10-20%
- **Précision navigation** : ±10-50 m
- **Réduction gîte** : 30-40%
- **Sécurité** : Amélioration évitement obstacles

---

**Points clés du chapitre** :
- Les véhicules autonomes utilisent la MPC pour planification et contrôle
- L'aéronautique applique la MPC pour pilotage et atterrissage
- Les systèmes ferroviaires optimisent vitesse et consommation
- La navigation maritime bénéficie de la MPC pour précision et économie
- Les défis communs incluent temps réel, obstacles dynamiques et non-linéarités
- Les résultats montrent des améliorations significatives en précision, sécurité et efficacité
