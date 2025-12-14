# Chapitre 20 : Robotique et Mécatronique

La MPC est largement utilisée en robotique et mécatronique pour le contrôle précis de trajectoires, la manipulation d'objets et la navigation.

## 20.1 Robots manipulateurs

### Contrôle de trajectoire

**Problème** : Suivre une trajectoire précise dans l'espace de travail tout en respectant les limites des actionneurs.

**Modèle dynamique** :
$$
M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + G(q) = \tau
$$

où :
- $q \in \mathbb{R}^n$ : Coordonnées articulaires
- $M(q)$ : Matrice d'inertie
- $C(q, \dot{q})$ : Termes de Coriolis/centrifuge
- $G(q)$ : Gravité
- $\tau$ : Couples articulaires (commandes)

**MPC** :
$$
J = \sum_{i=1}^{N_p} \|q - q_{ref}\|^2 + \|\dot{q} - \dot{q}_{ref}\|^2 + \|\tau\|^2
$$

**Contraintes** :
- Positions : $q_{min} \leq q \leq q_{max}$ (limites articulaires)
- Vitesses : $|\dot{q}| \leq \dot{q}_{max}$
- Couples : $|\tau| \leq \tau_{max}$ (limites actionneurs)
- Obstacles : Éviter collisions avec environnement

**Défis** :
- **Non-linéarités** : Dynamique fortement non-linéaire
- **Couplages** : Mouvements couplés entre articulations
- **Singularités** : Configurations singulières à éviter
- **Temps réel** : Résolution rapide nécessaire (1-10 ms)

**Résultats** :
- Précision : ±1 mm typiquement
- Vitesse : Amélioration de 20-30%
- Lissage : Réduction vibrations

### Manipulation d'objets

**Problème** : La manipulation d'objets nécessite de :
- **Saisir** : Contrôle de force pour préhension
- **Déplacer** : Trajectoire précise et lisse
- **Placer** : Positionnement final précis
- **Gérer les contacts** : Forces de contact, friction, stabilité

**Contexte** :
- **Assemblage** : Insertion précise de composants
- **Manipulation fragile** : Objets délicats (verre, électronique)
- **Collaboration** : Interaction avec humains
- **Dynamique** : Objets en mouvement, manipulation rapide

**Variables d'état** :
- **Position effecteur** : $p_{end}(t) \in \mathbb{R}^3$
- **Orientation** : $R_{end}(t) \in SO(3)$ (matrice rotation)
- **Vitesse** : $v_{end}(t)$, $\omega_{end}(t)$
- **Force contact** : $F_{contact}(t) \in \mathbb{R}^3$

**Variables de commande** :
- **Couples articulaires** : $\tau(t)$
- **Force de préhension** : $F_{grip}(t)$

**Modèle avec contact** :

**Dynamique effecteur** :
$$
m_{end} \ddot{p}_{end} = F_{contact} + F_{gravity} + F_{manipulator}
$$

**Contraintes de contact** :

1. **Contact unilatéral** :
   $$
   F_n \geq 0, \quad d_{contact} \geq 0, \quad F_n \cdot d_{contact} = 0
   $$
   où $F_n$ est la force normale et $d_{contact}$ la distance de contact.

2. **Friction (Cône de Coulomb)** :
   $$
   \|F_t\| \leq \mu F_n
   $$
   où $F_t$ est la force tangentielle et $\mu$ le coefficient de friction.

3. **Stabilité de préhension** :
   - Centre de masse objet dans support polygon
   - Forces de contact dans cône de friction
   - Moment résultant nul

**Formulation MPC avec contraintes de contact** :

**Horizons** :
- $N_p = 20-50$ ($T_s = 0.01-0.1$ s, manipulation rapide)
- $N_c = 5-15$

**Fonction de coût** :
$$
J = \sum_{i=1}^{N_p} \left( \|p_{end} - p_{target}\|^2 + \alpha \|R_{end} - R_{target}\|^2 + \beta \|F - F_{desired}\|^2 + \gamma \|\tau\|^2 \right)
$$

où :
- $p_{target}, R_{target}$ : Position et orientation cibles
- $F_{desired}$ : Force désirée (pour saisie)
- $\alpha, \beta, \gamma$ : Pondérations

**Contraintes** :

1. **Contact** :
   - $F_n \geq 0$ : Force normale positive (contact maintenu)
   - $d_{contact} = 0$ : En contact (si manipulation)

2. **Friction** :
   - $\|F_t\| \leq \mu F_n$ : Cône de friction
   - Éviter glissement

3. **Stabilité** :
   - Centre de masse dans support polygon
   - Forces équilibrées

4. **Limites robot** :
   - Positions : $q_{min} \leq q \leq q_{max}$
   - Vitesses : $|\dot{q}| \leq \dot{q}_{max}$
   - Couples : $|\tau| \leq \tau_{max}$

5. **Obstacles** :
   - Éviter collisions avec environnement
   - Distance minimale aux obstacles

**Défis** :
- **Hybride** : Contact/no-contact (changements discrets)
- **Non-linéarités** : Friction, contacts
- **Temps réel** : Décisions rapides nécessaires
- **Robustesse** : Incertitudes sur propriétés objets

**Applications détaillées** :

1. **Assemblage** :
   - Insertion précise (ex: connecteur électronique)
   - Contrôle force pour éviter dommages
   - Compensation erreurs position

2. **Manipulation fragile** :
   - Objets délicats (verre, œufs)
   - Contrôle force très précis
   - Trajectoires lisses

3. **Collaboration humain-robot** :
   - Partage d'objets
   - Adaptation à l'humain
   - Sécurité (forces limitées)

**Résultats** :
- **Précision** : ±0.1 mm pour assemblage
- **Force** : Contrôle ±0.1 N
- **Robustesse** : Gestion incertitudes
- **Sécurité** : Forces limitées pour collaboration

## 20.2 Robots mobiles

### Navigation et évitement d'obstacles

**Problème** : Naviguer vers un objectif tout en évitant les obstacles statiques et dynamiques.

**Modèle** (robot différentiel) :
$$
\begin{cases}
\dot{x} = v \cos(\theta) \\
\dot{y} = v \sin(\theta) \\
\dot{\theta} = \omega
\end{cases}
$$

où $(x, y)$ est la position, $\theta$ l'orientation, $v$ la vitesse linéaire, $\omega$ la vitesse angulaire.

**MPC** :
$$
J = \sum_{i=1}^{N_p} \|p - p_{goal}\|^2 + \|v - v_{ref}\|^2 + \|\omega\|^2 + \sum_{obs} \text{penalty}(d_{obs})
$$

où $d_{obs}$ est la distance aux obstacles.

**Contraintes** :
- Vitesse : $0 \leq v \leq v_{max}$, $|\omega| \leq \omega_{max}$
- Obstacles : $d_{obs} \geq d_{safe}$ (distance de sécurité)
- Dynamique : Limites d'accélération

**Défis** :
- **Obstacles dynamiques** : Autres robots, personnes
- **Prédiction** : Comportement des obstacles
- **Temps réel** : Décisions rapides

**Résultats** :
- Navigation fluide
- Évitement fiable
- Trajectoires optimales

### Formation de robots

**Problème** : Coordonner plusieurs robots ($N$ robots) pour accomplir une tâche commune tout en :
- **Évitant collisions** : Distance de sécurité entre robots
- **Optimisant globalement** : Performance collective
- **Gérant la communication** : Coordination avec délais/erreurs
- **S'adaptant** : Réaction aux changements d'environnement

**Contexte** :
- **Transport collaboratif** : Déplacer objets lourds ensemble
- **Exploration** : Couvrir une zone efficacement
- **Surveillance** : Patrouille coordonnée
- **Agriculture** : Robots de récolte coordonnés

**Modèle multi-robots** :

**Dynamique robot $i$** :
$$
\begin{cases}
\dot{x}_i = v_i \cos(\theta_i) \\
\dot{y}_i = v_i \sin(\theta_i) \\
\dot{\theta}_i = \omega_i
\end{cases}
$$

**Formation désirée** :
- **Formation géométrique** : Positions relatives fixes
- **Formation dynamique** : Positions relatives variables selon tâche
- **Leader-follower** : Un robot leader, autres suivent

**Formulation MPC distribuée** :

**Problème local pour robot $i$** :

Chaque robot $i$ résout :
$$
\min_{\mathbf{u}_i} \quad J_i = \sum_{k=1}^{N_p} \left( \|p_i - p_{i,goal}\|^2 + \alpha \|p_i - p_{i,formation}\|^2 + \sum_{j \neq i} \beta \max(0, d_{safe} - d_{ij})^2 \right)
$$

où :
- $p_{i,goal}$ : Objectif individuel robot $i$
- $p_{i,formation}$ : Position dans formation désirée
- $d_{ij} = \|p_i - p_j\|$ : Distance entre robots $i$ et $j$
- $d_{safe}$ : Distance de sécurité
- $\alpha, \beta$ : Pondérations

**Contraintes** :
- Vitesse : $0 \leq v_i \leq v_{max}$, $|\omega_i| \leq \omega_{max}$
- Distance sécurité : $d_{ij} \geq d_{safe}$ pour tout $j \neq i$
- Obstacles : $d_{i,obstacle} \geq d_{safe}$

**Coordination** :

**Communication** :
- Échange positions prévues : $p_{j,k|k}$ pour $j \neq i$
- Mise à jour périodique (toutes les $T_{comm}$ secondes)
- Gestion délais communication

**Algorithme distribué** :
1. Robot $i$ reçoit positions prévues autres robots
2. Résout problème MPC local avec contraintes de collision
3. Envoie sa trajectoire prévue aux autres
4. Répète à chaque pas de temps

**Convergence** : Sous certaines conditions, les robots convergent vers la formation désirée.

**Applications détaillées** :

1. **Transport collaboratif** :
   - Plusieurs robots portent objet lourd
   - Coordination forces
   - Formation adaptative selon objet

2. **Exploration** :
   - Couvrir zone efficacement
   - Éviter chevauchement
   - Communication pour coordination

3. **Surveillance** :
   - Patrouille coordonnée
   - Couverture optimale
   - Réaction aux événements

**Défis** :
- **Communication** : Délais, pertes, erreurs
- **Scalabilité** : Performance avec nombre croissant de robots
- **Robustesse** : Pannes, défaillances
- **Temps réel** : Décisions rapides nécessaires

**Résultats** :
- **Coordination** : Formation maintenue
- **Sécurité** : Pas de collisions
- **Efficacité** : Tâche accomplie collectivement
- **Robustesse** : Adaptation aux pannes

## 20.3 Drones et UAV

### Stabilisation et contrôle d'attitude

**Problème** : Maintenir l'attitude désirée malgré perturbations (vent, turbulences).

**Modèle de drone** (quadrotor) :
$$
\begin{cases}
\ddot{x} = \frac{T}{m} (\cos\phi \sin\theta \cos\psi + \sin\phi \sin\psi) \\
\ddot{y} = \frac{T}{m} (\cos\phi \sin\theta \sin\psi - \sin\phi \cos\psi) \\
\ddot{z} = \frac{T}{m} \cos\phi \cos\theta - g \\
\ddot{\phi} = \frac{\tau_\phi}{I_x} \\
\ddot{\theta} = \frac{\tau_\theta}{I_y} \\
\ddot{\psi} = \frac{\tau_\psi}{I_z}
\end{cases}
$$

où $T$ est la poussée totale et $\tau_\phi, \tau_\theta, \tau_\psi$ sont les couples.

**MPC** :
$$
J = \sum_{i=1}^{N_p} \|p - p_{ref}\|^2 + \|\phi - \phi_{ref}\|^2 + \|T - T_{hover}\|^2
$$

**Contraintes** :
- Attitude : $|\phi|, |\theta| \leq \phi_{max}$ (éviter basculement)
- Poussée : $0 \leq T \leq T_{max}$
- Vitesse : $|v| \leq v_{max}$

**Résultats** :
- Stabilité : Amélioration significative
- Rejet perturbations : +50-70%
- Précision : ±5 cm en position

### Suivi de trajectoire

**Problème** : Suivre une trajectoire 3D précise (inspection, livraison).

**MPC** :
$$
J = \sum_{i=1}^{N_p} \|p - p_{traj}\|^2 + \|v - v_{traj}\|^2 + \|\phi - \phi_{traj}\|^2
$$

**Contraintes** :
- Trajectoire : Suivre waypoints
- Vitesse : Respecter limites
- Obstacles : Éviter collisions

**Applications** :
- Inspection d'infrastructures
- Livraison autonome
- Cinématographie aérienne

## 20.4 Prothèses et exosquelettes

### Prothèses de membre

**Problème** : Contrôler une prothèse pour reproduire les mouvements naturels.

**Défis** :
- **Interface** : Signaux EMG (électromyographie) comme entrées
- **Comfort** : Mouvements naturels, fluides
- **Stabilité** : Éviter chutes
- **Adaptation** : S'adapter à l'utilisateur

**MPC** :
$$
J = \sum_{i=1}^{N_p} \|q_{prosthesis} - q_{natural}\|^2 + \|\tau\|^2 + c_{stability} \text{stability\_margin}
$$

**Contraintes** :
- Angles : Limites articulaires
- Couples : Limites actionneurs
- Stabilité : Centre de masse dans support

**Résultats** :
- Mouvements naturels
- Réduction effort utilisateur
- Stabilité améliorée

### Exosquelettes

**Problème** : Assister les mouvements humains (rééducation, assistance).

**MPC adaptative** :
- S'adapter à l'intention de l'utilisateur
- Fournir assistance selon besoin
- Apprendre les patterns de mouvement

**Applications** :
- Rééducation
- Assistance personnes âgées
- Augmentation capacités (militaire, industrie)

## 20.5 Systèmes de suspension active

### Suspension de véhicule

**Problème** : Améliorer confort et tenue de route.

**Modèle quart de véhicule** :
$$
\begin{cases}
m_s \ddot{z}_s = -k_s(z_s - z_u) - c_s(\dot{z}_s - \dot{z}_u) + u \\
m_u \ddot{z}_u = k_s(z_s - z_u) + c_s(\dot{z}_s - \dot{z}_u) - k_t(z_u - z_r) - u
\end{cases}
$$

où :
- $z_s$ : Position caisse
- $z_u$ : Position roue
- $z_r$ : Profil route (perturbation)
- $u$ : Force suspension (commande)

**MPC** :
$$
J = \sum_{i=1}^{N_p} \|z_s - z_{ref}\|^2 + \|\dot{z}_s\|^2 + \|u\|^2
$$

**Contraintes** :
- Course suspension : $|z_s - z_u| \leq \Delta_{max}$
- Force : $|u| \leq u_{max}$
- Contact roue : $z_u \geq z_r$ (pas de décollage)

**Résultats** :
- Confort : Réduction vibrations 40-60%
- Tenue de route : Amélioration adhérence
- Compromis : Optimisation automatique

### Suspension de bâtiment

**Problème** : Réduire les vibrations dues aux séismes, vent.

**MPC** :
- Prédiction des excitations (si possible)
- Contrôle actif des amortisseurs
- Optimisation confort/stabilité

**Applications** :
- Protection sismique
- Réduction vibrations vent
- Confort occupants

---

**Points clés du chapitre** :
- Les robots manipulateurs utilisent la MPC pour précision et lissage
- Les robots mobiles appliquent la MPC pour navigation et évitement
- Les drones bénéficient de la MPC pour stabilisation et suivi de trajectoire
- Les prothèses et exosquelettes utilisent la MPC pour mouvements naturels
- Les suspensions actives appliquent la MPC pour confort et performance
- Les défis communs incluent non-linéarités, temps réel et contraintes complexes
