# Chapitre 19 : Énergie et Bâtiments

La MPC trouve des applications importantes dans la gestion de l'énergie, des smart grids aux bâtiments intelligents, où l'optimisation économique et la gestion des ressources sont cruciales.

## 19.1 Smart grids

Les smart grids (réseaux électriques intelligents) intègrent des sources d'énergie renouvelables variables et nécessitent une gestion optimale.

### Problème

**Défis** :
- **Variabilité** : Production renouvelable (éolien, solaire) imprévisible
- **Équilibre** : Offre = Demande à tout instant
- **Coûts variables** : Prix de l'électricité variant dans le temps
- **Stockage** : Gestion des batteries, véhicules électriques

### MPC pour gestion de microgrid

**Système** : Microgrid avec :
- Production renouvelable : $P_{ren}(t)$ (variable)
- Production conventionnelle : $P_{gen}(t)$ (contrôlable)
- Stockage : $E_{batt}(t)$ (batterie)
- Charge : $P_{load}(t)$ (prédictible partiellement)

**Variables d'état** :
- $E_{batt}(t)$ : Énergie stockée dans la batterie
- $P_{grid}(t)$ : Échange avec le réseau (achat/vente)

**MPC économique** :
$$
J = \sum_{i=1}^{N_p} c_e(t+i) P_{buy}(t+i) - c_s(t+i) P_{sell}(t+i) + c_{batt} |P_{batt}(t+i)|
$$

**Contraintes** :
- Équilibre : $P_{ren} + P_{gen} + P_{batt} + P_{grid} = P_{load}$
- Batterie : $E_{min} \leq E_{batt} \leq E_{max}$, $|P_{batt}| \leq P_{batt,max}$
- Production : $0 \leq P_{gen} \leq P_{gen,max}$

**Stratégie** :
- **Chargement** : Quand prix bas, production excédentaire
- **Déchargement** : Quand prix élevé, demande élevée
- **Optimisation** : Décaler la consommation vers heures creuses

**Résultats** :
- Réduction coûts : 20-40%
- Intégration renouvelables : +30-50%
- Stabilité réseau : Amélioration

## 19.2 Gestion de l'énergie dans les bâtiments

La gestion énergétique des bâtiments représente un enjeu majeur, représentant environ 40% de la consommation énergétique mondiale. La MPC permet d'optimiser cette consommation tout en maintenant le confort des occupants.

### Problème

**Objectifs** :
- **Économie d'énergie** : Réduire la consommation de 20-40%
- **Confort** : Maintenir température, humidité, éclairage dans des bandes acceptables
- **Coûts** : Minimiser les coûts énergétiques (tarification variable)
- **Durabilité** : Réduire l'empreinte carbone

**Variables contrôlées** :
- **Température** : $T_{zone}(t)$ (confort thermique)
- **Humidité** : $H_{zone}(t)$ (confort, santé)
- **Éclairage** : $L_{zone}(t)$ (confort visuel, économie)
- **Qualité d'air** : $CO_2(t)$ (santé, ventilation)

**Variables de commande** :
- **HVAC** : Chauffage, climatisation, ventilation
- **Éclairage** : Intensité, automatisation
- **Stores** : Ouverture/fermeture (gains solaires)
- **Équipements** : Mise en veille, gestion charge

### Modélisation du bâtiment

**Modèle thermique multi-zones** :

Pour un bâtiment avec $N_z$ zones :

$$
C_i \frac{dT_i}{dt} = Q_{HVAC,i} + Q_{solar,i} + Q_{internal,i} + \sum_{j \neq i} K_{ij}(T_j - T_i) + Q_{losses,i}(T_i, T_{ext})
$$

où :
- $C_i$ : Capacité thermique de la zone $i$
- $T_i$ : Température zone $i$
- $Q_{HVAC,i}$ : Chauffage/climatisation zone $i$ (contrôlable)
- $Q_{solar,i}$ : Gains solaires (prédictible via météo)
- $Q_{internal,i}$ : Gains internes (occupants, équipements)
- $K_{ij}$ : Coefficient d'échange entre zones $i$ et $j$
- $Q_{losses,i}$ : Pertes vers l'extérieur (conduction, ventilation)

**Modèle simplifié (zone unique)** :
$$
C \frac{dT}{dt} = Q_{HVAC} + Q_{solar} + Q_{internal} + Q_{losses}
$$

où $Q_{losses} = \frac{T - T_{ext}}{R}$ avec $R$ résistance thermique.

**Modèle d'éclairage** :
$$
L(t) = L_{natural}(t) + L_{artificial}(t)
$$

où :
- $L_{natural}$ : Éclairage naturel (dépend stores, météo)
- $L_{artificial}$ : Éclairage artificiel (commande)

### Formulation MPC économique

**Horizons** :
- $N_p = 24-48$ heures (selon objectifs, $T_s = 15-60$ min)
- $N_c = 12-24$ heures

**Fonction de coût** :
$$
J = \sum_{i=1}^{N_p} \left( c_e(t+i) P_{total}(t+i) + c_{comfort} \|T(t+i) - T_{comfort}(t+i)\|^2 + c_{light} \|L(t+i) - L_{target}\|^2 \right)
$$

où :
- $P_{total} = P_{HVAC} + P_{light} + P_{equipment}$ : Puissance totale
- $c_e(t)$ : Prix de l'électricité (variable dans le temps)
- $T_{comfort}(t)$ : Température de confort (peut varier selon occupation)
- $c_{comfort}, c_{light}$ : Pondérations confort

**Contraintes** :

1. **Confort thermique** :
   - $T_{min}(t) \leq T(t) \leq T_{max}(t)$ : Bande de confort
   - Bande peut varier selon occupation (plus large si inoccupé)
   - Typiquement : $20-26°C$ si occupé, $18-28°C$ si inoccupé

2. **Éclairage** :
   - $L_{min} \leq L(t) \leq L_{max}$ : Niveaux acceptables
   - $L_{min}$ : Minimum pour sécurité/activité
   - $L_{max}$ : Maximum (éviter éblouissement)

3. **Puissance** :
   - $0 \leq P_{HVAC} \leq P_{HVAC,max}$ : Limites système
   - $0 \leq P_{light} \leq P_{light,max}$ : Limites éclairage

4. **Qualité d'air** :
   - $CO_2(t) \leq CO_{2,max}$ : Limite santé (typiquement 1000 ppm)
   - Nécessite ventilation si dépassement

**Prédictions nécessaires** :

1. **Météo** :
   - Température extérieure : $T_{ext}(t)$
   - Ensoleillement : $I_{solar}(t)$
   - Vent : $v_{wind}(t)$
   - Sources : Services météo, modèles de prédiction

2. **Occupation** :
   - Présence occupants : $occ(t)$ (0 ou 1)
   - Nombre occupants : $N_{occ}(t)$
   - Sources : Calendriers, capteurs présence, historique

3. **Prix énergie** :
   - $c_e(t)$ : Coût de l'électricité
   - Sources : Tarification dynamique, prévisions marché

4. **Gains internes** :
   - Équipements : $Q_{equipment}(t)$
   - Occupants : $Q_{occupants}(t) = N_{occ}(t) \times q_{person}$
   - Sources : Historique, modèles

### Stratégies d'optimisation

1. **Préchauffage/Prérefroidissement** :
   - Anticiper les besoins futurs
   - Chauffage avant arrivée occupants (matin)
   - Utilisation inertie thermique

2. **Décalage temporel** :
   - Utiliser l'inertie thermique du bâtiment
   - Chauffage pendant heures creuses
   - Réduction pendant heures pleines

3. **Optimisation multi-objectif** :
   - Équilibrer confort et coût
   - Pondérations adaptatives selon contexte
   - Priorité confort si occupants présents

4. **Gestion stores** :
   - Ouvrir stores : Gains solaires hiver (chauffage gratuit)
   - Fermer stores : Réduire gains solaires été (climatisation)
   - Optimisation automatique

5. **Ventilation adaptative** :
   - Ventilation minimale si $CO_2$ bas
   - Augmentation si $CO_2$ élevé
   - Économie d'énergie (moins de chauffage/refroidissement air extérieur)

### Exemple : Bâtiment de bureaux

**Système** :
- 5 zones (bureaux, salles de réunion, couloirs, etc.)
- HVAC centralisé avec zones indépendantes
- Éclairage LED contrôlable
- Stores automatiques

**MPC** :
- Horizon : $N_p = 24$ h ($T_s = 15$ min)
- Objectif : Minimiser coût énergétique
- Contraintes : Confort $20-24°C$ si occupé, $18-26°C$ si inoccupé

**Résultats** :
- **Économie d'énergie** : 20-40%
- **Confort maintenu** : > 95% du temps dans la bande
- **Coûts réduits** : 25-35%
- **Satisfaction occupants** : Amélioration (température plus stable)

### Bénéfices additionnels

- **Réduction pic de demande** : Lissage consommation
- **Intégration renouvelables** : Utilisation production locale
- **Maintenance prédictive** : Détection anomalies
- **Conformité réglementaire** : Respect normes énergétiques

## 19.3 Systèmes HVAC (chauffage, ventilation, climatisation)

Les systèmes HVAC représentent une part importante de la consommation énergétique des bâtiments (40-60%). La MPC permet d'optimiser leur fonctionnement tout en assurant le confort et la qualité de l'air.

### Contrôle multi-zones

**Problème** : Les bâtiments modernes sont divisés en plusieurs zones (bureaux, salles de réunion, couloirs, etc.) nécessitant :
- **Contrôle indépendant** : Température adaptée à chaque zone
- **Gestion des couplages** : Échanges thermiques entre zones
- **Qualité d'air** : Ventilation adéquate, contrôle $CO_2$
- **Optimisation globale** : Minimiser consommation totale

**Contexte** :
- **Zones multiples** : 5-50 zones typiquement
- **Couplages** : Échanges via murs, portes, ventilation
- **Variabilité** : Occupation, gains solaires variables
- **Contraintes** : Confort, qualité air, puissance totale

**Modèle multi-zones détaillé** :

**Bilan thermique par zone** :
$$
C_z \frac{dT_z}{dt} = Q_{z,HVAC} + Q_{z,solar} + Q_{z,internal} + \sum_{j \neq z} K_{zj}(T_j - T_z) + K_{z,ext}(T_{ext} - T_z) + Q_{z,vent}
$$

où :
- $C_z$ : Capacité thermique zone $z$
- $Q_{z,HVAC}$ : Chauffage/climatisation zone $z$ (commande)
- $Q_{z,solar}$ : Gains solaires (prédictible)
- $Q_{z,internal}$ : Gains internes (occupants, équipements)
- $K_{zj}$ : Coefficient échange zone $z$ ↔ zone $j$
- $K_{z,ext}$ : Coefficient échange zone $z$ ↔ extérieur
- $Q_{z,vent}$ : Échange via ventilation

**Forme matricielle** :
$$
\mathbf{C} \dot{\mathbf{T}} = \mathbf{Q}_{HVAC} + \mathbf{K}_{coupling} \mathbf{T} + \mathbf{Q}_{disturbance}
$$

où :
- $\mathbf{T} = [T_1, ..., T_{N_z}]^T$ : Températures zones
- $\mathbf{Q}_{HVAC} = [Q_{1,HVAC}, ..., Q_{N_z,HVAC}]^T$ : Commandes
- $\mathbf{K}_{coupling}$ : Matrice d'échange (couplages)
- $\mathbf{Q}_{disturbance}$ : Perturbations (solaire, interne, extérieur)

**Modèle qualité d'air** :

**Concentration $CO_2$** :
$$
V_z \frac{dCO_{2,z}}{dt} = Q_{vent,z} (CO_{2,ext} - CO_{2,z}) + G_{CO_2,z}
$$

où :
- $V_z$ : Volume zone $z$
- $Q_{vent,z}$ : Débit ventilation (commande)
- $G_{CO_2,z}$ : Génération $CO_2$ (occupants)

**Formulation MPC** :

**Horizons** :
- $N_p = 12-24$ heures ($T_s = 15-30$ min)
- $N_c = 6-12$ heures

**Fonction de coût** :
$$
J = \sum_{i=1}^{N_p} \sum_{z=1}^{N_z} \left( \|T_{z,k+i|k} - T_{z,ref,k+i}\|^2 + c_{energy} P_{z,HVAC,k+i} + c_{CO_2} \max(0, CO_{2,z} - CO_{2,max})^2 \right) + \lambda \left\| \sum_z P_{z,HVAC} \right\|^2
$$

où :
- $T_{z,ref}$ : Température référence zone $z$ (peut varier selon occupation)
- $P_{z,HVAC}$ : Puissance HVAC zone $z$
- $c_{energy}$ : Coût énergie
- $c_{CO_2}$ : Pénalité $CO_2$ élevé
- $\lambda$ : Pondération puissance totale (lissage)

**Contraintes** :

1. **Température par zone** :
   - $T_{z,min}(t) \leq T_z \leq T_{z,max}(t)$ : Bande de confort
   - Bande peut varier selon occupation
   - Typiquement : $20-24°C$ si occupé, $18-26°C$ si inoccupé

2. **Qualité d'air** :
   - $CO_{2,z} \leq CO_{2,max}$ : Limite santé (typiquement 1000 ppm)
   - Nécessite ventilation si dépassement
   - Contrainte souple possible (pénalisation)

3. **Puissance** :
   - $0 \leq P_{z,HVAC} \leq P_{z,max}$ : Limites par zone
   - $\sum_z P_{z,HVAC} \leq P_{total,max}$ : Limite globale (contrainte réseau)

4. **Ventilation** :
   - $Q_{vent,min} \leq Q_{vent,z} \leq Q_{vent,max}$ : Débits admissibles
   - Minimum pour qualité air même si $CO_2$ bas

**Défis spécifiques** :

1. **Couplages** :
   - Zones adjacentes s'influencent
   - Optimisation globale nécessaire
   - Pas d'optimisation zone par zone

2. **Variabilité occupation** :
   - Zones occupées/inoccupées varient
   - Ajustement consignes nécessaire
   - Prédiction occupation importante

3. **Gains solaires** :
   - Variables selon orientation, météo
   - Peuvent aider (hiver) ou gêner (été)
   - Prédiction nécessaire

4. **Contrainte puissance totale** :
   - Limite réseau électrique
   - Priorisation zones si nécessaire
   - Gestion pics de demande

**Stratégies d'optimisation** :

1. **Priorisation zones** :
   - Zones occupées : Priorité confort
   - Zones inoccupées : Économie d'énergie
   - Ajustement dynamique

2. **Exploitation couplages** :
   - Utiliser zones chaudes pour chauffer zones froides
   - Réduire besoins HVAC globaux

3. **Ventilation adaptative** :
   - Ventilation minimale si $CO_2$ bas
   - Augmentation si $CO_2$ élevé
   - Économie d'énergie (moins chauffage/refroidissement air)

4. **Anticipation** :
   - Préchauffage zones avant occupation
   - Utilisation inertie thermique
   - Réduction consommation

**Avantages de la MPC** :
- **Gestion couplages** : Optimisation globale vs contrôle local
- **Anticipation** : Prédiction besoins futurs
- **Multi-objectif** : Équilibre confort/économie/qualité air
- **Adaptation** : Ajustement selon occupation, météo

**Résultats obtenus** :
- **Économie** : 15-30% (selon configuration)
- **Confort** : Amélioration uniformité entre zones
- **Qualité air** : Respect normes $CO_2$ garanti
- **Stabilité** : Réduction variations température
- **Satisfaction** : Amélioration confort perçu

## 19.4 Énergies renouvelables

### 19.4.1 Éoliennes

**Problème** : Maximiser la production tout en respectant les contraintes mécaniques.

**Variables** :
- **Puissance** : $P(t)$ (à maximiser)
- **Vitesse rotor** : $\omega(t)$ (contrôlable via pitch)
- **Couple** : $\tau(t)$ (contrainte mécanique)

**Modèle** :
$$
P = \frac{1}{2} \rho A v^3 C_p(\lambda, \beta)
$$

où :
- $\rho$ : Densité air
- $A$ : Surface balayée
- $v$ : Vitesse vent (perturbation)
- $C_p$ : Coefficient de puissance (dépend de $\lambda = \omega R/v$ et $\beta$ = angle pitch)

**MPC** :
$$
J = \sum_{i=1}^{N_p} -P(t+i) + \|\beta - \beta_{nom}\|^2 + \|\tau - \tau_{max}\|^2
$$

**Contraintes** :
- Vitesse rotor : $\omega_{min} \leq \omega \leq \omega_{max}$
- Pitch : $\beta_{min} \leq \beta \leq \beta_{max}$
- Couple : $|\tau| \leq \tau_{max}$ (sécurité mécanique)
- Puissance : $P \leq P_{rated}$ (limite nominale)

**Stratégies** :
- **Région 2** (vent faible) : Maximiser $C_p$
- **Région 3** (vent nominal) : Maintenir puissance nominale
- **Région 4** (vent fort) : Réduire charge (pitch)

**Résultats** :
- Production : +2-5%
- Réduction charges : 20-30%
- Durée de vie : Amélioration

### 19.4.2 Panneaux solaires

**Problème** : Maximiser la production, gérer l'ombrage, optimiser l'orientation.

**Variables** :
- **Puissance** : $P(t) = \eta I(t) A \cos(\theta(t))$
- **Orientation** : $\theta(t)$ (tracking)
- **Température** : $T_{panel}(t)$ (affecte efficacité)

**MPC** :
$$
J = \sum_{i=1}^{N_p} -P(t+i) + c_{tracking} \|\dot{\theta}\|^2
$$

**Contraintes** :
- Orientation : $|\theta| \leq \theta_{max}$
- Vitesse tracking : $|\dot{\theta}| \leq \dot{\theta}_{max}$
- Température : $T \leq T_{max}$ (refroidissement si nécessaire)

**Défis** :
- **Ombrage** : Partiel, variable
- **Nuages** : Variations rapides
- **Prédiction** : Ensoleillement difficile à prédire

**Applications** :
- Tracking optimal
- Gestion de l'ombrage
- Optimisation avec stockage

## 19.5 Stockage d'énergie

### Gestion de batteries

**Problème** : Optimiser le cycle de charge/décharge pour maximiser durée de vie et efficacité.

**Modèle de batterie** :
$$
\begin{cases}
\dot{E} = \eta_{charge} P_{charge} - \frac{1}{\eta_{discharge}} P_{discharge} \\
\dot{SOH} = f(E, P, T) \quad \text{(State of Health)}
\end{cases}
$$

où $SOH$ représente la santé de la batterie (dégradation).

**MPC** :
$$
J = \sum_{i=1}^{N_p} c_e(t+i) P_{grid}(t+i) + c_{degradation} \Delta SOH + c_{safety} \|E - E_{nominal}\|^2
$$

**Contraintes** :
- Énergie : $E_{min} \leq E \leq E_{max}$ (20-80% typiquement pour durée de vie)
- Puissance : $|P| \leq P_{max}$
- Température : $T_{min} \leq T \leq T_{max}$ (sécurité, durée de vie)

**Stratégies** :
- **Éviter décharges profondes** : Limiter à 20%
- **Éviter surcharges** : Limiter à 80%
- **Gestion thermique** : Contrôle de température
- **Optimisation économique** : Charge/décharge selon prix

**Résultats** :
- Durée de vie : +20-40%
- Efficacité : Optimisation cycles
- Coûts : Réduction via optimisation économique

### Systèmes hybrides (batterie + supercondensateur)

**Idée** : Combiner batterie (énergie) et supercondensateur (puissance).

**MPC** :
- **Batterie** : Gestion énergie à long terme
- **Supercondensateur** : Gestion puissance à court terme

**Avantages** :
- Réduction dégradation batterie
- Meilleure réponse dynamique
- Optimisation globale

---

**Points clés du chapitre** :
- Les smart grids utilisent la MPC pour équilibrer offre/demande et optimiser les coûts
- Les bâtiments bénéficient de la MPC pour économie d'énergie et confort
- Les systèmes HVAC utilisent la MPC pour contrôle multi-zones optimal
- Les énergies renouvelables appliquent la MPC pour maximiser production et protéger équipements
- Le stockage d'énergie utilise la MPC pour optimiser cycles et durée de vie
- Les gains typiques sont de 20-40% en économie et efficacité
