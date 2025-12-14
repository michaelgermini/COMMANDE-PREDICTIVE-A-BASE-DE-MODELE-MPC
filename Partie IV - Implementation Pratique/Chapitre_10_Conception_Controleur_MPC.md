# Chapitre 10 : Conception d'un Contrôleur MPC

Ce chapitre présente une méthodologie systématique pour concevoir et mettre en œuvre un contrôleur MPC, de la définition des objectifs à la validation finale.

## 10.1 Étapes de conception

### 10.1.1 Définition des objectifs

Avant de commencer, il est crucial de définir clairement les **objectifs** du contrôleur.

**Questions à se poser** :
1. **Quelle est la variable à contrôler ?** (température, position, vitesse, etc.)
2. **Quel est l'objectif principal ?** (suivi de référence, régulation, optimisation économique)
3. **Quelles sont les contraintes critiques ?** (sécurité, limites physiques)
4. **Quelles sont les performances attendues ?** (temps de réponse, précision, robustesse)
5. **Quel est le contexte opérationnel ?** (perturbations, variations de charge)

**Exemple** : Pour un four industriel :
- **Objectif** : Maintenir la température à une consigne avec précision ±2°C
- **Contraintes** : Température max 800°C, puissance max 50kW
- **Performances** : Temps de réponse < 5 min, rejet de perturbations
- **Contexte** : Variations de charge, ouverture/fermeture de portes

### 10.1.2 Choix du modèle

Le choix du modèle dépend de :
- **Complexité du système** : Linéaire ou non-linéaire ?
- **Données disponibles** : Modèle physique ou identification ?
- **Précision requise** : Modèle simple ou détaillé ?
- **Temps de calcul disponible** : Modèle rapide ou précis ?

**Types de modèles** :

1. **Modèle physique** :
   - Basé sur les lois physiques (conservation, équations différentielles)
   - Avantages : Compréhension, extrapolation
   - Inconvénients : Paramètres à identifier, simplifications

2. **Modèle identifié** :
   - À partir de données expérimentales
   - Avantages : Précision, adapté au système réel
   - Inconvénients : Besoin de données, validité limitée

3. **Modèle hybride** :
   - Structure physique + paramètres identifiés
   - Compromis optimal

**Recommandation** : Commencer simple, complexifier si nécessaire.

### 10.1.3 Sélection des paramètres de réglage

Les paramètres principaux à régler sont :
- **Horizons** : $N_p$, $N_c$
- **Pondérations** : $Q$, $R$, $S$
- **Contraintes** : Limites, variables de relâchement
- **Période d'échantillonnage** : $T_s$

**Stratégie** : Réglage itératif basé sur la simulation et les tests.

## 10.2 Réglage des paramètres

### 10.2.1 Horizon de prédiction ($N_p$)

**Règles empiriques** :

1. **Couverture du temps de réponse** :
   $$
   N_p \geq \frac{T_{95}}{T_s}
   $$
   où $T_{95}$ est le temps de réponse à 95%.

2. **Systèmes stables** : $N_p = 10$ à $50$ pas typiquement

3. **Systèmes instables** : $N_p$ plus grand, ou utilisation de contraintes terminales

4. **Temps de calcul** : Limité par le temps disponible

**Impact de $N_p$** :
- **Trop petit** : Instabilité possible, performances dégradées
- **Trop grand** : Charge computationnelle, peu d'amélioration

**Méthode de réglage** :
1. Commencer avec $N_p = 20$
2. Augmenter si instabilité ou performances insuffisantes
3. Diminuer si temps de calcul trop élevé

### 10.2.2 Horizon de commande ($N_c$)

**Règles** :

1. **Typiquement** : $N_c = 3$ à $10$ pas

2. **Relation avec $N_p$** : $N_c \leq N_p$, souvent $N_c \approx N_p/3$ à $N_p/2$

3. **Systèmes rapides** : $N_c$ plus petit (1-3)

4. **Systèmes lents** : $N_c$ plus grand (5-10)

**Impact de $N_c$** :
- **Trop petit** : Manque de flexibilité, performances dégradées
- **Trop grand** : Charge computationnelle, peu d'amélioration

**Stratégie** : Commencer avec $N_c = N_p/2$, ajuster selon les résultats.

### 10.2.3 Matrices de pondération ($Q$, $R$, $S$)

#### Matrice $Q$ (pondération sur les sorties)

**Règle de base** : $Q$ grande = priorité au suivi de référence

**Choix initial** :
- **Diagonale** : $Q = \text{diag}(q_1, ..., q_p)$
- **Normalisation** : $q_i = 1 / y_{i,\max}^2$ pour normaliser les erreurs
- **Ajustement** : Multiplier par un facteur selon l'importance relative

**Exemple** : Pour température et pression :
$$
Q = \begin{bmatrix} 100 & 0 \\ 0 & 1 \end{bmatrix}
$$
si la température est 10 fois plus importante que la pression.

#### Matrice $R$ (pondération sur les commandes)

**Règle de base** : $R$ grande = économie d'effort, commandes plus petites

**Choix initial** :
- **Diagonale** : $R = \text{diag}(r_1, ..., r_m)$
- **Normalisation** : $r_i = \rho / u_{i,\max}^2$
- **Facteur $\rho$** : Commencer avec $\rho = 0.1$ à $1$

**Ajustement** :
- Augmenter $R$ si les commandes sont trop agressives
- Diminuer $R$ si le suivi est insuffisant

#### Matrice $S$ (pondération sur les variations)

**Règle de base** : $S$ grande = commandes plus lisses

**Choix initial** :
- **Diagonale** : $S = \text{diag}(s_1, ..., s_m)$
- **Relation avec $R$** : $S = \alpha R$ où $\alpha = 0.1$ à $1$

**Ajustement** :
- Augmenter $S$ si les commandes oscillent
- Diminuer $S$ si la réactivité est insuffisante

#### Méthode de réglage systématique

**Étape 1** : Normaliser les matrices
$$
Q = \text{diag}(1/y_{i,\max}^2), \quad R = \rho \text{diag}(1/u_{i,\max}^2)
$$

**Étape 2** : Ajuster $\rho$ pour équilibrer suivi vs effort
- Commencer avec $\rho = 1$
- Augmenter si commandes trop grandes
- Diminuer si suivi insuffisant

**Étape 3** : Ajuster les ratios entre variables
- Multiplier les lignes de $Q$ selon l'importance relative
- Ajuster $S$ pour le lissage

**Étape 4** : Validation par simulation

### Exemple de réglage

Pour un réservoir avec contrôle de niveau :

- **Sortie** : Niveau $h$ (0 à 2 m)
- **Commande** : Débit $u$ (0 à 1 m³/s)
- **Objectif** : Suivi de référence avec lissage

**Choix initial** :
$$
Q = \frac{1}{2^2} = 0.25, \quad R = \frac{0.1}{1^2} = 0.1, \quad S = 0.1 \times R = 0.01
$$

**Ajustements** :
- Si oscillations : Augmenter $S$ à $0.05$
- Si réactivité insuffisante : Diminuer $R$ à $0.05$
- Si suivi insuffisant : Augmenter $Q$ à $1$

## 10.3 Validation et simulation

La validation par simulation est une étape cruciale avant le déploiement. Elle permet de vérifier que le contrôleur MPC répond aux spécifications et se comporte correctement dans diverses situations.

### Importance de la validation

**Objectifs** :
- **Vérifier les performances** : Respect des spécifications (temps de réponse, précision)
- **Détecter les problèmes** : Instabilité, violations de contraintes, comportements indésirables
- **Optimiser le réglage** : Ajuster les paramètres pour améliorer les performances
- **Réduire les risques** : Éviter les problèmes en production

**Méthodologie** :
1. **Tests systématiques** : Scénarios variés et représentatifs
2. **Analyse quantitative** : Métriques de performance
3. **Analyse qualitative** : Comportement, lissage, stabilité
4. **Comparaison** : Avec contrôleur existant ou spécifications

### Tests de simulation

Avant l'implémentation réelle, valider le contrôleur par **simulation** sur le modèle de prédiction et sur des modèles plus détaillés.

**Scénarios de test essentiels** :

#### 1. Réponse indicielle

**Test** : Appliquer un échelon de référence $r(t) = r_0$ pour $t \geq t_0$

**Objectifs** :
- Vérifier le temps de réponse
- Évaluer le dépassement
- Mesurer l'erreur statique
- Observer le comportement transitoire

**Métriques à vérifier** :
- **Temps de montée** ($t_r$) : Temps pour atteindre 90% de la valeur finale
- **Temps de réponse** ($t_s$) : Temps pour entrer dans une bande de ±5%
- **Dépassement** ($M_p$) : Pourcentage de dépassement maximal
- **Erreur statique** : $e_{ss} = \lim_{t \to \infty} (y(t) - r_0)$

**Critères de réussite** :
- $t_r \leq t_{r,spec}$
- $M_p \leq M_{p,spec}$ (typiquement < 5-10%)
- $e_{ss} \approx 0$ (ou dans tolérance)

#### 2. Réponse à une perturbation

**Test** : Appliquer une perturbation en échelon $d(t) = d_0$ pour $t \geq t_0$ avec référence constante

**Objectifs** :
- Évaluer le rejet de perturbation
- Mesurer le temps de retour à l'équilibre
- Vérifier la stabilité

**Métriques** :
- **Amplitude maximale d'écart** : $\max |y(t) - r|$ après perturbation
- **Temps de retour** : Temps pour revenir dans bande de ±5%
- **Intégrale de l'erreur** : $\int |y(t) - r| dt$ (mesure globale)

**Critères** :
- Retour rapide à l'équilibre
- Écart maximal acceptable
- Pas d'oscillations persistantes

#### 3. Suivi de trajectoire

**Test** : Référence variable $r(t)$ (rampe, sinusoïde, trajectoire complexe)

**Objectifs** :
- Évaluer la capacité de suivi
- Vérifier le respect des contraintes pendant le suivi
- Observer le comportement dynamique

**Types de trajectoires** :
- **Rampe** : $r(t) = at + b$ (test vitesse de suivi)
- **Sinusoïde** : $r(t) = A \sin(\omega t)$ (test réponse fréquentielle)
- **Trajectoire complexe** : Trajectoire réaliste (ex: véhicule)

**Métriques** :
- **Erreur de suivi** : $\|y(t) - r(t)\|$
- **Décalage temporel** : Retard de phase
- **Respect contraintes** : Vérifier que contraintes toujours satisfaites

#### 4. Conditions limites

**Test** : États initiaux extrêmes, références aux limites

**Objectifs** :
- Vérifier la faisabilité
- Tester la robustesse
- Évaluer le domaine d'attraction

**Scénarios** :
- **État initial loin de l'équilibre** : $x_0$ proche des limites
- **Référence aux limites** : $r$ proche des contraintes
- **Combinaison** : État initial + référence extrêmes

**Vérifications** :
- Problème faisable
- Convergence vers équilibre
- Pas de violation de contraintes critiques

#### 5. Tests de robustesse (préliminaires)

**Test** : Erreurs de modèle, perturbations

**Objectifs** :
- Évaluer la sensibilité
- Identifier les points faibles
- Préparer les tests de robustesse détaillés (section 10.4)

**Scénarios** :
- **Gain modifié** : $K = 0.8K_{nom}$ à $1.2K_{nom}$
- **Constante de temps modifiée** : $\tau = 0.8\tau_{nom}$ à $1.2\tau_{nom}$
- **Bruit de mesure** : Ajouter bruit blanc

### Métriques de performance

#### Métriques temporelles

**Temps de montée** ($t_r$) :
$$
t_r = \min \{ t : |y(t) - y_{final}| \leq 0.1 |y_{final} - y_0| \}
$$
Temps pour atteindre 90% de la valeur finale.

**Temps de réponse** ($t_s$) :
$$
t_s = \min \{ t : |y(\tau) - y_{final}| \leq 0.05 |y_{final} - y_0|, \forall \tau \geq t \}
$$
Temps pour entrer et rester dans une bande de ±5%.

**Dépassement** ($M_p$) :
$$
M_p = \frac{y_{max} - y_{final}}{y_{final} - y_0} \times 100\%
$$
Pourcentage de dépassement maximal.

#### Métriques d'erreur

**Erreur quadratique moyenne** (RMSE) :
$$
RMSE = \sqrt{\frac{1}{N}\sum_{k=1}^{N} (y_k - r_k)^2}
$$
Mesure globale de l'erreur.

**Erreur absolue maximale** (MAE) :
$$
MAE = \max_{k} |y_k - r_k|
$$
Pire cas d'erreur.

**Erreur absolue moyenne** :
$$
MAE_{avg} = \frac{1}{N}\sum_{k=1}^{N} |y_k - r_k|
$$
Erreur moyenne.

**Erreur statique** :
$$
e_{ss} = \lim_{k \to \infty} (y_k - r_k)
$$
Erreur en régime permanent (devrait être nulle pour systèmes avec intégrateur).

#### Métriques de commande

**Effort de commande** :
$$
E_{effort} = \sum_{k=1}^{N} |u_k| \quad \text{ou} \quad \sum_{k=1}^{N} u_k^2
$$
Mesure de l'énergie dépensée.

**Variations de commande** :
$$
E_{variation} = \sum_{k=1}^{N} |\Delta u_k|
$$
Mesure de la "douceur" de la commande.

**Saturation** :
- **Nombre de saturations** : Nombre de fois où $u_k$ atteint $u_{min}$ ou $u_{max}$
- **Temps de saturation** : Pourcentage du temps en saturation
- **Critique** : Éviter saturations prolongées

**Taux d'utilisation** :
$$
U_{rate} = \frac{1}{N} \sum_{k=1}^{N} \frac{|u_k|}{u_{max}} \times 100\%
$$
Pourcentage moyen d'utilisation de la commande.

### Validation sur modèle détaillé

Après validation sur le modèle de prédiction, il est essentiel de tester sur un **modèle plus détaillé** qui capture mieux la réalité.

**Types de modèles détaillés** :

1. **Modèle non-linéaire complet** :
   - Inclut toutes les non-linéarités
   - Teste la validité de la linéarisation
   - Identifie les limitations

2. **Modèle avec perturbations réalistes** :
   - Bruit de mesure
   - Perturbations externes
   - Variations de paramètres

3. **Modèle avec retards** :
   - Retards de mesure
   - Retards d'actionnement
   - Retards de communication

4. **Modèle avec saturations** :
   - Saturation réelle des actionneurs
   - Dynamiques de saturation
   - Anti-windup

**Procédure** :
1. **Simulation sur modèle détaillé** : Utiliser le contrôleur MPC conçu
2. **Comparaison** : Performances vs modèle de prédiction
3. **Ajustement** : Modifier réglages si nécessaire
4. **Validation finale** : Vérifier que performances acceptables

**Critères de validation** :
- Performances similaires ou meilleures que prévu
- Stabilité maintenue
- Respect des contraintes
- Comportement acceptable

## 10.4 Tests de robustesse

Les tests de robustesse sont essentiels pour garantir que le contrôleur MPC fonctionne correctement malgré les incertitudes sur le modèle et les perturbations. Ces tests doivent être effectués avant tout déploiement en conditions réelles.

### Objectifs des tests de robustesse

**Vérifications** :
- **Stabilité robuste** : Le système reste stable malgré les incertitudes
- **Performances robustes** : Les performances restent acceptables
- **Faisabilité robuste** : Le problème reste faisable
- **Respect des contraintes** : Les contraintes critiques sont toujours respectées

**Importance** :
- Les modèles sont toujours approximatifs
- Les paramètres peuvent varier dans le temps
- Les perturbations sont inévitables
- La robustesse est cruciale pour la fiabilité

### Tests de robustesse paramétrique

Les paramètres du modèle ne sont jamais connus avec une précision parfaite. Il faut tester la robustesse aux variations de ces paramètres.

**Méthodologie** :

1. **Identification des paramètres incertains** :
   - Paramètres mal connus
   - Paramètres variant dans le temps
   - Paramètres sensibles

2. **Définition des plages d'incertitude** :
   - Plages réalistes basées sur expérience/identification
   - Typiquement : ±10% à ±30% selon paramètre

3. **Tests systématiques** :
   - Varier chaque paramètre dans sa plage
   - Tester combinaisons de paramètres
   - Utiliser grille de paramètres ou échantillonnage

**Exemple détaillé** : Système avec gain $K$ incertain

**Paramètre** : Gain $K$ avec valeur nominale $K_{nom}$

**Plage d'incertitude** : $K \in [0.8K_{nom}, 1.2K_{nom}]$ (±20%)

**Tests** :
- $K = 0.8K_{nom}$ : Gain faible (système moins sensible)
- $K = K_{nom}$ : Gain nominal (référence)
- $K = 1.2K_{nom}$ : Gain élevé (système plus sensible)

**Vérifications pour chaque valeur** :
- Stabilité : Pas d'oscillations, convergence
- Performances : Temps de réponse, précision
- Contraintes : Respect des limites
- Faisabilité : Problème toujours faisable

**Critères de réussite** :
- Stabilité maintenue pour toutes les valeurs
- Performances dégradées mais acceptables
- Pas de violation de contraintes critiques

**Méthodes avancées** :
- **Analyse Monte Carlo** : Échantillonnage aléatoire des paramètres
- **Analyse de sensibilité** : Dérivées partielles des performances
- **Robustesse structurée** : Incertitudes structurées (matrices)

### Tests avec perturbations

Les systèmes réels sont soumis à diverses perturbations. Il faut tester la capacité du contrôleur à les gérer.

#### Types de perturbations

**1. Perturbations constantes** :
- **Offset** : $d(t) = d_0$ (constant)
- **Biais** : Erreur de mesure constante
- **Charge constante** : Charge fixe sur le système

**Tests** :
- Appliquer perturbation constante
- Vérifier rejet (erreur statique nulle si intégrateur)
- Mesurer temps de retour à l'équilibre

**2. Perturbations variables** :
- **Bruit blanc** : $d(t) \sim \mathcal{N}(0, \sigma^2)$
- **Bruit coloré** : Bruit filtré (plus réaliste)
- **Variations lentes** : $d(t) = d_0 \sin(\omega t)$ (perturbation périodique)

**Tests** :
- Appliquer bruit avec différentes variances
- Vérifier filtrage efficace
- Mesurer dégradation des performances

**3. Perturbations impulsives** :
- **Chocs** : $d(t) = d_0 \delta(t - t_0)$ (impulsion)
- **Changements brusques** : Échelon de perturbation
- **Pannes** : Perturbations importantes et soudaines

**Tests** :
- Appliquer choc à différents instants
- Vérifier récupération rapide
- Mesurer amplitude maximale d'écart

#### Critères d'évaluation

**Stabilité** :
- Pas d'oscillations divergentes
- Convergence vers équilibre
- Pas d'instabilité même avec perturbations importantes

**Performances** :
- Performances dégradées mais acceptables
- Temps de réponse peut augmenter mais reste raisonnable
- Précision peut diminuer mais reste dans spécifications

**Contraintes** :
- Contraintes critiques toujours respectées (sécurité)
- Contraintes non-critiques peuvent être temporairement violées (avec pénalisation)

### Tests de faisabilité

La faisabilité du problème d'optimisation MPC doit être garantie dans diverses conditions opérationnelles.

**Scénarios de test** :

#### 1. États initiaux variés

**Test** : Initialiser le système à différents états $x_0$

**Objectifs** :
- Vérifier faisabilité pour différents $x_0$
- Identifier le domaine de faisabilité
- Tester limites du domaine

**Procédure** :
- États proches de l'équilibre
- États aux limites des contraintes
- États loin de l'équilibre
- États aux coins du domaine admissible

**Vérifications** :
- Problème faisable pour tous les $x_0$ testés
- Convergence vers équilibre
- Pas de violation de contraintes

#### 2. Perturbations importantes

**Test** : Appliquer perturbations importantes qui poussent le système vers les limites

**Objectifs** :
- Vérifier faisabilité même sous perturbations
- Tester capacité de récupération
- Identifier limites de robustesse

**Scénarios** :
- Perturbation poussant vers limite supérieure
- Perturbation poussant vers limite inférieure
- Perturbations combinées

#### 3. Changements de référence

**Test** : Changer la référence de manière importante

**Objectifs** :
- Vérifier faisabilité pour nouvelles références
- Tester capacité de suivi
- Identifier références non-atteignables

**Types de changements** :
- Échelon important
- Changement rapide
- Référence aux limites

### Analyse de sensibilité

L'analyse de sensibilité étudie comment les performances dépendent des paramètres de réglage, permettant d'identifier les paramètres critiques et d'optimiser le réglage.

#### Impact des horizons

**Horizon de prédiction ($N_p$)** :

**Tests** : Varier $N_p$ de $N_{p,min}$ à $N_{p,max}$

**Métriques** :
- Stabilité : $N_p$ trop petit → instabilité possible
- Performances : $N_p$ optimal → meilleures performances
- Temps de calcul : $N_p$ grand → calcul plus long

**Analyse** :
- Identifier $N_p$ minimal pour stabilité
- Identifier $N_p$ optimal pour performances
- Compromis stabilité/performances/temps

**Horizon de commande ($N_c$)** :

**Tests** : Varier $N_c$ de 1 à $N_p$

**Métriques** :
- Flexibilité : $N_c$ petit → moins de flexibilité
- Performances : $N_c$ optimal → meilleures performances
- Temps de calcul : $N_c$ grand → calcul plus long

**Analyse** :
- Identifier $N_c$ minimal acceptable
- Identifier $N_c$ optimal
- Impact sur performances vs complexité

#### Impact des pondérations

**Matrice $Q$ (pondération sorties)** :

**Tests** : Varier $Q$ de $Q_{min}$ à $Q_{max}$ (multiples de valeur nominale)

**Impact** :
- $Q$ grand : Priorité au suivi, commandes plus agressives
- $Q$ petit : Moins de priorité au suivi, commandes plus douces

**Analyse** :
- Sensibilité des performances au suivi
- Compromis suivi vs effort
- Identification valeur optimale

**Matrice $R$ (pondération commandes)** :

**Tests** : Varier $R$ de $R_{min}$ à $R_{max}$

**Impact** :
- $R$ grand : Économie d'effort, commandes plus petites
- $R$ petit : Moins de pénalisation, commandes plus grandes

**Analyse** :
- Sensibilité de l'effort de commande
- Impact sur performances
- Identification valeur optimale

**Matrice $S$ (pondération variations)** :

**Tests** : Varier $S$ de $S_{min}$ à $S_{max}$

**Impact** :
- $S$ grand : Commandes très lisses, moins réactif
- $S$ petit : Commandes plus réactives, moins lisses

**Analyse** :
- Compromis réactivité vs lissage
- Impact sur usure équipements
- Identification valeur optimale

#### Identification des paramètres critiques

**Méthode** : Analyse de sensibilité systématique

**Procédure** :
1. **Variation unidimensionnelle** : Varier un paramètre à la fois
2. **Mesure d'impact** : Quantifier impact sur performances
3. **Classement** : Identifier paramètres les plus critiques
4. **Optimisation** : Ajuster paramètres critiques en priorité

**Paramètres typiquement critiques** :
- $N_p$ : Impact majeur sur stabilité
- $Q$ : Impact majeur sur suivi
- $R$ : Impact majeur sur effort

**Paramètres moins critiques** :
- $S$ : Impact modéré (lissage)
- $N_c$ : Impact modéré (si $N_c \geq 3$)

#### Outils d'analyse

**Simulation Monte Carlo** :
- Échantillonnage aléatoire des paramètres
- Statistiques sur performances
- Identification distributions

**Analyse de variance** :
- Décomposition variance performances
- Contribution de chaque paramètre
- Identification paramètres dominants

**Optimisation automatique** :
- Optimisation des paramètres par essai-erreur
- Algorithmes d'optimisation (genetic algorithms, etc.)
- Recherche de l'optimum global

---

**Points clés du chapitre** :
- La conception MPC nécessite une méthodologie structurée
- La définition claire des objectifs est fondamentale
- Le choix du modèle est un compromis précision/complexité
- Le réglage des paramètres est itératif et basé sur la simulation
- La validation et les tests de robustesse sont essentiels avant déploiement
