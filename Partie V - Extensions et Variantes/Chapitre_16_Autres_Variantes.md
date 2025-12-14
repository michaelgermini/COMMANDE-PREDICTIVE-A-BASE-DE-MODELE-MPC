# Chapitre 16 : Autres Variantes

Ce chapitre présente d'autres variantes importantes de la MPC qui répondent à des besoins spécifiques.

## 16.1 MPC hybride (systèmes à événements discrets)

Les systèmes hybrides combinent dynamique continue et événements discrets, nécessitant des approches de contrôle spécialisées. La MPC hybride étend la MPC classique pour gérer ces systèmes complexes.

### Motivation

Les **systèmes hybrides** sont omniprésents dans l'industrie et nécessitent une modélisation et un contrôle adaptés :

**Caractéristiques** :
- **Dynamique continue** : Évolution temporelle continue des variables d'état
- **Événements discrets** : Changements discrets (modes, commutations, logique)
- **Interaction** : Les événements discrets affectent la dynamique continue

**Exemples concrets** :

1. **Circuits avec interrupteurs** :
   - États ON/OFF des interrupteurs
   - Dynamique continue des courants/tensions
   - Commutations contrôlées ou automatiques

2. **Systèmes avec modes** :
   - **Mode normal** : Fonctionnement standard
   - **Mode dégradé** : Fonctionnement réduit (panne partielle)
   - **Mode urgence** : Fonctionnement de secours
   - Transitions entre modes selon conditions

3. **Systèmes avec logique** :
   - Conditions if-then-else dans la dynamique
   - Seuils déclenchant changements
   - Automates finis combinés avec dynamique continue

4. **Systèmes manufacturiers** :
   - Machines ON/OFF
   - Changements de configuration
   - Séquences de production

### Modélisation

**Système hybride général** :

$$
\begin{cases}
x_{k+1} = f_{\sigma_k}(x_k, u_k) & \text{si } x_k \in \mathcal{R}_{\sigma_k} \\
\sigma_{k+1} = \delta(\sigma_k, x_k, u_k) & \text{changement de mode}
\end{cases}
$$

où :
- $\sigma_k \in \{1, ..., M\}$ : Mode discret à l'instant $k$
- $f_{\sigma_k}$ : Dynamique continue dans le mode $\sigma_k$
- $\mathcal{R}_{\sigma_k}$ : Région d'activation du mode $\sigma_k$
- $\delta$ : Fonction de transition entre modes

**Exemple simple** : Système avec deux modes

**Mode 1** (normal) :
$$
x_{k+1} = A_1 x_k + B_1 u_k
$$

**Mode 2** (dégradé) :
$$
x_{k+1} = A_2 x_k + B_2 u_k
$$

**Transition** :
- Mode 1 → Mode 2 : Si $x_k \in \mathcal{R}_{fault}$ (détection panne)
- Mode 2 → Mode 1 : Si $x_k \in \mathcal{R}_{recovery}$ (récupération)

### Formulation MPC hybride

**Problème d'optimisation** :

$$
\begin{aligned}
\min_{\mathbf{u}, \boldsymbol{\sigma}} \quad & \sum_{i=0}^{N_p-1} \ell_{\sigma_{k+i|k}}(x_{k+i|k}, u_{k+i|k}) \\
\text{s.t.} \quad & x_{k+i+1|k} = f_{\sigma_{k+i|k}}(x_{k+i|k}, u_{k+i|k}), \quad i = 0, ..., N_p-1 \\
& x_{k+i|k} \in \mathcal{R}_{\sigma_{k+i|k}}, \quad i = 0, ..., N_p \\
& \sigma_{k+i+1|k} = \delta(\sigma_{k+i|k}, x_{k+i|k}, u_{k+i|k}), \quad i = 0, ..., N_p-1 \\
& x_{k|k} = x_k, \quad \sigma_{k|k} = \sigma_k
\end{aligned}
$$

où :
- $\mathbf{u} = [u_{k|k}, ..., u_{k+N_c-1|k}]$ : Séquences de commandes continues
- $\boldsymbol{\sigma} = [\sigma_{k|k}, ..., \sigma_{k+N_p|k}]$ : Séquences de modes discrets

**Défis spécifiques** :

1. **Variables mixtes** :
   - Continues : $u_{k+i|k} \in \mathbb{R}^m$
   - Discrètes : $\sigma_{k+i|k} \in \{1, ..., M\}$
   - Problème **MILP** (Mixed Integer Linear Programming) ou **MIQP** (Mixed Integer Quadratic Programming)

2. **Complexité computationnelle** :
   - Nombre de séquences de modes : $M^{N_p}$ (exponentiel)
   - Pour $M=3$, $N_p=10$ : $3^{10} = 59,049$ séquences possibles
   - Résolution exacte souvent impossible en temps réel

3. **Contraintes logiques** :
   - Contraintes sur transitions entre modes
   - Contraintes sur séquences de modes
   - Contraintes sur compatibilité mode/état

### Méthodes de résolution

#### 1. Enumération complète

**Principe** : Tester toutes les séquences de modes possibles

**Algorithme** :
1. Générer toutes les séquences $\boldsymbol{\sigma}$ possibles
2. Pour chaque séquence, résoudre MPC avec modes fixes
3. Choisir la meilleure solution

**Avantages** : Solution optimale garantie

**Inconvénients** : 
- Complexité exponentielle
- Applicable seulement pour petits systèmes ($M \leq 3$, $N_p \leq 5$)

#### 2. Branch-and-bound

**Principe** : Exploration arborescente intelligente avec élagage

**Algorithme** :
1. Construire arbre de séquences de modes
2. Évaluer bornes inférieures/supérieures
3. Élaguer branches non-prometteuses
4. Explorer branches restantes

**Avantages** :
- Réduction significative de l'espace de recherche
- Solution optimale (si exploration complète)

**Inconvénients** :
- Complexité toujours élevée
- Pas de garantie temps de calcul

#### 3. Relaxation et arrondi

**Principe** : Relaxer variables discrètes, arrondir solution

**Algorithme** :
1. Relaxer $\sigma_k \in \{1, ..., M\}$ en $\sigma_k \in [1, M]$ (continu)
2. Résoudre problème continu
3. Arrondir $\sigma_k$ à valeurs entières
4. Résoudre MPC avec modes arrondis

**Avantages** :
- Complexité polynomiale
- Rapide

**Inconvénients** :
- Solution sous-optimale
- Peut violer contraintes logiques

#### 4. Heuristiques

**Principe** : Règles pour choisir les modes

**Exemples** :
- **Mode actuel** : Maintenir mode actuel si possible
- **Mode optimal local** : Choisir mode optimal pour état actuel
- **Séquence prédéfinie** : Utiliser séquence de modes connue

**Avantages** :
- Très rapide
- Simple à implémenter

**Inconvénients** :
- Pas d'optimalité garantie
- Peut manquer meilleures solutions

### Applications détaillées

#### Contrôle de circuits de puissance

**Système** : Convertisseur DC-DC avec interrupteurs

**Modes** :
- Mode 1 : Interrupteur ON
- Mode 2 : Interrupteur OFF
- Mode 3 : Diode conductrice

**MPC hybride** :
- Optimise séquence de commutation
- Contrôle tension/courant
- Minimise pertes de commutation

**Résultats** :
- Efficacité améliorée
- Réduction pertes de commutation
- Contrôle précis

#### Gestion de systèmes multi-modes

**Système** : Système avec modes normal, dégradé, urgence

**MPC hybride** :
- Détection automatique de mode
- Transition optimale entre modes
- Gestion de la dégradation

**Bénéfices** :
- Continuité de service
- Optimisation selon mode
- Robustesse aux pannes

## 16.2 MPC multi-objectif

Dans de nombreuses applications, plusieurs objectifs doivent être optimisés simultanément, souvent en conflit les uns avec les autres. La MPC multi-objectif permet de gérer ces compromis de manière systématique.

### Motivation

Les systèmes réels ont généralement plusieurs objectifs à satisfaire :

**Nature des objectifs** :
- **Conflictuels** : Améliorer un objectif dégrade l'autre
- **Indépendants** : Objectifs sans interaction
- **Complémentaires** : Objectifs qui se renforcent mutuellement

**Exemples typiques** :

1. **Confort vs Énergie** :
   - Confort maximal : Température proche de consigne
   - Consommation minimale : Réduire chauffage/climatisation
   - **Conflit** : Plus de confort → plus d'énergie

2. **Performance vs Robustesse** :
   - Performances élevées : Réactivité, précision
   - Marge de robustesse : Stabilité, sécurité
   - **Conflit** : Performances élevées → moins de marge

3. **Coût vs Qualité** :
   - Coût minimal : Réduire ressources
   - Qualité maximale : Utiliser plus de ressources
   - **Conflit** : Qualité élevée → coût élevé

4. **Précision vs Temps de calcul** :
   - Précision maximale : Modèle détaillé, horizon long
   - Temps de calcul minimal : Modèle simple, horizon court
   - **Conflit** : Précision → temps de calcul

### Formulation mathématique

**Problème multi-objectif standard** :

$$
\min_{\mathbf{u}} \quad \mathbf{J}(\mathbf{u}) = [J_1(\mathbf{u}), J_2(\mathbf{u}), ..., J_p(\mathbf{u})]^T
$$

où :
- $J_i(\mathbf{u})$ : $i$-ème objectif
- $p$ : Nombre d'objectifs
- $\mathbf{J}$ : Vecteur d'objectifs

**Problème fondamental** : 
- **Pas de solution unique** en général
- **Trade-off** : Compromis entre objectifs
- **Front de Pareto** : Ensemble de solutions optimales au sens de Pareto

**Définition de dominance** :

Solution $\mathbf{u}_1$ **domine** $\mathbf{u}_2$ si :
- $J_i(\mathbf{u}_1) \leq J_i(\mathbf{u}_2)$ pour tout $i$
- $J_j(\mathbf{u}_1) < J_j(\mathbf{u}_2)$ pour au moins un $j$

**Solution Pareto-optimale** : Solution non-dominée (pas de meilleure solution sur tous les objectifs).

### Approches de résolution

#### 1. Pondération (Weighted Sum)

**Principe** : Combiner les objectifs en une somme pondérée

**Formulation** :
$$
\min_{\mathbf{u}} \quad J_{weighted} = \sum_{i=1}^p w_i J_i(\mathbf{u})
$$

où $w_i \geq 0$ sont les poids avec $\sum_i w_i = 1$ (normalisation).

**Avantages** :
- **Simplicité** : Réduit à un problème standard
- **Efficacité** : Résolution rapide
- **Intuitif** : Poids reflètent importance relative

**Inconvénients** :
- **Choix des poids** : Arbitraire, difficile
- **Solutions manquées** : Peut manquer certaines solutions Pareto (régions concaves)
- **Sensibilité** : Petites variations de poids → grandes variations solution

**Méthode de réglage** :
1. **Normalisation** : Normaliser chaque $J_i$ pour comparabilité
2. **Pondération initiale** : $w_i = 1/p$ (égal)
3. **Ajustement** : Augmenter $w_i$ si objectif $i$ insuffisant
4. **Validation** : Vérifier compromis acceptable

**Exemple** : $J_1$ (coût) et $J_2$ (qualité)
- $w_1 = 0.7$, $w_2 = 0.3$ : Priorité coût
- $w_1 = 0.3$, $w_2 = 0.7$ : Priorité qualité
- $w_1 = 0.5$, $w_2 = 0.5$ : Équilibre

#### 2. $\epsilon$-contrainte

**Principe** : Optimiser un objectif principal sous contraintes sur les autres

**Formulation** :
$$
\begin{aligned}
\min_{\mathbf{u}} \quad & J_1(\mathbf{u}) \\
\text{s.t.} \quad & J_i(\mathbf{u}) \leq \epsilon_i, \quad i = 2, ..., p \\
& \text{Contraintes MPC standard}
\end{aligned}
$$

où $\epsilon_i$ sont les niveaux acceptables pour objectifs secondaires.

**Avantages** :
- **Interprétation claire** : Niveaux de performance garantis
- **Flexibilité** : Ajuster $\epsilon_i$ selon besoins
- **Garanties** : Performance minimale sur objectifs secondaires

**Inconvénients** :
- **Choix des $\epsilon_i$** : Délicat, peut rendre problème infaisable
- **Sous-optimalité** : Peut ne pas exploiter pleinement compromis
- **Sensibilité** : $\epsilon_i$ trop restrictifs → infaisabilité

**Méthode de réglage** :
1. **Évaluer performances nominales** : $J_i^*$ sans contraintes
2. **Définir $\epsilon_i$** : $\epsilon_i = (1 + \alpha) J_i^*$ où $\alpha$ est marge (ex: 10-20%)
3. **Ajustement** : Relâcher $\epsilon_i$ si infaisable, resserrer si performances insuffisantes

#### 3. Pareto optimalité

**Principe** : Trouver l'ensemble de solutions Pareto-optimales

**Définition formelle** : 

$\mathbf{u}^*$ est **Pareto-optimal** si :
$$
\nexists \mathbf{u} \text{ tel que } J_i(\mathbf{u}) \leq J_i(\mathbf{u}^*) \text{ pour tout } i \text{ et } J_j(\mathbf{u}) < J_j(\mathbf{u}^*) \text{ pour au moins un } j
$$

**Front de Pareto** : Ensemble de toutes les solutions Pareto-optimales dans l'espace des objectifs.

**Méthodes de génération** :

1. **Génération complète** :
   - Résoudre pour nombreuses combinaisons de poids
   - Coûteux mais complet
   - Applicable pour $p = 2-3$ objectifs

2. **Approximation** :
   - Générer quelques points représentatifs
   - Interpolation pour front complet
   - Compromis précision/temps

3. **Méthodes évolutionnaires** :
   - **NSGA-II** (Non-dominated Sorting Genetic Algorithm)
   - **MOEA/D** (Multi-Objective Evolutionary Algorithm)
   - Génération efficace du front

**Avantages** :
- **Vision complète** : Tous les compromis possibles
- **Décision éclairée** : Choix parmi solutions optimales
- **Pas de perte d'information** : Toutes solutions Pareto préservées

**Inconvénients** :
- **Coût computationnel** : Élevé pour $p > 2$
- **Complexité décision** : Choix parmi nombreuses solutions
- **Temps réel** : Souvent trop coûteux

#### 4. MPC hiérarchique d'objectifs

**Principe** : Organiser les objectifs par niveaux de priorité et résoudre séquentiellement

**Structure hiérarchique** :

**Niveau 1 (Critique)** :
- Sécurité, contraintes dures
- **Résolution** : Minimiser violations
- **Contraintes** : Doivent être respectées

**Niveau 2 (Important)** :
- Performance, qualité
- **Résolution** : Optimiser sous contraintes niveau 1
- **Contraintes** : Peuvent être relâchées si nécessaire

**Niveau 3 (Souhaitable)** :
- Confort, économie secondaire
- **Résolution** : Optimiser sous contraintes niveaux 1-2
- **Contraintes** : Peuvent être largement violées

**Algorithme** :

```
1. Résoudre niveau 1 : min violations contraintes critiques
2. Si faisable, résoudre niveau 2 : min J_2 sous contraintes niveau 1
3. Si faisable, résoudre niveau 3 : min J_3 sous contraintes niveaux 1-2
```

**Avantages** :
- **Priorisation claire** : Objectifs critiques garantis
- **Robustesse** : Faisabilité niveau 1 prioritaire
- **Flexibilité** : Ajustement selon contexte

**Inconvénients** :
- **Sous-optimalité** : Pas d'optimisation globale
- **Ordre important** : Ordre des priorités crucial

### Exemple détaillé : Bâtiment intelligent

**Système** : Bâtiment avec contrôle HVAC

**Objectifs multiples** :

1. **Minimiser consommation énergétique** :
   $$
   J_1 = \sum_{i=1}^{N_p} P_{HVAC}(t+i)
   $$

2. **Maximiser confort** (minimiser écart température) :
   $$
   J_2 = \sum_{i=1}^{N_p} (T(t+i) - T_{comfort})^2
   $$

3. **Minimiser variations** (lissage) :
   $$
   J_3 = \sum_{i=1}^{N_p} (T(t+i) - T(t+i-1))^2
   $$

**Formulation pondérée** :
$$
J = w_1 J_1 + w_2 J_2 + w_3 J_3
$$

où $w_1 + w_2 + w_3 = 1$.

**Ajustement adaptatif des poids** :

**Hiver (économie prioritaire)** :
- $w_1 = 0.6$ (énergie)
- $w_2 = 0.3$ (confort)
- $w_3 = 0.1$ (lissage)

**Été (confort prioritaire)** :
- $w_1 = 0.3$ (énergie)
- $w_2 = 0.6$ (confort)
- $w_3 = 0.1$ (lissage)

**Occupation variable** :
- **Occupé** : $w_2$ élevé (confort important)
- **Inoccupé** : $w_1$ élevé (économie prioritaire)

**Formulation $\epsilon$-contrainte** :

$$
\begin{aligned}
\min_{\mathbf{u}} \quad & J_1 \quad \text{(minimiser énergie)} \\
\text{s.t.} \quad & J_2 \leq \epsilon_{comfort} \quad \text{(confort garanti)} \\
& J_3 \leq \epsilon_{smooth} \quad \text{(variations limitées)}
\end{aligned}
$$

**Avantages** :
- Confort garanti : $J_2 \leq \epsilon_{comfort}$
- Économie optimisée sous contrainte confort
- Ajustement simple : Modifier $\epsilon_{comfort}$ selon besoins

## 16.3 MPC avec référence variable

### Motivation

Dans certaines applications, la **référence change** de manière imprévisible ou complexe :

**Exemples** :
- **Véhicules** : Trajectoire de référence calculée par un planificateur
- **Processus** : Consignes variables selon la demande
- **Robots** : Trajectoires dynamiques avec obstacles

### Défis

1. **Prédiction de référence** : Connaître $r_{k+i}$ pour $i = 1, ..., N_p$
2. **Changements brusques** : Référence changeant soudainement
3. **Référence non connue** : Référence future inconnue

### Approches

#### 1. Prédiction de référence

**Principe** : Prédire la référence future à partir de l'historique.

**Méthodes** :
- **Extrapolation** : $r_{k+i} = r_k + i \Delta r_k$ (tendance)
- **Modèle de référence** : $r_{k+1} = f_r(r_k, ...)$
- **Filtrage** : Lisser les changements brusques

#### 2. Référence comme variable d'optimisation

**Formulation étendue** :
$$
\min_{\mathbf{u}, \mathbf{r}} \sum_{i=1}^{N_p} \|y_{k+i|k} - r_{k+i|k}\|_Q^2 + \|\mathbf{r} - \mathbf{r}_{nom}\|_R^2
$$

où $\mathbf{r}_{nom}$ est une référence nominale.

**Interprétation** : Optimiser à la fois la commande et la référence "idéale".

#### 3. MPC adaptative à la référence

**Principe** : Adapter les paramètres MPC selon la référence.

**Exemple** : Si la référence change rapidement, augmenter la réactivité (diminuer $R$).

#### 4. Horizon glissant de référence

**Principe** : Utiliser seulement les références connues avec certitude.

**Formulation** :
$$
J = \sum_{i=1}^{N_{known}} \|y_{k+i|k} - r_{k+i}\|_Q^2 + \sum_{i=N_{known}+1}^{N_p} \|y_{k+i|k} - \hat{r}_{k+i}\|_Q^2
$$

où $N_{known}$ est le nombre de références connues et $\hat{r}$ sont des prédictions.

## 16.4 Move Blocking MPC

### Motivation

Réduire le **nombre de variables de décision** pour diminuer la complexité computationnelle.

### Principe

Au lieu d'optimiser $u_{k|k}, u_{k+1|k}, ..., u_{k+N_c-1|k}$ indépendamment, **bloquer** certaines valeurs :

**Exemple** : Move blocking avec structure $[1, 1, 2, 4, 8]$ signifie :
- $u_{k|k}$ : Variable libre
- $u_{k+1|k}$ : Variable libre
- $u_{k+2|k} = u_{k+3|k}$ : Bloquées ensemble
- $u_{k+4|k} = u_{k+5|k} = u_{k+6|k} = u_{k+7|k}$ : Bloquées ensemble
- $u_{k+8|k} = ... = u_{k+N_c-1|k}$ : Bloquées ensemble

### Avantages

1. **Réduction de dimension** : Moins de variables à optimiser
2. **Temps de calcul** : Réduction significative
3. **Lissage naturel** : Commandes plus lisses sur l'horizon lointain

### Structures typiques

1. **Exponentielle** : $[1, 1, 2, 4, 8, ...]$ (double à chaque bloc)
2. **Linéaire** : $[1, 1, 2, 3, 4, ...]$ (augmente progressivement)
3. **Uniforme** : $[1, 1, 1, ..., 1]$ (pas de blocking, MPC standard)

### Choix de la structure

**Règle empirique** :
- **Horizon proche** : Pas de blocking (réactivité importante)
- **Horizon lointain** : Blocking plus agressif (précision moins critique)

**Optimisation** : La structure peut être optimisée pour équilibrer performance et complexité.

### Applications

- Systèmes avec contraintes temporelles strictes
- MPC avec horizon long
- Applications embarquées avec ressources limitées

## 16.5 MPC périodique

### Motivation

Pour les systèmes avec **comportement périodique** (jour/nuit, saisons, cycles de production), une MPC qui exploite cette périodicité peut améliorer les performances.

### Principe

**Système périodique** :
$$
x_{k+T} = x_k, \quad u_{k+T} = u_k
$$

où $T$ est la période.

**MPC périodique** : Optimiser sur une période complète avec contrainte de périodicité.

### Formulation

$$
\begin{aligned}
\min_{\mathbf{u}} \quad & \sum_{i=0}^{T-1} \ell(x_{k+i|k}, u_{k+i|k}) \\
\text{s.t.} \quad & x_{k+i+1|k} = f(x_{k+i|k}, u_{k+i|k}) \\
& x_{k+T|k} = x_{k|k} \quad \text{(périodicité)}
\end{aligned}
$$

### Avantages

1. **Optimisation globale** : Optimise sur tout le cycle
2. **Anticipation** : Exploite la connaissance du cycle
3. **Stabilité** : La contrainte de périodicité peut garantir la stabilité

### Applications

- **Smart grids** : Cycles journaliers de consommation
- **Bâtiments** : Cycles jour/nuit, semaines
- **Processus manufacturiers** : Cycles de production
- **Agriculture** : Cycles saisonniers

### Défis

1. **Période variable** : Si la période change, adaptation nécessaire
2. **Perturbations** : Gérer les déviations du cycle nominal
3. **Initialisation** : Synchronisation avec le cycle

### Variantes

**MPC périodique avec horizon glissant** :
- Optimise sur $T$ pas avec contrainte de périodicité
- Horizon glissant standard mais avec connaissance du cycle

**MPC adaptative périodique** :
- Ajuste le cycle appris en ligne
- Combine périodicité et adaptation

---

**Points clés du chapitre** :
- La MPC hybride gère les systèmes avec événements discrets
- La MPC multi-objectif traite les objectifs conflictuels
- La MPC avec référence variable s'adapte aux changements de consigne
- Le move blocking réduit la complexité computationnelle
- La MPC périodique exploite les cycles naturels des systèmes
- Chaque variante répond à des besoins spécifiques d'applications
