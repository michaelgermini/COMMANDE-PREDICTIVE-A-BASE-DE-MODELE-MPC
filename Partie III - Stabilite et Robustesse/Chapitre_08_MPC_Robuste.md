# Chapitre 8 : MPC Robuste

Les systèmes réels sont toujours soumis à des incertitudes. La MPC robuste garantit la stabilité et le respect des contraintes malgré ces incertitudes.

## 8.1 Sources d'incertitudes

### Types d'incertitudes

1. **Incertitudes paramétriques** :
   - Paramètres du modèle mal connus : $A(\theta)$, $B(\theta)$ où $\theta \in \Theta$
   - Exemple : Masse d'un véhicule variable, résistance d'un circuit

2. **Incertitudes non structurées** :
   - Erreurs de modélisation : $\Delta A$, $\Delta B$
   - Dynamiques négligées, non-linéarités approximées
   - Exemple : Friction non modélisée, aérodynamique complexe

3. **Perturbations externes** :
   - Bruit de mesure : $w_k$ dans les mesures
   - Perturbations : $d_k$ affectant le système
   - Exemple : Vent pour un drone, charge variable pour un moteur

4. **Erreurs d'état** :
   - État non mesuré directement (estimation)
   - Erreurs d'observateur
   - Exemple : Position estimée d'un robot, température interne

### Modélisation mathématique

**Système incertain** :
$$
x_{k+1} = f(x_k, u_k, w_k, \theta)
$$

où :
- $w_k \in \mathcal{W}$ : perturbation bornée
- $\theta \in \Theta$ : paramètre incertain

**Ensemble d'incertitudes** :
- **Déterministe** : $\mathcal{W}$, $\Theta$ sont des ensembles (polytopes, ellipsoïdes)
- **Stochastique** : $w_k \sim \mathcal{P}$, $\theta \sim \mathcal{P}_\theta$ (distributions de probabilité)

## 8.2 Min-Max MPC

La **Min-Max MPC** (ou MPC robuste déterministe) résout un problème de minimax : on minimise le pire cas sur toutes les incertitudes possibles.

### Formulation

$$
\begin{aligned}
\min_{\mathbf{u}} \max_{w \in \mathcal{W}, \theta \in \Theta} \quad & J(\mathbf{u}, \mathbf{x}(w, \theta)) \\
\text{s.t.} \quad & x_{k+i+1|k} = f(x_{k+i|k}, u_{k+i|k}, w_{k+i}, \theta) \\
& g(x_{k+i|k}, u_{k+i|k}) \leq 0, \quad \forall w, \theta
\end{aligned}
$$

**Interprétation** : On cherche la commande qui minimise le coût dans le pire scénario d'incertitudes.

### Avantages et inconvénients

**Avantages** :
- Garantit les performances dans tous les cas
- Respect des contraintes garanti
- Stabilité robuste

**Inconvénients** :
- **Conservatisme** : Optimise pour le pire cas, souvent trop pessimiste
- **Complexité** : Problème beaucoup plus complexe (optimisation bi-niveau)
- **Temps de calcul** : Très élevé, souvent prohibitif en temps réel

### Exemple : Système linéaire avec incertitude additive

Pour $x_{k+1} = Ax_k + Bu_k + w_k$ avec $w_k \in \mathcal{W}$ (polytope) :

$$
\min_{\mathbf{u}}} \max_{w \in \mathcal{W}} \sum_{i=0}^{N_p-1} \|x_{k+i|k}\|_Q^2 + \|u_{k+i|k}\|_R^2
$$

**Résolution** : Le problème peut être reformulé en un problème d'optimisation standard (mais de grande dimension).

## 8.3 Tube-based MPC

La **Tube-based MPC** est une approche plus pratique qui garantit que l'état reste dans un "tube" autour de la trajectoire nominale.

### 8.3.1 Concept de tube

L'idée est de :
1. Calculer une trajectoire **nominale** (sans incertitudes)
2. Garantir que la trajectoire réelle reste dans un **tube** autour de la nominale
3. Utiliser une loi de commande de **correction** pour maintenir l'état dans le tube

**Formulation** :
- **Trajectoire nominale** : $\bar{x}_{k+i|k}$, $\bar{u}_{k+i|k}$ (sans incertitudes)
- **Loi de correction** : $u_k = \bar{u}_k + K(x_k - \bar{x}_k)$
- **Tube** : $x_k \in \bar{x}_k \oplus \mathcal{R}$ où $\mathcal{R}$ est un ensemble robuste positif invariant (RPI)

### 8.3.2 Calcul des tubes

L'ensemble $\mathcal{R}$ doit être **robuste positif invariant** : si $x \in \bar{x} \oplus \mathcal{R}$, alors $f(x, \bar{u} + K(x - \bar{x}), w) \in \bar{x}^+ \oplus \mathcal{R}$ pour tout $w \in \mathcal{W}$.

**Calcul de $\mathcal{R}$** :
- Pour systèmes linéaires : $\mathcal{R}$ peut être un polytope ou un ellipsoïde
- Calculé via des méthodes d'ensemble invariant
- Le gain $K$ est choisi pour minimiser $\mathcal{R}$

**Méthodes** :
1. **Approche polytopique** : $\mathcal{R}$ comme polytope
2. **Approche ellipsoïdale** : $\mathcal{R}$ comme ellipsoïde (plus simple mais plus conservateur)

### 8.3.3 Implémentation

**Algorithme Tube-based MPC** :

1. **Problème nominal** : Résoudre la MPC nominale
   $$
   \min_{\bar{\mathbf{u}}} \sum_{i=0}^{N_p-1} \ell(\bar{x}_{k+i|k}, \bar{u}_{k+i|k})
   $$

2. **Contraintes robustifiées** : Les contraintes sont "rétrécies" pour tenir compte du tube
   $$
   \bar{x}_{k+i|k} \in \mathcal{X} \ominus \mathcal{R}_i
   $$
   où $\ominus$ est l'opération de Minkowski (rétrécissement).

3. **Application** : $u_k = \bar{u}_{k|k} + K(x_k - \bar{x}_{k|k})$

**Avantages** :
- Complexité similaire à la MPC nominale
- Garanties de robustesse
- Temps de calcul raisonnable

**Inconvénients** :
- Encore quelque peu conservateur
- Calcul de $\mathcal{R}$ peut être complexe

## 8.4 Stochastic MPC

La **Stochastic MPC** traite les incertitudes comme des variables aléatoires avec des distributions de probabilité connues.

### 8.4.1 Contraintes probabilistes (chance constraints)

Au lieu de garantir les contraintes pour tous les cas, on les garantit avec une **probabilité** :

$$
\mathbb{P}(g(x_k, u_k) \leq 0) \geq 1 - \epsilon
$$

où $\epsilon$ est un niveau de risque accepté (par exemple $\epsilon = 0.05$ pour 95% de confiance).

**Exemple** : Pour un réservoir, garantir que le niveau ne dépasse pas la capacité avec probabilité 95%.

### Formulation

$$
\begin{aligned}
\min_{\mathbf{u}} \quad & \mathbb{E}[J(\mathbf{u}, \mathbf{x}(w))] \\
\text{s.t.} \quad & \mathbb{P}(g(x_{k+i|k}, u_{k+i|k}) \leq 0) \geq 1 - \epsilon_i
\end{aligned}
$$

où $\mathbb{E}[\cdot]$ est l'espérance mathématique.

**Avantages** :
- Moins conservateur que Min-Max
- Permet de quantifier le risque
- Plus réaliste pour les applications avec bruit

**Inconvénients** :
- Nécessite la connaissance des distributions
- Résolution plus complexe
- Pas de garantie déterministe

### 8.4.2 Scenario-based MPC

La **Scenario-based MPC** (MPC basée sur scénarios) est une approche pratique et intuitive pour traiter les incertitudes stochastiques. Elle consiste à résoudre la MPC pour plusieurs réalisations possibles (scénarios) des incertitudes.

**Principe fondamental** :

Au lieu d'optimiser sur l'espérance mathématique (difficile à calculer), on :
1. **Génère** $N_s$ scénarios représentatifs des incertitudes
2. **Résout** la MPC pour tous les scénarios simultanément
3. **Garantit** que les contraintes sont satisfaites pour tous les scénarios

**Génération de scénarios** :

Les scénarios $w^{(1)}, ..., w^{(N_s)}$ sont générés selon la distribution de probabilité des incertitudes :

**Méthodes de génération** :
1. **Échantillonnage aléatoire** : $w^{(j)} \sim \mathcal{P}$ (distribution)
2. **Échantillonnage quasi-aléatoire** : Scénarios mieux distribués (Sobol, Halton)
3. **Moment matching** : Scénarios reproduisant moments statistiques
4. **Scénarios historiques** : Utiliser données passées

**Exemple** : Pour incertitude gaussienne $w \sim \mathcal{N}(0, \Sigma)$ :
- Générer $N_s$ échantillons selon $\mathcal{N}(0, \Sigma)$
- Ou utiliser points de quadrature pour meilleure couverture

**Formulation mathématique** :

Le problème devient :

$$
\begin{aligned}
\min_{\mathbf{u}} \quad & \frac{1}{N_s} \sum_{j=1}^{N_s} J(\mathbf{u}, \mathbf{x}^{(j)}) \\
\text{s.t.} \quad & x_{k+i+1|k}^{(j)} = f(x_{k+i|k}^{(j)}, u_{k+i|k}, w_{k+i}^{(j)}), \quad \forall j, i \\
& g(x_{k+i|k}^{(j)}, u_{k+i|k}) \leq 0, \quad \forall j, i \\
& x_{k|k}^{(j)} = x_k, \quad \forall j
\end{aligned}
$$

où :
- $\mathbf{x}^{(j)} = [x_{k+1|k}^{(j)}, ..., x_{k+N_p|k}^{(j)}]^T$ : Trajectoire pour scénario $j$
- $w^{(j)} = [w_{k}^{(j)}, ..., w_{k+N_p-1}^{(j)}]^T$ : Réalisation incertitudes scénario $j$
- $\mathbf{u} = [u_{k|k}, ..., u_{k+N_c-1|k}]^T$ : Commande (identique pour tous scénarios)

**Point clé** : La commande $\mathbf{u}$ est **identique** pour tous les scénarios (décision robuste).

**Structure du problème** :

Le problème a une structure particulière :
- **Variables de décision** : $\mathbf{u}$ (dimension $N_c \times m$)
- **Variables d'état** : $\mathbf{x}^{(j)}$ pour chaque scénario $j$ (dimension $N_s \times N_p \times n$)
- **Contraintes** : $N_s \times N_p$ contraintes (une par scénario et par pas)

**Dimension** : Le problème est $N_s$ fois plus grand que la MPC nominale.

**Avantages** :

1. **Simplicité** :
   - Formulation intuitive
   - Pas besoin de calculer espérances complexes
   - Facile à comprendre et expliquer

2. **Flexibilité** :
   - Fonctionne avec distributions arbitraires
   - Peut utiliser scénarios historiques
   - Facile d'ajouter/retirer scénarios

3. **Garanties probabilistes** :
   - Si $N_s$ suffisant, garantit contraintes avec probabilité élevée
   - Théorie de concentration de mesure

4. **Implémentation** :
   - Peut utiliser solveurs standards
   - Structure exploitable (parallélisation possible)

**Inconvénients** :

1. **Dimension** :
   - Problème $N_s$ fois plus grand
   - Temps de calcul multiplié
   - Mémoire nécessaire accrue

2. **Nombre de scénarios** :
   - $N_s$ nécessaire peut être élevé (typiquement 10-100)
   - Dépend de dimension incertitude, niveau de confiance
   - Pas de règle universelle

3. **Approximation** :
   - Pas de garantie exacte (approximation)
   - Qualité dépend nombre et qualité scénarios
   - Risque de sous-estimation si $N_s$ trop petit

**Choix du nombre de scénarios ($N_s$)** :

**Facteurs influençant $N_s$** :

1. **Dimension incertitude** :
   - Incertitude scalaire : $N_s = 10-20$ peut suffire
   - Incertitude vectorielle : $N_s = 50-100$ ou plus

2. **Niveau de confiance** :
   - 90% confiance : $N_s$ modéré
   - 99% confiance : $N_s$ plus élevé

3. **Temps de calcul disponible** :
   - Temps réel strict : $N_s$ limité (5-10)
   - Offline ou temps moins critique : $N_s$ plus grand (50-100)

4. **Complexité système** :
   - Système simple : $N_s$ plus petit
   - Système complexe : $N_s$ plus grand

**Règles empiriques** :
- **Minimum** : $N_s = 10$ (approximation grossière)
- **Typique** : $N_s = 20-50$ (bon compromis)
- **Précis** : $N_s = 100-200$ (approximation fine)
- **Théorique** : $N_s$ peut être calculé via théorie de concentration

**Méthodes d'échantillonnage** :

**Échantillonnage uniforme** :
- Simple mais peut nécessiter beaucoup de scénarios
- Risque de zones peu couvertes

**Échantillonnage adaptatif** :
- Concentrer scénarios dans zones critiques
- Réduire $N_s$ nécessaire

**Moment matching** :
- Scénarios reproduisant moyenne, variance, etc.
- Meilleure représentation avec moins de scénarios

**Exemple d'application** : Gestion de microgrid

**Système** : Microgrid avec production renouvelable incertaine

**Incertitude** : $P_{PV}(t)$ (production photovoltaïque) incertaine

**Scénarios** :
- $P_{PV}^{(1)}(t)$ : Scénario optimiste (soleil)
- $P_{PV}^{(2)}(t)$ : Scénario nominal (moyen)
- $P_{PV}^{(3)}(t)$ : Scénario pessimiste (nuages)
- $P_{PV}^{(4-20)}(t)$ : Autres scénarios (échantillonnage)

**MPC** :
$$
\min_{\mathbf{u}} \quad \frac{1}{20} \sum_{j=1}^{20} J(\mathbf{u}, P_{PV}^{(j)})
$$

sous contraintes pour tous les scénarios.

**Résultats** :
- Robustesse : Fonctionne pour tous les scénarios
- Performance : Optimise en moyenne
- Faisabilité : Garantie pour scénarios testés

## 8.5 MPC adaptative

La **MPC adaptative** ajuste le modèle en ligne à partir des mesures pour réduire les incertitudes.

### Principe

1. **Identification en ligne** : Estimer les paramètres $\theta$ du modèle à partir des données
2. **Mise à jour du modèle** : Utiliser le modèle mis à jour dans la MPC
3. **Récursion** : Répéter à chaque pas de temps

**Formulation adaptative** :
$$
\begin{aligned}
\text{Étape 1 :} \quad & \hat{\theta}_k = \arg\min_{\theta} \sum_{i=k-N}^{k-1} \|y_i - \hat{y}_i(\theta)\|^2 \\
\text{Étape 2 :} \quad & \text{Résoudre MPC avec } A(\hat{\theta}_k), B(\hat{\theta}_k) \\
\text{Étape 3 :} \quad & \text{Appliquer } u_k^*
\end{aligned}
$$

### Méthodes d'identification

**Moindres carrés récursifs (RLS)** :
- Mise à jour récursive des paramètres
- Efficace en temps réel
- Adapté aux variations lentes

**Filtre de Kalman étendu (EKF)** :
- Pour modèles non-linéaires
- Gère les incertitudes sur les paramètres
- Plus complexe

### Défis

1. **Persistance d'excitation** : Besoin de données riches pour identifier
2. **Stabilité** : L'adaptation peut introduire de l'instabilité
3. **Temps de calcul** : Identification + optimisation à chaque pas
4. **Convergence** : Garantir la convergence des paramètres

### Applications

- Systèmes avec paramètres variant dans le temps
- Usure, vieillissement
- Conditions opérationnelles changeantes

---

**Points clés du chapitre** :
- Les incertitudes sont omniprésentes dans les systèmes réels
- Min-Max MPC garantit le pire cas mais est très conservateur
- Tube-based MPC offre un compromis pratique
- Stochastic MPC quantifie le risque avec des probabilités
- MPC adaptative réduit les incertitudes en ligne
- Le choix de la méthode dépend du type d'incertitude et des contraintes de l'application
