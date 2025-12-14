# Chapitre 3 : Formulation Mathématique de la MPC

Ce chapitre présente la formulation mathématique complète d'un problème de commande prédictive, incluant les horizons, la fonction de coût, les contraintes et le problème d'optimisation résultant.

## 3.1 Horizon de prédiction et de commande

### Horizon de prédiction ($N_p$)

L'**horizon de prédiction** $N_p$ est le nombre de pas de temps futurs sur lesquels on prédit le comportement du système. Il détermine jusqu'où dans le futur on "regarde" pour prendre une décision.

**Influence de $N_p$** :
- **Petit $N_p$** : Réactivité élevée, mais risque d'instabilité et de performances sous-optimales
- **Grand $N_p$** : Meilleure anticipation, mais charge computationnelle accrue
- **Règle empirique** : $N_p$ devrait couvrir au moins le temps de réponse du système

### Horizon de commande ($N_c$)

L'**horizon de commande** $N_c$ (ou horizon de contrôle) est le nombre de pas de temps sur lesquels on optimise les commandes futures. Typiquement, $N_c \leq N_p$.

**Stratégies** :
- **$N_c = N_p$** : Optimisation complète sur tout l'horizon
- **$N_c < N_p$** : Après $N_c$ pas, on suppose $u_k = u_{N_c-1}$ (commande bloquée) ou $u_k = 0$

**Notation** : À l'instant $k$, on optimise :
- $u_{k|k}, u_{k+1|k}, ..., u_{k+N_c-1|k}$ : séquence de commandes
- $x_{k+1|k}, ..., x_{k+N_p|k}$ : prédictions d'état
- $y_{k+1|k}, ..., y_{k+N_p|k}$ : prédictions de sortie

où $u_{i|k}$ signifie "commande à l'instant $i$ calculée à l'instant $k$".

## 3.2 Fonction de coût (critère de performance)

La fonction de coût $J$ quantifie la performance désirée sur l'horizon de prédiction.

### 3.2.1 Critère quadratique

La forme la plus courante est le critère quadratique :

$$
J = \sum_{i=1}^{N_p} \|y_{k+i|k} - r_{k+i}\|_Q^2 + \sum_{i=0}^{N_c-1} \|u_{k+i|k}\|_R^2 + \sum_{i=0}^{N_c-1} \|\Delta u_{k+i|k}\|_S^2
$$

où :
- $r_{k+i}$ : référence à suivre
- $\|x\|_Q^2 = x^T Q x$ : norme pondérée
- $Q \succeq 0$ : matrice de pondération sur les erreurs de sortie
- $R \succeq 0$ : matrice de pondération sur les commandes
- $S \succeq 0$ : matrice de pondération sur les variations de commande
- $\Delta u_k = u_k - u_{k-1}$ : variation de commande

**Interprétation** :
- Premier terme : **Suivi de référence** (minimiser l'erreur de suivi)
- Deuxième terme : **Économie d'effort** (éviter les commandes trop grandes)
- Troisième terme : **Lissage** (éviter les variations brusques)

### 3.2.2 Pondération des variables

**Choix de $Q$** :
- Grande valeur : priorité au suivi de référence
- Petite valeur : tolérance aux erreurs
- Matrice diagonale : pondération indépendante pour chaque sortie
- Matrice non-diagonale : prise en compte des corrélations

**Choix de $R$** :
- Grande valeur : pénalisation forte des commandes (économie d'énergie)
- Petite valeur : autorisation de commandes importantes
- Permet de gérer les limites d'actionnement indirectement

**Choix de $S$** :
- Contrôle la "douceur" de la commande
- Important pour préserver la durée de vie des actionneurs
- Évite les oscillations rapides

### 3.2.3 Pénalisation des variations de commande

Le terme $\sum_{i=0}^{N_c-1} \|\Delta u_{k+i|k}\|_S^2$ est crucial car :

1. **Réduction des sollicitations mécaniques** : Évite les à-coups
2. **Amélioration du confort** : Particulièrement important pour les applications véhicules
3. **Stabilité numérique** : Aide à la convergence de l'optimisation

**Formulation alternative** : Au lieu de pénaliser $\Delta u$, on peut directement contraindre $|\Delta u_k| \leq \Delta u_{\max}$.

## 3.3 Contraintes

La capacité à gérer explicitement les contraintes est l'un des principaux avantages de la MPC.

### 3.3.1 Contraintes sur les entrées

**Contraintes de magnitude** :
$$
u_{\min} \leq u_{k+i|k} \leq u_{\max}, \quad i = 0, ..., N_c-1
$$

**Contraintes de variation** :
$$
\Delta u_{\min} \leq \Delta u_{k+i|k} \leq \Delta u_{\max}, \quad i = 0, ..., N_c-1
$$

**Exemple** : Pour un moteur, on peut avoir :
- $u_{\min} = 0$ (pas de commande négative)
- $u_{\max} = 100\%$ (puissance maximale)
- $|\Delta u_k| \leq 10\%$ par seconde (limitation de la vitesse de variation)

### 3.3.2 Contraintes sur les sorties

**Contraintes de sortie** :
$$
y_{\min} \leq y_{k+i|k} \leq y_{\max}, \quad i = 1, ..., N_p
$$

**Exemple** : Pour un réservoir :
- $y_{\min} = 0$ (pas de niveau négatif)
- $y_{\max} = h_{\max}$ (hauteur maximale du réservoir)

**Note importante** : Les contraintes sur les sorties doivent être vérifiées sur tout l'horizon de prédiction, pas seulement sur l'horizon de commande.

### 3.3.3 Contraintes sur les états

**Contraintes d'état** :
$$
x_{\min} \leq x_{k+i|k} \leq x_{\max}, \quad i = 1, ..., N_p
$$

**Exemple** : Pour un pendule inversé :
- Contrainte sur l'angle : $|\theta| \leq \theta_{\max}$
- Contrainte sur la position du chariot : $|x| \leq x_{\max}$

### 3.3.4 Contraintes souples vs contraintes dures

**Contraintes dures** : Doivent être respectées strictement
$$
y_{k+i|k} \leq y_{\max}
$$

**Contraintes souples** : Peuvent être violées avec pénalisation
$$
y_{k+i|k} \leq y_{\max} + \epsilon_{k+i}, \quad \epsilon_{k+i} \geq 0
$$

avec pénalisation dans la fonction de coût :
$$
J = ... + \rho \sum_{i=1}^{N_p} \epsilon_{k+i}^2
$$

où $\rho > 0$ est un poids de pénalisation (très grand pour approcher une contrainte dure).

**Avantages des contraintes souples** :
- Garantissent la faisabilité du problème d'optimisation
- Permettent des violations temporaires en cas de perturbations importantes
- Améliorent la robustesse

## 3.4 Problème d'optimisation résultant

### Formulation standard

À chaque instant $k$, la MPC résout :

$$
\begin{aligned}
\min_{u_{k|k}, ..., u_{k+N_c-1|k}} \quad & J(x_k, u_{k|k}, ..., u_{k+N_c-1|k}) \\
\text{s.t.} \quad & x_{k+i+1|k} = f(x_{k+i|k}, u_{k+i|k}), \quad i = 0, ..., N_p-1 \\
& y_{k+i|k} = h(x_{k+i|k}, u_{k+i|k}), \quad i = 1, ..., N_p \\
& u_{\min} \leq u_{k+i|k} \leq u_{\max}, \quad i = 0, ..., N_c-1 \\
& y_{\min} \leq y_{k+i|k} \leq y_{\max}, \quad i = 1, ..., N_p \\
& x_{k|k} = x_k \text{ (état mesuré ou estimé)}
\end{aligned}
$$

### Cas linéaire : Programme Quadratique (QP)

Pour un système linéaire avec critère quadratique, le problème devient un **Programme Quadratique (QP)** :

$$
\begin{aligned}
\min_{\mathbf{u}} \quad & \frac{1}{2} \mathbf{u}^T H \mathbf{u} + \mathbf{c}^T \mathbf{u} \\
\text{s.t.} \quad & G \mathbf{u} \leq \mathbf{g} \\
& A_{eq} \mathbf{u} = \mathbf{b}_{eq}
\end{aligned}
$$

où $\mathbf{u} = [u_{k|k}^T, ..., u_{k+N_c-1|k}^T]^T$ est le vecteur de décision.

**Avantages du QP** :
- Convexité garantie (si $H \succeq 0$)
- Solveurs efficaces disponibles
- Solution globale garantie

### Cas non-linéaire : Programme Non-Linéaire (NLP)

Pour les systèmes non-linéaires, on obtient un **Programme Non-Linéaire (NLP)** :

$$
\begin{aligned}
\min_{\mathbf{u}} \quad & J(\mathbf{u}, \mathbf{x}) \\
\text{s.t.} \quad & g(\mathbf{u}, \mathbf{x}) \leq 0 \\
& h(\mathbf{u}, \mathbf{x}) = 0
\end{aligned}
$$

**Défis** :
- Non-convexité possible
- Multiples minima locaux
- Temps de calcul plus élevé

### Structure du problème

Le problème MPC a une structure particulière :

1. **Problème récurrent** : Résolu à chaque pas de temps
2. **Problème séquentiel** : Les contraintes couplent les variables temporelles
3. **Problème contraint** : Nombreuses contraintes (typiquement $O(N_p)$)

Cette structure peut être exploitée par des solveurs spécialisés (voir Chapitre 6).

---

**Points clés du chapitre** :
- Les horizons $N_p$ et $N_c$ sont des paramètres de réglage cruciaux
- La fonction de coût combine suivi, économie d'effort et lissage
- Les contraintes peuvent être dures ou souples selon les besoins
- Le problème résultant est un QP (linéaire) ou NLP (non-linéaire) à résoudre en temps réel
