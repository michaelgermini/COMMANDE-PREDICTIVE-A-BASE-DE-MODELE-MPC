# Chapitre 13 : MPC Économique (EMPC)

La MPC économique (Economic MPC, EMPC) étend la MPC classique en optimisant directement des critères économiques plutôt que le suivi de référence.

## 13.1 Motivation et objectifs

### Limitations de la MPC classique

La MPC classique optimise généralement :
- Suivi de référence
- Minimisation de l'effort de commande
- Lissage des commandes

Mais **pas directement** :
- Coût économique (énergie, matières premières)
- Profit, revenus
- Efficacité énergétique

### Objectifs de l'EMPC

L'EMPC optimise directement des **fonctions de coût économiques** :

$$
J = \sum_{i=0}^{N_p-1} \ell_e(x_{k+i|k}, u_{k+i|k})
$$

où $\ell_e$ est un **coût économique instantané**.

**Exemples de coûts économiques** :
- **Coût énergétique** : $\ell_e = \alpha P(u)$ où $P$ est la puissance
- **Coût des matières premières** : $\ell_e = \beta u$ (débit de réactif)
- **Revenu** : $\ell_e = -\gamma y$ (production vendue, négatif car on maximise)
- **Profit** : $\ell_e = \text{Revenu} - \text{Coût}$

### Applications typiques

- **Processus chimiques** : Optimisation du rendement, minimisation des réactifs
- **Smart grids** : Optimisation du coût de l'énergie
- **Bâtiments** : Minimisation de la consommation énergétique
- **Manufacturing** : Optimisation de la productivité

## 13.2 Formulation avec coût économique

### Formulation standard

$$
\begin{aligned}
\min_{\mathbf{u}} \quad & \sum_{i=0}^{N_p-1} \ell_e(x_{k+i|k}, u_{k+i|k}) \\
\text{s.t.} \quad & x_{k+i+1|k} = f(x_{k+i|k}, u_{k+i|k}) \\
& g(x_{k+i|k}, u_{k+i|k}) \leq 0 \\
& x_{k|k} = x_k
\end{aligned}
$$

### Différences avec MPC classique

| Aspect | MPC Classique | EMPC |
|--------|---------------|------|
| **Objectif** | Suivi de référence | Optimisation économique |
| **Fonction de coût** | Quadratique en erreur | Coût économique (souvent linéaire) |
| **Référence** | Nécessaire | Optionnelle |
| **Stabilité** | Facile à garantir | Plus complexe |

### Exemple : Optimisation énergétique d'un bâtiment

**Coût économique** :
$$
\ell_e = c_e P_{HVAC} + c_{comfort} (T - T_{ref})^2
$$

où :
- $c_e$ : Prix de l'électricité (variable dans le temps)
- $P_{HVAC}$ : Puissance du système HVAC
- $c_{comfort}$ : Coût de pénalisation du confort

**Formulation EMPC** :
$$
\min \sum_{i=0}^{N_p-1} c_e(k+i) P_{HVAC}(k+i) + c_{comfort} (T(k+i) - T_{ref})^2
$$

sous contraintes de température acceptable.

### Coûts économiques variables

L'EMPC peut gérer des **coûts variables dans le temps** :

**Exemple** : Tarification de l'électricité
- Heures creuses : $c_e = 0.10$ €/kWh
- Heures pleines : $c_e = 0.20$ €/kWh
- Heures de pointe : $c_e = 0.30$ €/kWh

L'EMPC peut **décaler** la consommation vers les heures creuses.

## 13.3 Différences avec la MPC de suivi

### MPC de suivi classique

**Objectif** : Suivre une référence $r_k$
$$
J = \sum_{i=1}^{N_p} \|y_{k+i|k} - r_{k+i}\|_Q^2 + \sum_{i=0}^{N_c-1} \|u_{k+i|k}\|_R^2
$$

**Comportement** : Le système converge vers $r_k$ (si faisable).

### EMPC

**Objectif** : Optimiser le coût économique
$$
J = \sum_{i=0}^{N_p-1} \ell_e(x_{k+i|k}, u_{k+i|k})
$$

**Comportement** : Le système converge vers un **point économique optimal** (qui peut être différent de toute référence).

### Point d'équilibre économique optimal

L'EMPC cherche à atteindre un **point d'équilibre économique optimal** (EOP - Economically Optimal Point) :

$$
(x^*, u^*) = \arg\min_{x, u} \ell_e(x, u) \text{ s.t. } x = f(x, u), g(x, u) \leq 0
$$

Ce point peut être :
- **Unique** : Un seul optimum économique
- **Multiple** : Plusieurs optima (problème plus complexe)
- **Périodique** : Optimum variant périodiquement (coûts variables)

### Exemple : Réacteur chimique

**MPC classique** :
- Objectif : Maintenir la température à $T_{ref} = 80°C$
- Comportement : Suit $T_{ref}$ même si ce n'est pas optimal économiquement

**EMPC** :
- Objectif : Maximiser le rendement (minimiser coût par unité produite)
- Comportement : Trouve la température optimale (peut être $75°C$ ou $85°C$ selon les conditions)

## 13.4 Stabilité de l'EMPC

### Défis de stabilité

La stabilité de l'EMPC est **plus complexe** que la MPC classique car :

1. **Pas de référence** : Pas de point d'équilibre "naturel" vers lequel converger
2. **Coûts non quadratiques** : Les coûts économiques sont souvent linéaires ou non convexes
3. **Optima multiples** : Risque de convergence vers un optimum local

### Méthodes de garantie de stabilité

**Méthode 1 : Contrainte terminale vers EOP**

Contraindre l'état terminal à l'EOP :
$$
x_{k+N_p|k} = x^*
$$

**Méthode 2 : Coût terminal**

Ajouter un coût terminal qui garantit la convergence :
$$
J = \sum_{i=0}^{N_p-1} \ell_e(x_{k+i|k}, u_{k+i|k}) + V_f(x_{k+N_p|k})
$$

où $V_f$ est une fonction de Lyapunov.

**Méthode 3 : Contrainte de décroissance**

Imposer que le coût économique décroisse :
$$
\sum_{i=0}^{N_p-1} \ell_e(x_{k+i|k}, u_{k+i|k}) \leq \sum_{i=0}^{N_p-1} \ell_e(x_{k+i|k-1}^*, u_{k+i|k-1}^*)
$$

**Méthode 4 : EMPC avec suivi**

Combiner EMPC et suivi :
$$
J = \alpha \sum_{i=0}^{N_p-1} \ell_e(x_{k+i|k}, u_{k+i|k}) + (1-\alpha) \sum_{i=1}^{N_p} \|y_{k+i|k} - r_{k+i}\|_Q^2
$$

où $\alpha \in [0,1]$ pondère l'importance économique vs suivi.

### Stabilité périodique

Si les coûts économiques sont **périodiques** (ex: tarification journalière), l'EMPC peut converger vers une **trajectoire périodique optimale** plutôt qu'un point d'équilibre.

**Théorème** : Sous certaines conditions, l'EMPC avec horizon suffisamment long converge vers la trajectoire périodique économiquement optimale.

## 13.5 Applications industrielles

### Industrie pétrochimique

**Objectif** : Optimiser le rendement d'une colonne de distillation

**Coût économique** :
$$
\ell_e = c_{feed} F_{feed} - c_{product} F_{product} - c_{energy} Q
$$

où :
- $F_{feed}$ : Débit d'alimentation
- $F_{product}$ : Débit de produit
- $Q$ : Énergie de chauffage

**Bénéfices** : Amélioration du rendement de 2-5% typiquement.

### Smart grids

**Objectif** : Optimiser la consommation/production d'énergie

**Coût économique** :
$$
\ell_e = c_e(t) P_{cons}(t) - c_s(t) P_{prod}(t) + c_{battery} |P_{batt}(t)|
$$

où les prix $c_e(t)$ et $c_s(t)$ varient dans le temps.

**Bénéfices** : Réduction des coûts énergétiques de 10-30%.

### Traitement des eaux

**Objectif** : Minimiser les coûts opérationnels

**Coût économique** :
$$
\ell_e = c_{chemical} u_{chemical} + c_{energy} P_{pump} + c_{penalty} \max(0, y - y_{max})
$$

**Bénéfices** : Réduction des coûts de 15-25% avec maintien de la qualité.

### Bâtiments intelligents

**Objectif** : Minimiser la consommation énergétique sous contraintes de confort

**Coût économique** :
$$
\ell_e = c_e(t) P_{total}(t) + c_{comfort} \|T(t) - T_{comfort}\|^2
$$

**Bénéfices** : Réduction de la consommation de 20-40% avec maintien du confort.

---

**Points clés du chapitre** :
- L'EMPC optimise directement des critères économiques
- Elle diffère fondamentalement de la MPC de suivi
- La stabilité nécessite des techniques spéciales
- Les applications montrent des gains économiques significatifs
- L'EMPC est particulièrement adaptée aux systèmes avec coûts variables dans le temps
