# Chapitre 6 : Optimisation et Solveurs

La résolution efficace du problème d'optimisation est cruciale pour la MPC en temps réel. Ce chapitre présente les méthodes d'optimisation et les solveurs spécialisés.

## 6.1 Programmation quadratique (QP)

Un problème QP standard s'écrit :

$$
\begin{aligned}
\min_{\mathbf{x}} \quad & \frac{1}{2} \mathbf{x}^T H \mathbf{x} + \mathbf{c}^T \mathbf{x} \\
\text{s.t.} \quad & A_{eq} \mathbf{x} = \mathbf{b}_{eq} \\
& A_{ineq} \mathbf{x} \leq \mathbf{b}_{ineq} \\
& \mathbf{x}_{lb} \leq \mathbf{x} \leq \mathbf{x}_{ub}
\end{aligned}
$$

où $H \succeq 0$ (matrice semi-définie positive).

### 6.1.1 Méthode de l'ensemble actif

La méthode de l'ensemble actif identifie quelles contraintes sont actives (saturées) à l'optimum.

**Principe** :
1. Estimer l'ensemble actif $\mathcal{A}$
2. Résoudre le QP avec seulement les contraintes actives (problème non-contraint)
3. Vérifier les conditions d'optimalité (KKT)
4. Mettre à jour l'ensemble actif et itérer

**Conditions KKT** :
$$
\begin{aligned}
H \mathbf{x}^* + \mathbf{c} + A_{eq}^T \boldsymbol{\lambda}_{eq} + A_{ineq}^T \boldsymbol{\lambda}_{ineq} &= 0 \\
A_{eq} \mathbf{x}^* &= \mathbf{b}_{eq} \\
A_{ineq} \mathbf{x}^* &\leq \mathbf{b}_{ineq} \\
\boldsymbol{\lambda}_{ineq} &\geq 0 \\
\boldsymbol{\lambda}_{ineq}^T (A_{ineq} \mathbf{x}^* - \mathbf{b}_{ineq}) &= 0
\end{aligned}
$$

**Avantages** :
- Convergence en nombre fini d'itérations (théoriquement)
- Efficace pour les problèmes peu contraints

**Inconvénients** :
- Peut nécessiter de nombreux changements d'ensemble actif
- Coûteux si beaucoup de contraintes

### 6.1.2 Méthodes de points intérieurs

Les méthodes de points intérieurs traversent l'intérieur du domaine faisable plutôt que de suivre les bords.

**Principe** :
- Transformer les contraintes d'inégalité en barrières logarithmiques
- Résoudre une séquence de problèmes non-contraints
- Converger vers la solution optimale

**Formulation barrière** :
$$
\min_{\mathbf{x}} \quad \frac{1}{2} \mathbf{x}^T H \mathbf{x} + \mathbf{c}^T \mathbf{x} - \mu \sum_i \log(s_i)
$$

sous $A_{ineq} \mathbf{x} + \mathbf{s} = \mathbf{b}_{ineq}$, $\mathbf{s} > 0$, où $\mu > 0$ décroît vers 0.

**Avantages** :
- Convergence polynomiale
- Efficace pour les problèmes de grande taille
- Traitement uniforme des contraintes

**Inconvénients** :
- Nécessite un point initial strictement faisable
- Paramétrage du paramètre de barrière

## 6.2 Programmation non-linéaire (NLP)

Pour les problèmes NMPC, on doit résoudre des NLP :

$$
\begin{aligned}
\min_{\mathbf{x}} \quad & f(\mathbf{x}) \\
\text{s.t.} \quad & \mathbf{g}(\mathbf{x}) = 0 \\
& \mathbf{h}(\mathbf{x}) \leq 0
\end{aligned}
$$

### Méthodes principales

**SQP (Sequential Quadratic Programming)** :
- Résout une séquence de QP approximés
- Linéarise les contraintes à chaque itération
- Efficace pour les problèmes bien conditionnés

**Méthodes de points intérieurs** :
- Extension des méthodes QP
- Traverse l'intérieur du domaine
- Bonne convergence globale

**Méthodes de gradient** :
- Méthodes du premier ordre (moins précises mais rapides)
- Utiles pour les problèmes de très grande taille

## 6.3 Solveurs populaires

### 6.3.1 OSQP

**OSQP** (Operator Splitting Quadratic Program) est un solveur QP open-source très rapide.

**Caractéristiques** :
- Méthode : ADMM (Alternating Direction Method of Multipliers)
- Licence : Apache 2.0
- Langages : C, Python, MATLAB, Julia
- Spécialisé pour les problèmes creux (sparse)

**Avantages** :
- Très rapide pour les problèmes MPC
- Warm starting efficace
- Code simple et modulaire

**Utilisation typique** :
```python
import osqp
prob = osqp.OSQP()
prob.setup(P, q, A, l, u, ...)
result = prob.solve()
```

### 6.3.2 qpOASES

**qpOASES** est un solveur QP spécialisé pour la MPC, développé à l'ETH Zurich.

**Caractéristiques** :
- Méthode : Ensemble actif avec factorisation QR
- Licence : LGPL
- Langages : C++, MATLAB, Simulink
- Optimisé pour les problèmes récurrents (MPC)

**Avantages** :
- Warm starting très efficace
- Hot starting (réutilisation de la factorisation)
- Spécialement conçu pour la MPC

**Utilisation** :
```cpp
QProblem qp(nV, nC);
qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);
qp.getPrimalSolution(xOpt);
```

### 6.3.3 IPOPT

**IPOPT** (Interior Point OPTimizer) est un solveur NLP puissant.

**Caractéristiques** :
- Méthode : Points intérieurs avec recherche linéaire
- Licence : EPL (Eclipse Public License)
- Langages : C++, interfaces Python, MATLAB, Julia
- Gère les problèmes de grande taille

**Avantages** :
- Robuste et fiable
- Bonne convergence
- Large communauté

**Inconvénients** :
- Plus lent que les solveurs QP spécialisés
- Nécessite les gradients (ou approximations)

### 6.3.4 ACADOS

**ACADOS** (A Computationally Efficient, Modular and Extensible Software Framework for Embedded Optimization) est un framework moderne pour l'optimisation embarquée.

**Caractéristiques** :
- Méthodes : SQP, points intérieurs, collocation
- Licence : LGPL
- Langages : C, Python, MATLAB
- Spécialisé pour la MPC/NMPC embarquée

**Avantages** :
- Très rapide (optimisé pour temps réel)
- Support NMPC natif
- Génération de code automatique
- Structure MPC exploitée

**Utilisation** :
```python
from acados_template import AcadosOcp, AcadosOcpSolver
ocp = AcadosOcp()
ocp_solver = AcadosOcpSolver(ocp)
status = ocp_solver.solve()
```

## 6.4 Optimisation en temps réel

### 6.4.1 Warm starting

Le **warm starting** utilise la solution de l'instant précédent comme initialisation.

**Pour QP** :
- Initialiser avec $\mathbf{u}^{(0)}_k = [u_{k-1}^*, u_{k}^*, ..., u_{k+N_c-2}^*]$
- L'ensemble actif est souvent similaire
- Réduction significative du nombre d'itérations

**Pour NLP** :
- Initialiser avec la trajectoire précédente
- Les multiplicateurs de Lagrange peuvent être réutilisés
- Amélioration importante de la convergence

**Impact** : Réduction typique de 50-90% du temps de calcul.

### 6.4.2 Early termination

L'**early termination** accepte une solution sous-optimale si :
- Le temps alloué est dépassé
- Une solution "assez bonne" est trouvée
- Les améliorations deviennent marginales

**Critères d'arrêt adaptatifs** :
- Tolérance sur l'optimalité : $\|\nabla L\| < \epsilon$
- Tolérance sur la faisabilité : $\|\mathbf{g}(\mathbf{x})\| < \delta$
- Nombre maximum d'itérations

**Stratégie** :
1. Commencer avec tolérances strictes
2. Si le temps presse, relâcher les tolérances
3. Accepter la meilleure solution trouvée

**Risques** :
- Violation des contraintes possibles
- Performances dégradées
- Instabilité potentielle

### Autres techniques

**Condensing** : Réduire la dimension du problème en éliminant les variables d'état

**Move blocking** : Réduire le nombre de variables de commande en bloquant certaines valeurs

**Horizon adaptatif** : Ajuster dynamiquement $N_p$ et $N_c$ selon le temps disponible

**Parallélisation** : Utiliser plusieurs cœurs pour les calculs indépendants

---

**Points clés du chapitre** :
- Les solveurs QP sont très efficaces pour la MPC linéaire
- Les solveurs NLP sont nécessaires pour la NMPC mais plus lents
- OSQP, qpOASES, IPOPT et ACADOS sont des choix populaires
- Le warm starting et l'early termination sont essentiels pour le temps réel
- Le choix du solveur dépend de l'application et des contraintes temporelles
