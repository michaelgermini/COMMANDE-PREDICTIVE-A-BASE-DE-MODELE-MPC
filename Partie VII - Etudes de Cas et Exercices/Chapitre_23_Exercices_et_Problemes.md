# Chapitre 23 : Exercices et Problèmes

Ce chapitre propose des exercices progressifs pour maîtriser la MPC, de la modélisation à l'implémentation avancée.

## 23.1 Exercices de modélisation

### Exercice 1 : Modèle d'un système du premier ordre

**Énoncé** : Un système thermique est décrit par :
$$
\tau \frac{dT}{dt} + T = K u
$$

où $T$ est la température, $u$ la commande, $\tau = 10$ s, $K = 2$ °C/unité.

**Questions** :
1. Discrétiser le modèle avec $T_s = 1$ s
2. Calculer la réponse indicielle
3. Déterminer le temps de réponse à 95%

**Solution** :
1. Forme d'état : $\dot{T} = -\frac{1}{\tau} T + \frac{K}{\tau} u$
   Discrétisation : $T_{k+1} = e^{-T_s/\tau} T_k + K(1-e^{-T_s/\tau}) u_k$
   Avec $T_s = 1$, $\tau = 10$ : $T_{k+1} = 0.905 T_k + 0.19 u_k$

2. Réponse indicielle : $u = 1$ constant
   $T(k) = K(1 - e^{-k/\tau}) = 2(1 - e^{-k/10})$

3. Temps de réponse : $T_{95} = 3\tau = 30$ s

### Exercice 2 : Modèle d'un système du second ordre

**Énoncé** : Système masse-ressort-amortisseur :
$$
m \ddot{x} + c \dot{x} + k x = F
$$

avec $m = 1$ kg, $c = 2$ N·s/m, $k = 5$ N/m.

**Questions** :
1. Mettre sous forme d'état
2. Discrétiser avec $T_s = 0.1$ s
3. Analyser la stabilité

**Solution** :
1. État : $x_1 = x$, $x_2 = \dot{x}$
   $$
   \begin{bmatrix}
   \dot{x}_1 \\
   \dot{x}_2
   \end{bmatrix}
   =
   \begin{bmatrix}
   0 & 1 \\
   -k/m & -c/m
   \end{bmatrix}
   \begin{bmatrix}
   x_1 \\
   x_2
   \end{bmatrix}
   +
   \begin{bmatrix}
   0 \\
   1/m
   \end{bmatrix}
   F
   $$

2. Discrétisation : $A_d = e^{AT_s}$, $B_d = \int_0^{T_s} e^{A\tau} d\tau \cdot B$
   (Calcul numérique nécessaire)

3. Valeurs propres de $A$ : $\lambda = \frac{-c \pm \sqrt{c^2-4mk}}{2m} = -1 \pm 2j$
   Parties réelles négatives → système stable

### Exercice 3 : Identification de modèle

**Données** : Réponse indicielle mesurée

| Temps (s) | 0 | 1 | 2 | 3 | 4 | 5 | 10 | 20 |
|-----------|---|---|---|---|---|---|----|----|
| Sortie    | 0 | 0.5 | 1.2 | 1.8 | 2.2 | 2.5 | 2.9 | 3.0 |

**Questions** :
1. Identifier un modèle du premier ordre
2. Valider le modèle

**Solution** :
1. Gain : $K = 3.0$ (valeur finale)
   Constante de temps : $\tau \approx 3$ s (63% de la valeur finale)
   Modèle : $G(s) = \frac{3}{3s+1}$

2. Validation : Comparer prédictions avec données

## 23.2 Exercices de formulation MPC

### Exercice 4 : Formulation QP simple

**Énoncé** : Système $x_{k+1} = 0.9 x_k + 0.5 u_k$, $y_k = x_k$

**Questions** :
1. Écrire les prédictions sur $N_p = 3$ en fonction de $x_k$ et $\mathbf{u} = [u_k, u_{k+1}]^T$
2. Formuler le problème QP pour minimiser $J = \sum_{i=1}^3 (y_{k+i} - 1)^2 + 0.1 \sum_{i=0}^1 u_{k+i}^2$
3. Résoudre analytiquement

**Solution** :
1. Prédictions :
   $$
   \begin{bmatrix}
   y_{k+1} \\
   y_{k+2} \\
   y_{k+3}
   \end{bmatrix}
   =
   \begin{bmatrix}
   0.9 \\
   0.81 \\
   0.729
   \end{bmatrix}
   x_k
   +
   \begin{bmatrix}
   0.5 & 0 \\
   0.45 & 0.5 \\
   0.405 & 0.45
   \end{bmatrix}
   \begin{bmatrix}
   u_k \\
   u_{k+1}
   \end{bmatrix}
   $$

2. QP : $\min_{\mathbf{u}} \frac{1}{2} \mathbf{u}^T H \mathbf{u} + \mathbf{c}^T \mathbf{u}$
   où $H$ et $\mathbf{c}$ sont calculés à partir des prédictions

3. Solution : $\mathbf{u}^* = -H^{-1} \mathbf{c}$

### Exercice 5 : Contraintes

**Énoncé** : Même système que l'exercice 4, avec contraintes :
- $|u_k| \leq 1$
- $|y_k| \leq 2$

**Questions** :
1. Écrire les contraintes sous forme $G \mathbf{u} \leq g$
2. Résoudre le QP contraint

**Solution** :
1. Contraintes sur $u$ : $-1 \leq u_k \leq 1$, $-1 \leq u_{k+1} \leq 1$
   Contraintes sur $y$ : Utiliser les prédictions pour exprimer en fonction de $\mathbf{u}$

2. Résolution avec solveur QP (ex: `quadprog` MATLAB)

### Exercice 6 : Choix des horizons

**Énoncé** : Système avec temps de réponse $T_{95} = 20$ s, période $T_s = 1$ s.

**Questions** :
1. Recommander $N_p$ et $N_c$
2. Justifier le choix

**Solution** :
1. $N_p \geq T_{95}/T_s = 20$, donc $N_p = 20-30$
   $N_c \approx N_p/3$ à $N_p/2$, donc $N_c = 5-10$

2. $N_p$ doit couvrir le temps de réponse pour anticipation
   $N_c$ plus petit pour réduire complexité

## 23.3 Exercices d'implémentation

### Exercice 7 : MPC simple en MATLAB

**Énoncé** : Implémenter une MPC pour le système de l'exercice 4.

**Questions** :
1. Écrire le code MATLAB complet
2. Simuler avec référence $r = 1$
3. Analyser les résultats

**Solution** :
```matlab
% Modèle
A = 0.9;
B = 0.5;
C = 1;
model = ss(A, B, C, 0, 1);

% MPC
mpcobj = mpc(model, 1, 3, 2);
mpcobj.Weights.OutputVariables = 1;
mpcobj.Weights.ManipulatedVariables = 0.1;

% Simulation
T = 20;
r = ones(T, 1);
[y, u] = sim(mpcobj, T, r);

% Visualisation
plot(y); hold on; plot(r, '--'); legend('Sortie', 'Référence');
```

### Exercice 8 : MPC avec contraintes

**Énoncé** : Ajouter les contraintes de l'exercice 5.

**Questions** :
1. Modifier le code
2. Comparer avec/sans contraintes

**Solution** :
```matlab
% Ajouter contraintes
mpcobj.ManipulatedVariables.Min = -1;
mpcobj.ManipulatedVariables.Max = 1;
mpcobj.OutputVariables.Min = -2;
mpcobj.OutputVariables.Max = 2;
```

### Exercice 9 : MPC en Python

**Énoncé** : Réimplémenter l'exercice 7 en Python avec `do-mpc` ou `cvxpy`.

**Questions** :
1. Écrire le code Python
2. Comparer avec MATLAB

**Solution** :
```python
import numpy as np
import cvxpy as cp

# Paramètres
A, B = 0.9, 0.5
Np, Nc = 3, 2
Q, R = 1, 0.1

# Variables
u = cp.Variable(Nc)
x = cp.Variable(Np+1)

# Contraintes
constraints = [x[0] == x0]
for i in range(Np):
    constraints += [x[i+1] == A*x[i] + B*u[min(i, Nc-1)]]
    if i < Nc:
        constraints += [-1 <= u[i], u[i] <= 1]

# Objectif
cost = cp.sum_squares(x[1:] - 1) + R*cp.sum_squares(u)
prob = cp.Problem(cp.Minimize(cost), constraints)
prob.solve()
```

## 23.4 Problèmes avancés

### Problème 1 : MPC non-linéaire

**Énoncé** : Système non-linéaire $\dot{x} = -x^2 + u$.

**Questions** :
1. Formuler le problème NMPC
2. Implémenter avec CasADi
3. Comparer avec linéarisation successive

**Indications** :
- Utiliser CasADi pour NLP
- Linéariser autour de la trajectoire courante
- Comparer performances et temps de calcul

### Problème 2 : MPC robuste

**Énoncé** : Système avec incertitude $x_{k+1} = (0.9 + \delta) x_k + 0.5 u_k$ où $|\delta| \leq 0.1$.

**Questions** :
1. Formuler une MPC robuste (Min-Max)
2. Comparer avec MPC nominale
3. Analyser la robustesse

**Indications** :
- Formuler le problème Min-Max
- Résoudre pour différents $\delta$
- Analyser la dégradation des performances

### Problème 3 : MPC économique

**Énoncé** : Système avec coût énergétique $c_e(t)$ variable.

**Questions** :
1. Formuler l'EMPC
2. Optimiser avec tarification jour/nuit
3. Comparer avec MPC de suivi

**Indications** :
- Coût : $c_e(t) P(t)$ où $c_e$ varie
- Objectif : Minimiser coût total
- Analyser le décalage de consommation

## 23.5 Solutions commentées

### Solution Exercice 1 (détaillée)

**Étape 1 : Discrétisation**

Pour un système $\dot{x} = ax + bu$, la solution exacte discrétisée est :
$$
x_{k+1} = e^{aT_s} x_k + \frac{b}{a}(e^{aT_s} - 1) u_k
$$

Avec $a = -1/\tau = -0.1$, $b = K/\tau = 0.2$ :
$$
T_{k+1} = e^{-0.1} T_k + 2(e^{-0.1} - 1) u_k = 0.905 T_k + 0.19 u_k
$$

**Étape 2 : Réponse indicielle**

Pour $u_k = 1$ constant et $T_0 = 0$ :
$$
T_k = K(1 - e^{-k/\tau}) = 2(1 - e^{-k/10})
$$

**Étape 3 : Temps de réponse**

$T_{95}$ est tel que $T(T_{95}) = 0.95 \times K = 1.9$
$$
1.9 = 2(1 - e^{-T_{95}/10}) \Rightarrow T_{95} = -10 \ln(0.05) \approx 30 \text{ s}
$$

### Solution Exercice 4 (détaillée)

**Étape 1 : Prédictions**

Pour $x_{k+1} = Ax_k + Bu_k$ :
- $x_{k+1} = Ax_k + Bu_k$
- $x_{k+2} = A^2 x_k + AB u_k + Bu_{k+1}$
- $x_{k+3} = A^3 x_k + A^2B u_k + AB u_{k+1}$

Avec $A = 0.9$, $B = 0.5$ :
$$
\mathbf{y} = \mathbf{F} x_k + \mathbf{G} \mathbf{u}
$$

où $\mathbf{F} = [0.9, 0.81, 0.729]^T$ et
$$
\mathbf{G} = \begin{bmatrix}
0.5 & 0 \\
0.45 & 0.5 \\
0.405 & 0.45
\end{bmatrix}
$$

**Étape 2 : Formulation QP**

$J = \|\mathbf{y} - \mathbf{1}\|^2 + 0.1 \|\mathbf{u}\|^2$

Substituant $\mathbf{y}$ :
$$
J = \|\mathbf{F} x_k + \mathbf{G} \mathbf{u} - \mathbf{1}\|^2 + 0.1 \|\mathbf{u}\|^2
$$

Développant :
$$
J = \mathbf{u}^T (\mathbf{G}^T \mathbf{G} + 0.1 \mathbf{I}) \mathbf{u} + 2(\mathbf{F} x_k - \mathbf{1})^T \mathbf{G} \mathbf{u} + \text{constante}
$$

Donc $H = \mathbf{G}^T \mathbf{G} + 0.1 \mathbf{I}$ et $\mathbf{c} = \mathbf{G}^T (\mathbf{F} x_k - \mathbf{1})$

**Étape 3 : Solution**

$\mathbf{u}^* = -H^{-1} \mathbf{c}$ (calcul numérique)

---

**Points clés du chapitre** :
- Les exercices progressent de la modélisation à l'implémentation
- Chaque exercice renforce des concepts spécifiques
- Les problèmes avancés introduisent des extensions
- Les solutions commentées expliquent la démarche
- La pratique est essentielle pour maîtriser la MPC
