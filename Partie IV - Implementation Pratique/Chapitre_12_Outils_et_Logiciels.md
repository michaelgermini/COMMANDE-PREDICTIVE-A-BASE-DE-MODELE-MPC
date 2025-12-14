# Chapitre 12 : Outils et Logiciels

Ce chapitre présente les principaux outils et logiciels disponibles pour développer et implémenter des contrôleurs MPC.

## 12.1 MATLAB/Simulink

MATLAB et Simulink sont les outils les plus répandus dans l'industrie et la recherche pour la MPC.

### 12.1.1 Model Predictive Control Toolbox

Le **MPC Toolbox** de MATLAB fournit des fonctionnalités complètes pour la MPC.

**Fonctionnalités principales** :

1. **Création de contrôleurs MPC** :
```matlab
mpcobj = mpc(model, Ts, p, m);
```
où `model` est le modèle du système, `Ts` la période d'échantillonnage, `p` et `m` les horizons.

2. **Réglage des paramètres** :
```matlab
mpcobj.PredictionHorizon = 20;
mpcobj.ControlHorizon = 5;
mpcobj.Weights.OutputVariables = [1 0.5];
mpcobj.Weights.ManipulatedVariables = 0.1;
```

3. **Définition des contraintes** :
```matlab
mpcobj.ManipulatedVariables.Min = 0;
mpcobj.ManipulatedVariables.Max = 100;
mpcobj.OutputVariables.Min = -10;
mpcobj.OutputVariables.Max = 10;
```

4. **Simulation** :
```matlab
sim(mpcobj, T, r, v);
```

5. **Génération de code** :
```matlab
generateCode(mpcobj);
```

**Avantages** :
- Interface graphique intuitive
- Documentation complète
- Intégration Simulink
- Génération de code C
- Support technique

**Inconvénients** :
- Coût de licence
- Principalement pour systèmes linéaires
- Moins flexible que les outils open-source

### 12.1.2 Exemples pratiques

**Exemple 1 : Contrôle de température**

```matlab
% Modèle du système
G = tf(1, [10 1]);
model = ss(G);

% Création du contrôleur MPC
mpcobj = mpc(model, 1, 20, 5);

% Réglage
mpcobj.Weights.OutputVariables = 1;
mpcobj.Weights.ManipulatedVariables = 0.1;

% Simulation
T = 100;
r = ones(T, 1) * 50; % Référence
sim(mpcobj, T, r);
```

**Exemple 2 : Système multivariable**

```matlab
% Modèle MIMO
A = [0.9 0.1; 0.05 0.95];
B = [1 0; 0 1];
C = eye(2);
model = ss(A, B, C, [], 1);

% MPC
mpcobj = mpc(model, 1, 15, 3);
mpcobj.Weights.OutputVariables = [1 1];
mpcobj.Weights.ManipulatedVariables = [0.1 0.1];
```

## 12.2 Python

Python offre un écosystème riche d'outils open-source pour la MPC.

### 12.2.1 do-mpc

**do-mpc** est un framework complet pour la MPC non-linéaire.

**Caractéristiques** :
- Support NMPC
- Intégration avec CasADi
- Estimation d'état
- Simulation et visualisation

**Exemple d'utilisation** :

```python
import do_mpc

# Modèle
model = do_mpc.model.Model('continuous')
x = model.set_variable('_x', 'x', 2)
u = model.set_variable('_u', 'u')
model.set_rhs('x', f(x, u))

# MPC
mpc = do_mpc.controller.MPC(model)
mpc.set_param(n_horizon=20, t_step=0.1)
mpc.set_objective(mterm=x**2, lterm=x**2 + u**2)
mpc.set_rterm(u=0.1)
mpc.bounds['lower', '_u', 'u'] = -1
mpc.bounds['upper', '_u', 'u'] = 1

# Setup
mpc.setup()

# Simulation
mpc.x0 = x0
u0 = mpc.make_step(x0)
```

**Avantages** :
- Open-source
- Flexible et extensible
- Bonne documentation
- Communauté active

### 12.2.2 CVXPY

**CVXPY** est un outil de modélisation pour l'optimisation convexe.

**Utilisation pour MPC** :

```python
import cvxpy as cp
import numpy as np

# Variables
u = cp.Variable((N, m))
x = cp.Variable((N+1, n))

# Contraintes
constraints = [x[0] == x0]
for i in range(N):
    constraints += [x[i+1] == A @ x[i] + B @ u[i]]
    constraints += [u_min <= u[i], u[i] <= u_max]
    constraints += [x_min <= x[i+1], x[i+1] <= x_max]

# Objectif
cost = cp.sum_squares(x - r) + 0.1 * cp.sum_squares(u)
prob = cp.Problem(cp.Minimize(cost), constraints)

# Résolution
prob.solve()
u_opt = u.value
```

**Avantages** :
- Syntaxe intuitive
- Support de nombreux solveurs
- Bon pour prototypage

**Inconvénients** :
- Moins optimisé pour temps réel
- Principalement pour problèmes convexes

### 12.2.3 CasADi

**CasADi** est un framework pour l'optimisation non-linéaire et la génération de code.

**Caractéristiques** :
- Symbolic computation
- Automatic differentiation
- Interface vers IPOPT, ACADOS
- Génération de code C

**Exemple** :

```python
import casadi as ca

# Variables symboliques
x = ca.MX.sym('x', n)
u = ca.MX.sym('u', m)

# Dynamique
x_next = f(x, u)

# Fonction
F = ca.Function('F', [x, u], [x_next])

# NLP
opti = ca.Opti()
X = opti.variable(n, N+1)
U = opti.variable(m, N)

# Contraintes
opti.subject_to(X[:, 0] == x0)
for k in range(N):
    opti.subject_to(X[:, k+1] == F(X[:, k], U[:, k]))
    opti.subject_to(u_min <= U[:, k] <= u_max)

# Objectif
J = sum(ca.sumsqr(X[:, k] - r) for k in range(N+1))
opti.minimize(J)

# Résolution
opti.solver('ipopt')
sol = opti.solve()
```

**Avantages** :
- Très puissant
- Efficace pour NMPC
- Génération de code

**Inconvénients** :
- Courbe d'apprentissage
- Syntaxe moins intuitive

## 12.3 Autres environnements

### 12.3.1 Julia (JuMP)

**Julia** avec **JuMP** offre des performances élevées pour l'optimisation.

**Exemple** :

```julia
using JuMP, Ipopt

model = Model(Ipopt.Optimizer)

@variable(model, u[1:N, 1:m])
@variable(model, x[1:N+1, 1:n])

@constraint(model, x[1, :] .== x0)
for i in 1:N
    @constraint(model, x[i+1, :] .== A * x[i, :] + B * u[i, :])
    @constraint(model, u_min .<= u[i, :] .<= u_max)
end

@objective(model, Min, sum((x[i, :] - r)' * Q * (x[i, :] - r) for i in 1:N+1) 
                        + sum(u[i, :]' * R * u[i, :] for i in 1:N))

optimize!(model)
```

**Avantages** :
- Performances élevées
- Syntaxe claire
- Écosystème croissant

### 12.3.2 C/C++ (ACADOS, OSQP)

Pour les applications embarquées, l'implémentation directe en C/C++ est souvent nécessaire.

**ACADOS** :
- Framework C avec interfaces Python/MATLAB
- Optimisé pour temps réel
- Génération de code

**OSQP** :
- Solveur QP en C
- Interface C++ disponible
- Très rapide

**Exemple OSQP en C** :

```c
#include "osqp.h"

// Définition du problème
c_float P_x[3] = {4.0, 1.0, 2.0};
c_int P_nnz = 3;
c_int P_i[3] = {0, 0, 1};
c_int P_p[3] = {0, 1, 3};

// Setup
OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));

osqp_setup(&work, data, settings);
osqp_solve(work);

// Solution
c_float *solution = work->solution->x;
```

## 12.4 Comparaison des outils

| Outil | Type | Coût | Complexité | NMPC | Temps réel | Génération code |
|-------|------|------|------------|------|------------|-----------------|
| MATLAB MPC Toolbox | Commercial | Payant | Faible | Non | Oui | Oui |
| do-mpc | Open-source | Gratuit | Modérée | Oui | Oui | Partiel |
| CVXPY | Open-source | Gratuit | Faible | Limité | Non | Non |
| CasADi | Open-source | Gratuit | Élevée | Oui | Oui | Oui |
| ACADOS | Open-source | Gratuit | Modérée | Oui | Oui | Oui |
| Julia/JuMP | Open-source | Gratuit | Modérée | Oui | Oui | Partiel |

### Recommandations

**Pour débutants** :
- MATLAB MPC Toolbox (si licence disponible)
- CVXPY pour comprendre les concepts

**Pour recherche/académique** :
- do-mpc ou CasADi pour NMPC
- Julia pour performances

**Pour applications industrielles** :
- MATLAB pour développement
- ACADOS ou code C pour déploiement

**Pour systèmes embarqués** :
- ACADOS ou OSQP en C/C++
- Génération de code depuis MATLAB/Python

---

**Points clés du chapitre** :
- MATLAB/Simulink reste la référence industrielle
- Python offre un écosystème riche et open-source
- Le choix dépend du contexte (recherche, industrie, embarqué)
- La génération de code est cruciale pour le déploiement
- Les outils évoluent rapidement, rester à jour avec les dernières versions
