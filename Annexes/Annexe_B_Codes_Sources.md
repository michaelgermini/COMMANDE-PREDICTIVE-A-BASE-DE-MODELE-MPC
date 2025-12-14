# Annexe B : Codes Sources

Cette annexe fournit des templates et exemples de code pour implémenter la MPC.

## B.1 Templates MATLAB

### Template MPC basique

```matlab
%% Configuration MPC
% Modèle du système
A = [0.9 0.1; 0 0.95];
B = [0.5; 1];
C = [1 0];
D = 0;
model = ss(A, B, C, D, 1); % Période d'échantillonnage = 1s

% Création du contrôleur MPC
mpcobj = mpc(model, 1, 20, 5); % Ts, Np, Nc

% Réglage des pondérations
mpcobj.Weights.OutputVariables = 1;
mpcobj.Weights.ManipulatedVariables = 0.1;
mpcobj.Weights.ManipulatedVariablesRate = 0.01;

% Définition des contraintes
mpcobj.ManipulatedVariables.Min = -10;
mpcobj.ManipulatedVariables.Max = 10;
mpcobj.ManipulatedVariablesRate.Min = -2;
mpcobj.ManipulatedVariablesRate.Max = 2;
mpcobj.OutputVariables.Min = -5;
mpcobj.OutputVariables.Max = 5;

% Simulation
T = 100;
r = ones(T, 1) * 1; % Référence
[y, u, x] = sim(mpcobj, T, r);

% Visualisation
figure;
subplot(2,1,1); plot(y); hold on; plot(r, '--'); 
ylabel('Sortie'); legend('Sortie', 'Référence');
subplot(2,1,2); plot(u); ylabel('Commande');
```

### Template MPC personnalisée (QP manuel)

```matlab
function u = mpc_custom(A, B, C, x, r, Np, Nc, Q, R, S, umin, umax)
    % Construction des matrices de prédiction
    n = size(A, 1);
    m = size(B, 2);
    p = size(C, 1);
    
    % Matrice F (réponse libre)
    F = zeros(Np*p, n);
    for i = 1:Np
        F((i-1)*p+1:i*p, :) = C * A^i;
    end
    
    % Matrice G (réponse forcée)
    G = zeros(Np*p, Nc*m);
    for i = 1:Np
        for j = 1:min(i, Nc)
            G((i-1)*p+1:i*p, (j-1)*m+1:j*m) = C * A^(i-j) * B;
        end
    end
    
    % Matrice de pondération
    Qbar = kron(eye(Np), Q);
    Rbar = kron(eye(Nc), R);
    Sbar = kron(eye(Nc), S);
    
    % Hessienne
    H = G' * Qbar * G + Rbar + Sbar;
    
    % Gradient
    c = G' * Qbar * (F * x - kron(ones(Np,1), r));
    
    % Contraintes
    lb = kron(ones(Nc,1), umin);
    ub = kron(ones(Nc,1), umax);
    
    % Résolution QP
    options = optimoptions('quadprog', 'Display', 'off');
    u_seq = quadprog(H, c, [], [], [], [], lb, ub, [], options);
    
    % Retourner première commande
    u = u_seq(1:m);
end
```

### Template MPC non-linéaire avec fmincon

```matlab
function u = nmpc_custom(f, h, x, r, Np, Nc, Q, R, umin, umax)
    % Variables de décision
    u_vec = zeros(Nc, 1);
    
    % Fonction objectif
    function J = objective(u_vec)
        J = 0;
        x_pred = x;
        for i = 1:Np
            u_i = u_vec(min(i, Nc));
            x_pred = f(x_pred, u_i);
            y_pred = h(x_pred, u_i);
            J = J + (y_pred - r)' * Q * (y_pred - r);
            if i <= Nc
                J = J + u_i' * R * u_i;
            end
        end
    end
    
    % Contraintes
    function [c, ceq] = constraints(u_vec)
        c = [];
        ceq = [];
        x_pred = x;
        for i = 1:Np
            u_i = u_vec(min(i, Nc));
            x_pred = f(x_pred, u_i);
            % Ajouter contraintes ici
        end
    end
    
    % Résolution
    options = optimoptions('fmincon', 'Display', 'off');
    u_opt = fmincon(@objective, u_vec, [], [], [], [], ...
                    umin*ones(Nc,1), umax*ones(Nc,1), ...
                    @constraints, options);
    
    u = u_opt(1);
end
```

## B.2 Templates Python

### Template avec do-mpc

```python
import do_mpc
import numpy as np

# Modèle
model = do_mpc.model.Model('discrete')
x = model.set_variable('_x', 'x', 2)
u = model.set_variable('_u', 'u')
model.set_rhs('x', A @ x + B @ u)
model.setup()

# MPC
mpc = do_mpc.controller.MPC(model)
mpc.set_param(n_horizon=20, t_step=0.1)
mpc.set_objective(mterm=x.T @ Q @ x, lterm=x.T @ Q @ x + u.T @ R @ u)
mpc.set_rterm(u=0.1)
mpc.bounds['lower', '_u', 'u'] = -10
mpc.bounds['upper', '_u', 'u'] = 10
mpc.setup()

# Simulation
mpc.x0 = x0
for k in range(100):
    u0 = mpc.make_step(x0)
    x0 = simulate(x0, u0)
```

### Template avec CVXPY

```python
import cvxpy as cp
import numpy as np

def mpc_cvxpy(A, B, C, x0, r, Np, Nc, Q, R, umin, umax):
    n, m, p = A.shape[0], B.shape[1], C.shape[0]
    
    # Variables
    u = cp.Variable((Nc, m))
    x = cp.Variable((Np+1, n))
    
    # Contraintes
    constraints = [x[0] == x0]
    for i in range(Np):
        u_i = u[min(i, Nc-1), :]
        constraints += [x[i+1] == A @ x[i] + B @ u_i]
        if i < Nc:
            constraints += [umin <= u[i, :], u[i, :] <= umax]
    
    # Objectif
    cost = 0
    for i in range(1, Np+1):
        y = C @ x[i]
        cost += cp.quad_form(y - r, Q)
    for i in range(Nc):
        cost += cp.quad_form(u[i, :], R)
    
    # Résolution
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve()
    
    return u.value[0, :]
```

### Template avec CasADi

```python
import casadi as ca
import numpy as np

# Variables symboliques
x = ca.MX.sym('x', n)
u = ca.MX.sym('u', m)

# Dynamique
x_next = A @ x + B @ u
F = ca.Function('F', [x, u], [x_next])

# NLP
opti = ca.Opti()
X = opti.variable(n, Np+1)
U = opti.variable(m, Nc)

# Contraintes
opti.subject_to(X[:, 0] == x0)
for k in range(Np):
    u_k = U[:, min(k, Nc-1)]
    opti.subject_to(X[:, k+1] == F(X[:, k], u_k))
    if k < Nc:
        opti.subject_to(umin <= u_k <= umax)

# Objectif
J = 0
for k in range(Np+1):
    J += (X[:, k] - r).T @ Q @ (X[:, k] - r)
for k in range(Nc):
    J += U[:, k].T @ R @ U[:, k]
opti.minimize(J)

# Résolution
opti.solver('ipopt')
sol = opti.solve()
u_opt = sol.value(U[:, 0])
```

## B.3 Exemples complets commentés

### Exemple 1 : Contrôle de température (MATLAB)

```matlab
%% Exemple complet : Contrôle de température d'un four
% Modèle : T(k+1) = 0.95*T(k) + 0.1*P(k)
% Objectif : Maintenir T = 200°C

clear; close all;

% Paramètres
Ts = 1; % Période d'échantillonnage (min)
Np = 20; % Horizon de prédiction
Nc = 5;  % Horizon de commande
T_ref = 200; % Température de référence (°C)

% Modèle
A = 0.95;
B = 0.1;
C = 1;
model = ss(A, B, C, 0, Ts);

% MPC
mpcobj = mpc(model, Ts, Np, Nc);

% Pondérations
mpcobj.Weights.OutputVariables = 1;      % Priorité sur suivi
mpcobj.Weights.ManipulatedVariables = 0.1; % Économie d'effort
mpcobj.Weights.ManipulatedVariablesRate = 0.01; % Lissage

% Contraintes
mpcobj.ManipulatedVariables.Min = 0;     % Pas de refroidissement
mpcobj.ManipulatedVariables.Max = 10;    % Puissance max 10 kW
mpcobj.OutputVariables.Min = 180;        % Température min
mpcobj.OutputVariables.Max = 220;       % Température max

% Simulation
T_sim = 100; % Durée simulation (min)
r = T_ref * ones(T_sim, 1); % Référence constante

% Perturbation (ouverture de porte à t=50)
v = zeros(T_sim, 1);
v(50:55) = -20; % Perturbation de -20°C

[y, u, x, info] = sim(mpcobj, T_sim, r, v);

% Visualisation
figure;
subplot(2,1,1);
plot(y, 'b-', 'LineWidth', 2); hold on;
plot(r, 'r--', 'LineWidth', 1.5);
xlabel('Temps (min)');
ylabel('Température (°C)');
legend('Température', 'Référence', 'Location', 'best');
grid on;
title('Contrôle de température avec MPC');

subplot(2,1,2);
plot(u, 'g-', 'LineWidth', 2);
xlabel('Temps (min)');
ylabel('Puissance (kW)');
legend('Commande MPC', 'Location', 'best');
grid on;
```

### Exemple 2 : Pendule inversé (Python)

```python
"""
Exemple : Stabilisation d'un pendule inversé
Modèle linéarisé autour de theta = 0
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.linalg import solve_continuous_are

# Paramètres
M = 1.0  # Masse chariot (kg)
m = 0.1  # Masse pendule (kg)
l = 0.5  # Longueur (m)
g = 9.81 # Gravité (m/s²)

# Modèle linéarisé
A = np.array([[0, 1, 0, 0],
              [0, 0, -m*g/M, 0],
              [0, 0, 0, 1],
              [0, 0, (M+m)*g/(M*l), 0]])

B = np.array([[0],
              [1/M],
              [0],
              [-1/(M*l)]])

C = np.eye(4)
D = np.zeros((4, 1))

# Discrétisation
Ts = 0.1
from scipy.linalg import expm
Ad = expm(A * Ts)
Bd = np.linalg.solve(A, (Ad - np.eye(4))) @ B

# MPC
def mpc_pendulum(x0, Np=20, Nc=5):
    n, m = 4, 1
    
    # Pondérations
    Q = np.diag([1, 0.1, 10, 0.1])  # Priorité sur theta
    R = np.array([[0.1]])
    
    # Fonction objectif
    def objective(u_vec):
        u_vec = u_vec.reshape(Nc, m)
        J = 0
        x = x0.copy()
        for i in range(Np):
            u_i = u_vec[min(i, Nc-1), 0]
            x = Ad @ x + Bd @ np.array([[u_i]])
            J += x.T @ Q @ x
            if i < Nc:
                J += u_i * R[0,0] * u_i
        return J
    
    # Contraintes
    bounds = [(-10, 10)] * Nc  # Force limitée
    
    # Résolution
    u0 = np.zeros(Nc)
    result = minimize(objective, u0, method='L-BFGS-B', bounds=bounds)
    
    return result.x[0]  # Retourner première commande

# Simulation
T_sim = 10
x = np.array([[0.1], [0], [0.2], [0]])  # État initial
x_history = [x.copy()]
u_history = []

for k in range(int(T_sim/Ts)):
    u = mpc_pendulum(x)
    x = Ad @ x + Bd @ np.array([[u]])
    x_history.append(x.copy())
    u_history.append(u)

# Visualisation
t = np.arange(0, T_sim+Ts, Ts)
x_history = np.array(x_history).squeeze()

plt.figure(figsize=(12, 8))
plt.subplot(2,2,1)
plt.plot(t, x_history[:, 0])
plt.xlabel('Temps (s)')
plt.ylabel('Position chariot (m)')
plt.grid(True)

plt.subplot(2,2,2)
plt.plot(t, x_history[:, 1])
plt.xlabel('Temps (s)')
plt.ylabel('Vitesse chariot (m/s)')
plt.grid(True)

plt.subplot(2,2,3)
plt.plot(t, x_history[:, 2])
plt.xlabel('Temps (s)')
plt.ylabel('Angle pendule (rad)')
plt.grid(True)

plt.subplot(2,2,4)
plt.plot(t[:-1], u_history)
plt.xlabel('Temps (s)')
plt.ylabel('Force (N)')
plt.grid(True)

plt.tight_layout()
plt.show()
```

---

**Points clés de l'annexe** :
- Les templates fournissent des bases réutilisables
- Les exemples illustrent des applications concrètes
- Le code est commenté pour faciliter la compréhension
- Différents outils sont présentés (MATLAB, Python)
- Les exemples couvrent différents niveaux de complexité
