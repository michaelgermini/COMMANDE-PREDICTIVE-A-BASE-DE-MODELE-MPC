# Chapitre 7 : Analyse de Stabilité

La stabilité en boucle fermée est une propriété fondamentale pour tout système de contrôle. Ce chapitre présente les méthodes d'analyse et de garantie de stabilité pour la MPC.

## 7.1 Stabilité en boucle fermée

### Problème fondamental

Contrairement au contrôle LQR (qui garantit la stabilité par construction), la MPC avec horizon fini **ne garantit pas automatiquement la stabilité**. En effet :

- L'horizon fini peut être trop court pour "voir" les conséquences à long terme
- L'optimisation locale peut ne pas garantir un comportement stable global
- Les contraintes peuvent empêcher d'atteindre l'état d'équilibre

### Définition de stabilité

Pour un système en boucle fermée :

$$
x_{k+1} = f(x_k, \kappa_{MPC}(x_k))
$$

où $\kappa_{MPC}$ est la loi de commande MPC, on cherche à garantir :

1. **Stabilité asymptotique** : $\lim_{k \to \infty} x_k = x_{eq}$ pour tout $x_0$ dans un domaine d'attraction
2. **Stabilité exponentielle** : Convergence avec un taux exponentiel
3. **Stabilité robuste** : Stabilité maintenue sous incertitudes

## 7.2 Contrainte terminale

Une méthode classique pour garantir la stabilité consiste à imposer une **contrainte terminale**.

### 7.2.1 Ensemble terminal

L'idée est de contraindre l'état à l'horizon $N_p$ à appartenir à un **ensemble terminal** $\mathcal{X}_f$ :

$$
x_{k+N_p|k} \in \mathcal{X}_f
$$

**Choix de $\mathcal{X}_f$** :

1. **Point d'équilibre** : $\mathcal{X}_f = \{x_{eq}\}$ (contrainte la plus simple mais restrictive)

2. **Ensemble invariant** : Un ensemble $\mathcal{X}_f$ tel que si $x \in \mathcal{X}_f$, alors $f(x, \kappa_f(x)) \in \mathcal{X}_f$ pour une loi de commande terminale $\kappa_f$

3. **Ensemble de contrôle invariant** : Ensemble où il existe toujours une commande admissible qui maintient l'état dans l'ensemble

**Avantages** :
- Garantit la stabilité (sous certaines conditions)
- Permet de prouver la faisabilité récursive

**Inconvénients** :
- Peut être très restrictif (petit domaine d'attraction)
- Difficile à calculer pour les systèmes non-linéaires

### 7.2.2 Coût terminal

Une alternative (ou complément) est d'ajouter un **coût terminal** $V_f(x_{k+N_p|k})$ dans la fonction de coût :

$$
J = \sum_{i=1}^{N_p-1} \ell(x_{k+i|k}, u_{k+i|k}) + V_f(x_{k+N_p|k})
$$

**Choix de $V_f$** :

1. **Fonction de Lyapunov** : $V_f$ est une fonction de Lyapunov pour le système en boucle fermée avec une loi terminale $\kappa_f$

2. **Coût infini approché** : $V_f$ approche le coût sur un horizon infini

3. **Fonction quadratique** : $V_f(x) = x^T P x$ où $P$ est solution d'une équation de Riccati

**Condition de stabilité** : Si $V_f$ est une fonction de Lyapunov et vérifie :

$$
V_f(f(x, \kappa_f(x))) - V_f(x) \leq -\ell(x, \kappa_f(x))
$$

alors la stabilité est garantie.

### Combinaison contrainte + coût terminal

La méthode la plus puissante combine les deux :

$$
\begin{aligned}
\min \quad & \sum_{i=0}^{N_p-1} \ell(x_{k+i|k}, u_{k+i|k}) + V_f(x_{k+N_p|k}) \\
\text{s.t.} \quad & x_{k+N_p|k} \in \mathcal{X}_f \\
& u_{k+i|k} \in \mathcal{U}, \quad i = 0, ..., N_p-1
\end{aligned}
$$

**Théorème de stabilité** : Si :
- $\mathcal{X}_f$ est un ensemble invariant sous $\kappa_f$
- $V_f$ est une fonction de Lyapunov
- Les conditions de décroissance sont satisfaites

Alors le système en boucle fermée est asymptotiquement stable.

## 7.3 Fonction de Lyapunov et MPC

### Principe

Une **fonction de Lyapunov** $V(x)$ est une fonction qui :
- $V(x) > 0$ pour $x \neq x_{eq}$
- $V(x_{eq}) = 0$
- $\Delta V(x) = V(f(x, \kappa(x))) - V(x) < 0$ (décroissance)

### Fonction de Lyapunov pour MPC

La **valeur optimale** $V^*(x_k) = J^*(x_k)$ du problème MPC peut servir de fonction de Lyapunov.

**Théorème** : Si $V^*(x)$ est la valeur optimale et vérifie :

$$
V^*(f(x, \kappa_{MPC}(x))) - V^*(x) \leq -\ell(x, \kappa_{MPC}(x))
$$

alors le système est stable.

**Preuve intuitive** : La valeur optimale décroît à chaque pas, garantissant la convergence.

### Calcul pratique

Pour les systèmes linéaires avec coût quadratique, $V^*(x) = x^T P x$ où $P$ peut être calculé analytiquement.

Pour les systèmes non-linéaires, $V^*$ doit être calculée numériquement, ce qui est complexe.

## 7.4 Stabilité nominale vs stabilité robuste

### Stabilité nominale

La **stabilité nominale** suppose que le modèle est parfaitement exact. C'est une condition nécessaire mais pas suffisante pour les applications réelles.

**Garanties** :
- Stabilité sous le modèle nominal
- Convergence vers l'équilibre désiré
- Pas de garantie sous incertitudes

### Stabilité robuste

La **stabilité robuste** garantit la stabilité même en présence d'incertitudes sur le modèle.

**Types d'incertitudes** :
- **Paramétriques** : Paramètres incertains $p \in \mathcal{P}$
- **Non structurées** : Erreurs de modélisation $\Delta$
- **Perturbations** : Bruit, perturbations externes $w \in \mathcal{W}$

**Approches** :
1. **MPC robuste** : Formulation qui garantit la stabilité pour toutes les incertitudes possibles (voir Chapitre 8)
2. **Analyse de robustesse** : Vérifier a posteriori la robustesse d'une MPC nominale
3. **MPC adaptative** : Ajuster le modèle en ligne

## 7.5 MPC avec horizon infini

### Principe

La **MPC avec horizon infini** ($N_p = \infty$) résout théoriquement le problème de stabilité car elle optimise sur un horizon infini, similaire au LQR.

**Formulation** :
$$
\min_{\mathbf{u}} \quad \sum_{i=0}^{\infty} \ell(x_{k+i|k}, u_{k+i|k})
$$

### Problèmes pratiques

1. **Incalculable** : Impossible de résoudre directement avec un horizon infini
2. **Approximation** : Utiliser un horizon très long (mais coûteux)
3. **Méthodes indirectes** : Utiliser des techniques d'approximation

### Solutions approchées

**Approximation par horizon long** :
- Utiliser $N_p$ très grand
- Risque de charge computationnelle élevée

**Dual-mode MPC** :
- Horizon fini + loi terminale qui garantit la stabilité
- Équivalent à un horizon infini sous certaines conditions

**Explicit MPC** :
- Pré-calculer la loi de commande
- Permet d'analyser la stabilité a priori

### Relation avec LQR

Si le problème MPC avec horizon infini est non-contraint, il se réduit au **LQR** (Linear Quadratic Regulator), qui garantit la stabilité.

**LQR** :
$$
u_k = -K x_k
$$

où $K$ est le gain optimal calculé via l'équation de Riccati.

---

**Points clés du chapitre** :
- La MPC avec horizon fini ne garantit pas automatiquement la stabilité
- Les contraintes terminales et coûts terminaux permettent de garantir la stabilité
- La fonction de valeur optimale peut servir de fonction de Lyapunov
- La distinction entre stabilité nominale et robuste est cruciale
- La MPC avec horizon infini garantit la stabilité mais est difficile à implémenter
