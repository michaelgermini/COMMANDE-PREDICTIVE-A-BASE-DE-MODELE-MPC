# Annexe A : Rappels Mathématiques

Cette annexe rappelle les concepts mathématiques fondamentaux utilisés dans la MPC.

## A.1 Algèbre linéaire

### Matrices et vecteurs

**Notation** :
- Vecteur : $\mathbf{x} \in \mathbb{R}^n$
- Matrice : $A \in \mathbb{R}^{m \times n}$
- Transposée : $A^T$
- Inverse : $A^{-1}$ (si $A$ carrée et inversible)

**Opérations** :
- Produit : $AB$ (dimensions compatibles)
- Produit scalaire : $\mathbf{x}^T \mathbf{y} = \sum_i x_i y_i$
- Norme : $\|\mathbf{x}\| = \sqrt{\mathbf{x}^T \mathbf{x}}$

### Valeurs propres et vecteurs propres

Pour $A \in \mathbb{R}^{n \times n}$ :
$$
A \mathbf{v} = \lambda \mathbf{v}
$$

où $\lambda$ est valeur propre, $\mathbf{v}$ vecteur propre.

**Propriétés** :
- Trace : $\text{tr}(A) = \sum_i \lambda_i$
- Déterminant : $\det(A) = \prod_i \lambda_i$
- Stabilité : $A$ stable si $\text{Re}(\lambda_i) < 0$ pour tout $i$

### Décomposition

**Décomposition en valeurs singulières (SVD)** :
$$
A = U \Sigma V^T
$$

où $U$, $V$ orthogonales, $\Sigma$ diagonale avec valeurs singulières.

**Décomposition de Cholesky** :
Pour $A$ symétrique définie positive :
$$
A = LL^T
$$

où $L$ est triangulaire inférieure.

### Normes matricielles

**Norme spectrale** : $\|A\|_2 = \sigma_{\max}(A)$ (plus grande valeur singulière)

**Norme de Frobenius** : $\|A\|_F = \sqrt{\sum_{i,j} a_{ij}^2}$

## A.2 Optimisation convexe

### Convexité

**Ensemble convexe** : Pour tout $\mathbf{x}, \mathbf{y} \in \mathcal{C}$ et $\lambda \in [0,1]$ :
$$
\lambda \mathbf{x} + (1-\lambda) \mathbf{y} \in \mathcal{C}
$$

**Fonction convexe** : Pour tout $\mathbf{x}, \mathbf{y}$ et $\lambda \in [0,1]$ :
$$
f(\lambda \mathbf{x} + (1-\lambda) \mathbf{y}) \leq \lambda f(\mathbf{x}) + (1-\lambda) f(\mathbf{y})
$$

### Programmation quadratique (QP)

**Forme standard** :
$$
\begin{aligned}
\min_{\mathbf{x}} \quad & \frac{1}{2} \mathbf{x}^T H \mathbf{x} + \mathbf{c}^T \mathbf{x} \\
\text{s.t.} \quad & A_{eq} \mathbf{x} = \mathbf{b}_{eq} \\
& A_{ineq} \mathbf{x} \leq \mathbf{b}_{ineq}
\end{aligned}
$$

**Conditions d'optimalité (KKT)** :
Si $H \succeq 0$ (semi-définie positive), le problème est convexe et :
- Solution unique si $H \succ 0$ (définie positive)
- Conditions KKT nécessaires et suffisantes

### Conditions KKT

Pour le problème :
$$
\min_{\mathbf{x}} f(\mathbf{x}) \text{ s.t. } g(\mathbf{x}) \leq 0, h(\mathbf{x}) = 0
$$

Les conditions KKT sont :
$$
\begin{aligned}
\nabla f(\mathbf{x}^*) + \sum_i \lambda_i \nabla g_i(\mathbf{x}^*) + \sum_j \mu_j \nabla h_j(\mathbf{x}^*) &= 0 \\
g_i(\mathbf{x}^*) &\leq 0 \\
h_j(\mathbf{x}^*) &= 0 \\
\lambda_i &\geq 0 \\
\lambda_i g_i(\mathbf{x}^*) &= 0
\end{aligned}
$$

### Programmation linéaire (LP)

**Forme standard** :
$$
\begin{aligned}
\min_{\mathbf{x}} \quad & \mathbf{c}^T \mathbf{x} \\
\text{s.t.} \quad & A \mathbf{x} = \mathbf{b} \\
& \mathbf{x} \geq 0
\end{aligned}
$$

**Méthode du simplexe** : Algorithme efficace pour LP.

## A.3 Théorie de Lyapunov

### Stabilité

**Point d'équilibre** : $\mathbf{x}_{eq}$ tel que $f(\mathbf{x}_{eq}) = 0$

**Stabilité de Lyapunov** : $\mathbf{x}_{eq}$ est stable si pour tout $\epsilon > 0$, il existe $\delta > 0$ tel que $\|\mathbf{x}(0) - \mathbf{x}_{eq}\| < \delta \Rightarrow \|\mathbf{x}(t) - \mathbf{x}_{eq}\| < \epsilon$ pour tout $t \geq 0$

**Stabilité asymptotique** : Stable + $\lim_{t \to \infty} \mathbf{x}(t) = \mathbf{x}_{eq}$

### Fonction de Lyapunov

**Théorème de Lyapunov** : Si existe $V(\mathbf{x})$ telle que :
1. $V(\mathbf{x}) > 0$ pour $\mathbf{x} \neq \mathbf{x}_{eq}$, $V(\mathbf{x}_{eq}) = 0$
2. $\dot{V}(\mathbf{x}) = \frac{\partial V}{\partial \mathbf{x}} f(\mathbf{x}) < 0$ pour $\mathbf{x} \neq \mathbf{x}_{eq}$

Alors $\mathbf{x}_{eq}$ est asymptotiquement stable.

**Fonction de Lyapunov quadratique** : Pour système linéaire $\dot{\mathbf{x}} = A \mathbf{x}$ :
$$
V(\mathbf{x}) = \mathbf{x}^T P \mathbf{x}
$$

où $P \succ 0$ et $A^T P + PA \prec 0$ (équation de Lyapunov).

### Stabilité discrète

Pour système discret $\mathbf{x}_{k+1} = A \mathbf{x}_k$ :
- Stable si $|\lambda_i| < 1$ pour tout $i$
- Fonction de Lyapunov : $V(\mathbf{x}) = \mathbf{x}^T P \mathbf{x}$ avec $A^T PA - P \prec 0$

## A.4 Systèmes dynamiques

### Représentation d'état

**Système continu** :
$$
\begin{cases}
\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}) \\
\mathbf{y} = h(\mathbf{x}, \mathbf{u})
\end{cases}
$$

**Système linéaire** :
$$
\begin{cases}
\dot{\mathbf{x}} = A \mathbf{x} + B \mathbf{u} \\
\mathbf{y} = C \mathbf{x} + D \mathbf{u}
\end{cases}
$$

### Discrétisation

**Méthode d'Euler** :
$$
\mathbf{x}_{k+1} = \mathbf{x}_k + T_s f(\mathbf{x}_k, \mathbf{u}_k)
$$

**Méthode exacte (systèmes LTI)** :
$$
\mathbf{x}_{k+1} = A_d \mathbf{x}_k + B_d \mathbf{u}_k
$$

où :
$$
A_d = e^{AT_s}, \quad B_d = \int_0^{T_s} e^{A\tau} d\tau \cdot B
$$

### Contrôlabilité et observabilité

**Contrôlabilité** : Le système est contrôlable si la matrice :
$$
\mathcal{C} = [B \ AB \ A^2B \ ... \ A^{n-1}B]
$$
est de rang plein ($\text{rank}(\mathcal{C}) = n$).

**Observabilité** : Le système est observable si la matrice :
$$
\mathcal{O} = [C^T \ (CA)^T \ (CA^2)^T \ ... \ (CA^{n-1})^T]^T
$$
est de rang plein ($\text{rank}(\mathcal{O}) = n$).

### Réponse impulsionnelle et indicielle

**Réponse impulsionnelle** : $h(t)$ pour $u(t) = \delta(t)$

**Réponse indicielle** : $s(t)$ pour $u(t) = 1$ pour $t \geq 0$

**Relation** : $s(t) = \int_0^t h(\tau) d\tau$

### Transformée de Laplace

**Définition** :
$$
F(s) = \mathcal{L}\{f(t)\} = \int_0^{\infty} f(t) e^{-st} dt
$$

**Fonction de transfert** : Pour système LTI :
$$
G(s) = C(sI - A)^{-1}B + D
$$

### Transformée en Z

**Définition** :
$$
F(z) = \mathcal{Z}\{f_k\} = \sum_{k=0}^{\infty} f_k z^{-k}
$$

**Fonction de transfert discrète** :
$$
G(z) = C(zI - A_d)^{-1}B_d + D
$$

---

**Points clés de l'annexe** :
- L'algèbre linéaire est fondamentale pour la MPC
- L'optimisation convexe garantit l'optimalité globale
- La théorie de Lyapunov permet d'analyser la stabilité
- Les systèmes dynamiques sont la base de la modélisation
- Ces concepts sont essentiels pour comprendre et appliquer la MPC
