# Chapitre 2 : Modélisation des Systèmes Dynamiques

La qualité d'un contrôleur MPC dépend directement de la qualité du modèle utilisé pour la prédiction. Ce chapitre présente les différentes formes de modélisation adaptées à la MPC.

## 2.1 Représentation d'état

### 2.1.1 Systèmes linéaires invariants dans le temps (LTI)

Pour un système linéaire continu, la représentation d'état standard est :

$$
\begin{cases}
\dot{x}(t) = Ax(t) + Bu(t) \\
y(t) = Cx(t) + Du(t)
\end{cases}
$$

où :
- $x(t) \in \mathbb{R}^n$ est le vecteur d'état
- $u(t) \in \mathbb{R}^m$ est le vecteur d'entrée
- $y(t) \in \mathbb{R}^p$ est le vecteur de sortie
- $A$, $B$, $C$, $D$ sont des matrices de dimensions appropriées

**Propriétés importantes** :
- **Contrôlabilité** : Le système est contrôlable si la matrice $[B \ AB \ A^2B \ ... \ A^{n-1}B]$ est de rang plein.
- **Observabilité** : Le système est observable si la matrice $[C^T \ (CA)^T \ (CA^2)^T \ ... \ (CA^{n-1})^T]^T$ est de rang plein.
- **Stabilité** : Les valeurs propres de $A$ doivent avoir des parties réelles négatives pour la stabilité asymptotique.

### 2.1.2 Systèmes non-linéaires

Pour les systèmes non-linéaires, la représentation d'état devient :

$$
\begin{cases}
\dot{x}(t) = f(x(t), u(t)) \\
y(t) = h(x(t), u(t))
\end{cases}
$$

où $f$ et $h$ sont des fonctions non-linéaires.

**Approches de traitement** :
1. **Linéarisation autour d'un point d'équilibre** : Pour les petites variations
2. **Linéarisation successive** : À chaque pas de temps dans la MPC
3. **MPC non-linéaire (NMPC)** : Utilisation directe du modèle non-linéaire

### 2.1.3 Discrétisation des modèles continus

La MPC opère généralement en temps discret. Pour discrétiser un modèle continu, on utilise :

**Méthode d'Euler explicite** (approximation simple) :
$$
x_{k+1} = x_k + T_s (Ax_k + Bu_k)
$$

où $T_s$ est la période d'échantillonnage.

**Méthode exacte** (pour systèmes LTI) :
$$
x_{k+1} = A_d x_k + B_d u_k
$$

où :
$$
A_d = e^{AT_s}, \quad B_d = \int_0^{T_s} e^{A\tau} d\tau \cdot B
$$

**Choix de la période d'échantillonnage** :
- Trop petite : Charge computationnelle élevée, sensibilité au bruit
- Trop grande : Perte d'information, risque d'instabilité
- Règle empirique : $T_s \approx T_{95}/10$ où $T_{95}$ est le temps de réponse à 95%

## 2.2 Modèles de prédiction

### 2.2.1 Modèle de réponse impulsionnelle

Le modèle de réponse impulsionnelle (FIR - Finite Impulse Response) exprime la sortie comme :

$$
y_k = \sum_{i=1}^{N} h_i u_{k-i}
$$

où $h_i$ sont les coefficients de réponse impulsionnelle.

**Avantages** :
- Pas besoin de modèle d'état
- Identification simple à partir de données
- Utilisé dans le DMC (Dynamic Matrix Control)

**Inconvénients** :
- Nécessite un grand nombre de coefficients pour les systèmes lents
- Ne capture pas les modes instables

### 2.2.2 Modèle de réponse indicielle

Le modèle de réponse indicielle (Step Response Model) est similaire :

$$
y_k = y_0 + \sum_{i=1}^{N} s_i \Delta u_{k-i}
$$

où $s_i$ sont les coefficients de réponse indicielle et $\Delta u_k = u_k - u_{k-1}$.

**Utilisation** : Base du DMC classique, particulièrement adapté aux processus industriels.

### 2.2.3 Modèle d'état

Le modèle d'état discret est la forme la plus générale :

$$
\begin{cases}
x_{k+1} = f(x_k, u_k) \\
y_k = h(x_k, u_k)
\end{cases}
$$

**Avantages** :
- Forme compacte
- Capture la dynamique interne
- Permet l'analyse de stabilité
- Base de la plupart des formulations MPC modernes

**Forme linéaire** :
$$
\begin{cases}
x_{k+1} = Ax_k + Bu_k \\
y_k = Cx_k + Du_k
\end{cases}
$$

## 2.3 Identification des systèmes

L'identification consiste à déterminer les paramètres du modèle à partir de données expérimentales.

### 2.3.1 Méthodes paramétriques

**Moindres carrés** : Pour les modèles ARX (AutoRegressive with eXogenous input)

$$
y_k = -a_1 y_{k-1} - ... - a_n y_{k-n} + b_1 u_{k-1} + ... + b_m u_{k-m} + e_k
$$

**Maximum de vraisemblance** : Pour tenir compte des propriétés statistiques du bruit.

**Méthodes récursives** : RLS (Recursive Least Squares), adaptatives pour les systèmes variant dans le temps.

### 2.3.2 Méthodes non-paramétriques

**Analyse fréquentielle** : Identification par transformée de Fourier, analyse de la réponse fréquentielle.

**Méthodes de corrélation** : Estimation de la fonction de corrélation entrée-sortie.

**Méthodes modernes** : Identification par sous-espaces (Subspace Identification Methods), particulièrement efficace pour les systèmes multivariables.

### Validation du modèle

Après identification, il est crucial de valider le modèle :

1. **Validation sur données de test** : Comparer les prédictions avec des données non utilisées pour l'identification
2. **Analyse des résidus** : Vérifier l'absence de corrélation
3. **Critères de qualité** : FIT, RMSE, AIC, BIC

## 2.4 Gestion des incertitudes de modèle

Aucun modèle n'est parfaitement exact. Il est important de caractériser et gérer les incertitudes.

### Types d'incertitudes

1. **Incertitudes paramétriques** : Les paramètres du modèle ne sont connus qu'avec une certaine précision
2. **Incertitudes structurelles** : Le modèle ne capture pas tous les phénomènes (non-linéarités négligées, dynamiques non modélisées)
3. **Perturbations** : Bruit de mesure, perturbations externes

### Approches de gestion

**Modèles d'incertitude** :
- **Ensemble** : $A \in \mathcal{A}$, où $\mathcal{A}$ est un ensemble de matrices possibles
- **Stochastique** : Les paramètres suivent une distribution de probabilité
- **Norme bornée** : $\|\Delta A\| \leq \gamma$ pour une certaine norme

**MPC robuste** : Formulations qui garantissent la stabilité et le respect des contraintes malgré les incertitudes (voir Chapitre 8).

**MPC adaptative** : Ajustement en ligne des paramètres du modèle à partir des mesures.

### Exemple : Modèle avec incertitude additive

$$
x_{k+1} = Ax_k + Bu_k + w_k
$$

où $w_k$ représente l'incertitude, souvent modélisée comme :
- **Déterministe** : $w_k \in \mathcal{W}$ (ensemble borné)
- **Stochastique** : $w_k \sim \mathcal{N}(0, \Sigma)$ (bruit gaussien)

---

**Points clés du chapitre** :
- Le choix du modèle dépend de l'application et des données disponibles
- La discrétisation doit être effectuée avec soin pour préserver les propriétés du système
- L'identification nécessite des données de qualité et une validation rigoureuse
- La gestion des incertitudes est cruciale pour la robustesse de la MPC
