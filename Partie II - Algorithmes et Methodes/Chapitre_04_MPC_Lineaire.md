# Chapitre 4 : MPC Linéaire (LMPC)

La MPC linéaire est la forme la plus répandue et la mieux maîtrisée de la commande prédictive. Ce chapitre présente les principales approches et algorithmes pour les systèmes linéaires.

## 4.1 Formulation du problème QP (Quadratic Programming)

Pour un système linéaire discret :

$$
\begin{cases}
x_{k+1} = Ax_k + Bu_k \\
y_k = Cx_k
\end{cases}
$$

avec un critère quadratique :

$$
J = \sum_{i=1}^{N_p} \|y_{k+i|k} - r_{k+i}\|_Q^2 + \sum_{i=0}^{N_c-1} \|u_{k+i|k}\|_R^2 + \sum_{i=0}^{N_c-1} \|\Delta u_{k+i|k}\|_S^2
$$

on peut réécrire le problème sous forme de **Programme Quadratique (QP)** standard.

### Élimination des états

En utilisant le modèle, on peut exprimer les prédictions de sortie en fonction de l'état actuel $x_k$ et de la séquence de commandes $\mathbf{u} = [u_{k|k}^T, ..., u_{k+N_c-1|k}^T]^T$ :

$$
\mathbf{y} = \mathbf{F} x_k + \mathbf{G} \mathbf{u}
$$

où $\mathbf{y} = [y_{k+1|k}^T, ..., y_{k+N_p|k}^T]^T$ et les matrices $\mathbf{F}$ et $\mathbf{G}$ sont construites à partir de $A$, $B$, $C$.

### Forme QP standard

Le problème devient :

$$
\begin{aligned}
\min_{\mathbf{u}} \quad & \frac{1}{2} \mathbf{u}^T H \mathbf{u} + \mathbf{c}^T \mathbf{u} + \text{constante} \\
\text{s.t.} \quad & \mathbf{u}_{\min} \leq \mathbf{u} \leq \mathbf{u}_{\max} \\
& \mathbf{y}_{\min} \leq \mathbf{F} x_k + \mathbf{G} \mathbf{u} \leq \mathbf{y}_{\max} \\
& \Delta \mathbf{u}_{\min} \leq \Delta \mathbf{u} \leq \Delta \mathbf{u}_{\max}
\end{aligned}
$$

où :
- $H = \mathbf{G}^T \bar{Q} \mathbf{G} + \bar{R} + \bar{S}$ (matrice Hessienne)
- $\mathbf{c} = \mathbf{G}^T \bar{Q} (\mathbf{F} x_k - \mathbf{r})$ (vecteur gradient)
- $\mathbf{r}$ : vecteur de références
- $\bar{Q}$, $\bar{R}$, $\bar{S}$ : matrices de pondération bloquées

**Propriétés** :
- Si $Q \succeq 0$, $R \succ 0$, $S \succeq 0$, alors $H \succ 0$ et le problème est **convexe**
- Solution unique garantie
- Solveurs efficaces disponibles (OSQP, qpOASES, etc.)

## 4.2 Dynamic Matrix Control (DMC)

Le DMC est l'une des premières méthodes MPC développées dans l'industrie, particulièrement adaptée aux processus lents.

### 4.2.1 Principe de fonctionnement

Le DMC utilise un **modèle de réponse indicielle** :

$$
y_k = y_0 + \sum_{i=1}^{N} s_i \Delta u_{k-i}
$$

où $s_i$ sont les coefficients de réponse indicielle et $N$ est l'horizon du modèle.

**Avantages** :
- Pas besoin de modèle d'état
- Identification simple à partir de données
- Intuitif pour les opérateurs industriels

### 4.2.2 Algorithme détaillé

**Étape 1 : Prédiction**

Les prédictions de sortie s'écrivent :

$$
\mathbf{y} = \mathbf{y}_0 + \mathbf{S} \Delta \mathbf{u}
$$

où :
- $\mathbf{y}_0$ : réponse libre (sans nouvelles commandes)
- $\mathbf{S}$ : matrice dynamique (Toeplitz des coefficients $s_i$)
- $\Delta \mathbf{u} = [\Delta u_{k|k}, ..., \Delta u_{k+N_c-1|k}]^T$

**Étape 2 : Optimisation**

Minimiser :
$$
J = \|\mathbf{y} - \mathbf{r}\|_Q^2 + \|\Delta \mathbf{u}\|_R^2
$$

sous contraintes sur $\mathbf{y}$ et $\Delta \mathbf{u}$.

**Étape 3 : Application**

Appliquer $\Delta u_{k|k}$ (première variation de commande).

### 4.2.3 Exemples d'application

**Application typique** : Contrôle de niveau dans un réservoir

- Modèle : Réponse indicielle identifiée expérimentalement
- Horizon : $N_p = 20$, $N_c = 5$
- Contraintes : Niveau min/max, débit max
- Résultat : Suivi de référence avec limitation des variations de débit

## 4.3 Generalized Predictive Control (GPC)

Le GPC, développé par Clarke et ses collaborateurs, utilise un modèle CARIMA (Controlled AutoRegressive Integrated Moving Average).

### 4.3.1 Modèle CARIMA

Le modèle CARIMA s'écrit :

$$
A(z^{-1}) y_k = B(z^{-1}) u_{k-1} + \frac{C(z^{-1})}{\Delta} e_k
$$

où :
- $A(z^{-1})$, $B(z^{-1})$, $C(z^{-1})$ : polynômes en $z^{-1}$
- $\Delta = 1 - z^{-1}$ : opérateur de différence
- $e_k$ : bruit blanc

**Avantages** :
- Capture les intégrateurs naturellement
- Gère les perturbations constantes (rejet intégral)
- Forme standard en identification de systèmes

### 4.3.2 Équations de prédiction

En utilisant la théorie de Diophantine, on peut écrire les prédictions optimales :

$$
\hat{y}_{k+j|k} = G_j(z^{-1}) \Delta u_{k+j-1} + F_j(z^{-1}) y_k
$$

où $G_j$ et $F_j$ sont des polynômes calculés récursivement.

**Forme vectorielle** :
$$
\mathbf{\hat{y}} = \mathbf{G} \Delta \mathbf{u} + \mathbf{f}
$$

où $\mathbf{f}$ contient les termes dépendant des données passées.

### 4.3.3 Loi de commande

Le problème d'optimisation devient :

$$
\min_{\Delta \mathbf{u}} \|\mathbf{\hat{y}} - \mathbf{r}\|_Q^2 + \|\Delta \mathbf{u}\|_R^2
$$

Solution analytique (sans contraintes) :
$$
\Delta \mathbf{u} = (\mathbf{G}^T Q \mathbf{G} + R)^{-1} \mathbf{G}^T Q (\mathbf{r} - \mathbf{f})
$$

**Avec contraintes** : Résolution d'un QP.

## 4.4 Model Algorithmic Control (MAC)

Le **Model Algorithmic Control (MAC)** est une méthode MPC développée dans les années 1970, contemporaine du DMC. Elle utilise un modèle de réponse impulsionnelle et a été largement utilisée dans l'industrie, notamment en France.

### Principe

Le MAC repose sur un **modèle de réponse impulsionnelle** :

$$
y_k = \sum_{i=1}^{N} h_i u_{k-i}
$$

où :
- $h_i$ : coefficients de réponse impulsionnelle
- $N$ : horizon du modèle (nombre de coefficients)
- $u_{k-i}$ : commandes passées

**Caractéristiques** :
- Le modèle capture directement la réponse du système à une impulsion
- Pas besoin de modèle d'état explicite
- Identification simple à partir de données expérimentales

### Différences avec DMC

| Aspect | MAC | DMC |
|--------|-----|-----|
| **Modèle** | Réponse impulsionnelle | Réponse indicielle |
| **Variable** | $u_k$ (commande absolue) | $\Delta u_k$ (variation) |
| **Formulation** | Plus directe | Nécessite calcul des variations |
| **Intégrateurs** | Gérés via modèle | Gérés naturellement |

**Avantages du MAC** :
- Formulation plus directe (pas de calcul de variations)
- Identification simple
- Bon pour systèmes stables

**Inconvénients du MAC** :
- Ne gère pas naturellement les intégrateurs
- Peut nécessiter plus de coefficients pour systèmes lents
- Moins utilisé aujourd'hui que le DMC

### Algorithme MAC détaillé

**Étape 1 : Construction de la matrice de prédiction**

Les prédictions s'écrivent sous forme matricielle :

$$
\mathbf{y} = \mathbf{H} \mathbf{u} + \mathbf{y}_0
$$

où :
- $\mathbf{y} = [y_{k+1|k}, ..., y_{k+N_p|k}]^T$ : vecteur de prédictions
- $\mathbf{u} = [u_{k|k}, ..., u_{k+N_c-1|k}]^T$ : vecteur de commandes futures
- $\mathbf{H}$ : matrice de Toeplitz construite à partir des coefficients $h_i$
- $\mathbf{y}_0$ : réponse libre (prédiction sans nouvelles commandes)

**Construction de $\mathbf{H}$** :
$$
\mathbf{H} = \begin{bmatrix}
h_1 & 0 & \cdots & 0 \\
h_2 & h_1 & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
h_{N_p} & h_{N_p-1} & \cdots & h_{N_p-N_c+1}
\end{bmatrix}
$$

**Calcul de $\mathbf{y}_0$** :
$$
y_{0,k+i} = \sum_{j=i+1}^{N} h_j u_{k+i-j}
$$

**Étape 2 : Optimisation**

Le problème d'optimisation devient :

$$
\min_{\mathbf{u}} \quad J = \|\mathbf{y} - \mathbf{r}\|_Q^2 + \|\mathbf{u}\|_R^2
$$

où $\mathbf{r} = [r_{k+1}, ..., r_{k+N_p}]^T$ est le vecteur de références.

Sous forme QP :
$$
\min_{\mathbf{u}} \quad \frac{1}{2} \mathbf{u}^T H \mathbf{u} + \mathbf{c}^T \mathbf{u}
$$

où :
- $H = \mathbf{H}^T Q \mathbf{H} + R$
- $\mathbf{c} = \mathbf{H}^T Q (\mathbf{y}_0 - \mathbf{r})$

**Contraintes** :
- Sur les commandes : $u_{\min} \leq u_{k+i|k} \leq u_{\max}$
- Sur les sorties : $y_{\min} \leq y_{k+i|k} \leq y_{\max}$

**Étape 3 : Application**

Appliquer uniquement la première commande : $u_k = u_{k|k}^*$

### Identification du modèle

**Méthode expérimentale** :

1. **Test impulsionnel** : Appliquer une impulsion $u_0 = \delta$ à $t=0$
2. **Mesurer la réponse** : Enregistrer $y_k$ pour $k = 1, ..., N$
3. **Calculer les coefficients** : $h_i = y_i / \delta$ (normalisation)

**Méthode de corrélation** :
$$
h_i = \frac{R_{yu}(i)}{R_{uu}(0)}
$$

où $R_{yu}$ et $R_{uu}$ sont les fonctions de corrélation.

### Exemple d'application

**Système** : Contrôle de température d'un four

- **Modèle identifié** : $N = 30$ coefficients de réponse impulsionnelle
- **Horizons** : $N_p = 20$, $N_c = 5$
- **Contraintes** : $0 \leq u \leq 100\%$ (puissance), $180 \leq T \leq 220°C$
- **Résultat** : Suivi précis avec respect des contraintes

### Limitations et évolutions

**Limitations du MAC** :
- Modèle non-paramétrique (beaucoup de coefficients)
- Difficile à analyser théoriquement
- Moins flexible que les modèles d'état

**Évolutions** :
- Intégration dans des outils modernes
- Combinaison avec d'autres méthodes
- Remplacement progressif par MPC basée sur modèle d'état

## 4.5 Comparaison des différentes approches linéaires

| Aspect | DMC | GPC | MAC | MPC État |
|--------|-----|-----|-----|----------|
| **Modèle** | Réponse indicielle | CARIMA | Réponse impulsionnelle | État |
| **Variable** | $\Delta u$ | $\Delta u$ | $u$ | $u$ ou $\Delta u$ |
| **Identification** | Simple | Moyenne | Simple | Complexe |
| **Intégrateurs** | Gérés | Naturels | Gérés | Explicites |
| **Contraintes** | QP | QP | QP | QP |
| **Stabilité** | Analyse difficile | Analyse possible | Analyse difficile | Analyse standard |
| **Industrie** | Très répandu | Académique | Moins utilisé | Moderne |

### Choix de la méthode

**DMC** : 
- Processus industriels lents
- Identification simple souhaitée
- Opérateurs non-experts

**GPC** :
- Systèmes avec intégrateurs
- Analyse théorique importante
- Applications académiques

**MPC État** :
- Systèmes multivariables complexes
- Analyse de stabilité requise
- Applications modernes (véhicules, robots)

**Recommandation** : Pour les nouvelles applications, privilégier la MPC basée sur modèle d'état pour sa flexibilité et ses garanties théoriques.

---

**Points clés du chapitre** :
- La MPC linéaire se réduit à un problème QP
- DMC, GPC et MAC sont des méthodes historiques importantes
- La MPC basée sur modèle d'état est la plus générale et moderne
- Le choix de la méthode dépend de l'application et des contraintes
