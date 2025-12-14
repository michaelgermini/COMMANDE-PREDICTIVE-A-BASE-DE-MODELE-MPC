# Chapitre 15 : MPC et Apprentissage Automatique

La convergence de la MPC avec l'apprentissage automatique ouvre de nouvelles possibilités pour améliorer les performances et gérer l'incertitude.

## 15.1 Learning-based MPC

### Motivation

Les limitations traditionnelles de la MPC :
- **Modèle nécessaire** : Besoin d'un modèle précis
- **Incertitudes** : Difficulté à gérer les incertitudes complexes
- **Adaptation** : Modèle fixe, pas d'adaptation aux changements

L'apprentissage automatique peut aider à :
- **Apprendre le modèle** à partir de données
- **Améliorer les prédictions** avec des modèles non-paramétriques
- **Adapter en ligne** aux changements du système

### Approches principales

1. **Modèles de prédiction par apprentissage** : Remplacer ou compléter le modèle physique
2. **Approximation de la loi de commande** : Apprendre directement la fonction de commande
3. **Apprentissage des incertitudes** : Modéliser les incertitudes avec des méthodes probabilistes
4. **Reinforcement Learning** : Combiner MPC et RL

## 15.2 MPC avec réseaux de neurones

### 15.2.1 Modèles de prédiction par NN

Les **réseaux de neurones (NN)** peuvent être utilisés comme modèles de prédiction dans la MPC.

**Architecture** :
$$
\hat{x}_{k+1} = \text{NN}(x_k, u_k; \theta)
$$

où $\theta$ sont les paramètres du réseau (poids et biais).

**Types de réseaux** :

1. **Feedforward NN** :
   - Architecture simple
   - $x_{k+1} = \sigma(W_2 \sigma(W_1 [x_k; u_k] + b_1) + b_2)$
   - Adapté aux systèmes sans mémoire longue

2. **RNN/LSTM** :
   - Capture la mémoire à long terme
   - Adapté aux systèmes avec dynamiques complexes
   - Plus complexe à entraîner

3. **Neural ODE** :
   - Modélise la dérivée : $\dot{x} = \text{NN}(x, u)$
   - Intègre avec solveur ODE
   - Plus physique, meilleure généralisation

**Entraînement** :
- **Données** : Collecter $(x_k, u_k, x_{k+1})$ du système réel
- **Perte** : $L = \sum_k \|\hat{x}_{k+1} - x_{k+1}\|^2$
- **Optimisation** : Backpropagation, Adam, etc.

**Avantages** :
- Pas besoin de modèle physique
- Capture des non-linéarités complexes
- Adaptation possible

**Inconvénients** :
- Besoin de beaucoup de données
- Pas de garanties théoriques
- Risque de sur-apprentissage
- Difficile à interpréter

### 15.2.2 Approximation de la loi de commande

Au lieu d'optimiser en ligne, on peut **apprendre** directement la fonction de commande :

$$
u_k = \text{NN}(x_k, r_k; \theta)
$$

**Entraînement** :
1. Générer des données : Résoudre la MPC pour différents $(x_k, r_k)$
2. Entraîner le NN pour approximer $u_k^* = \text{MPC}(x_k, r_k)$
3. Utiliser le NN comme contrôleur (plus rapide)

**Avantages** :
- **Temps de calcul** : Évaluation du NN très rapide
- **Déterminisme** : Pas d'optimisation, temps garanti
- **Implémentation simple** : Juste une évaluation de fonction

**Inconvénients** :
- **Précision** : Approximation, erreurs possibles
- **Généralisation** : Performance sur données non vues
- **Contraintes** : Difficile de garantir le respect des contraintes

**Applications** :
- Remplacement de l'Explicit MPC pour systèmes complexes
- Contrôleurs embarqués avec ressources limitées

## 15.3 Reinforcement Learning et MPC

### Principe

Le **Reinforcement Learning (RL)** apprend une politique optimale par interaction avec l'environnement. La MPC peut être vue comme une forme de RL avec modèle.

### MPC comme RL

**Correspondance** :
- **État** : $x_k$ (état du système)
- **Action** : $u_k$ (commande)
- **Récompense** : $r_k = -\ell(x_k, u_k)$ (négatif du coût)
- **Modèle** : $f(x_k, u_k)$ (dynamique)
- **Politique** : $\pi(x_k) = u_k^*$ (solution MPC)

**Différence clé** : La MPC utilise un **modèle explicite**, le RL classique apprend sans modèle.

### Approches hybrides

#### 1. MPC avec modèle appris

**Principe** : Apprendre le modèle avec RL, puis utiliser MPC.

**Algorithme** :
1. **Apprentissage du modèle** : Apprendre $f$ à partir de données
2. **MPC** : Utiliser le modèle appris dans la MPC
3. **Mise à jour** : Améliorer le modèle avec nouvelles données

**Avantages** :
- Combine apprentissage et garanties MPC
- Améliore le modèle en ligne

#### 2. RL guidé par MPC

**Principe** : Utiliser la MPC pour guider l'exploration en RL.

**Algorithme** :
- **Exploration** : Utiliser la MPC pour suggérer des actions prometteuses
- **Apprentissage** : RL apprend la politique optimale
- **Exploitation** : Utiliser la politique apprise (plus rapide que MPC)

#### 3. MPC avec coût appris

**Principe** : Apprendre la fonction de coût $\ell$ avec RL.

**Motivation** : La fonction de coût peut être difficile à spécifier manuellement.

**Algorithme** :
1. **Apprentissage** : Apprendre $\ell$ à partir de démonstrations ou préférences
2. **MPC** : Utiliser $\ell$ appris dans la MPC

## 15.4 Gaussian Process MPC

### Principe

Les **Gaussian Processes (GP)** modélisent les incertitudes de manière probabiliste, idéal pour la MPC robuste.

### Modèle GP

Un GP modélise une fonction inconnue $f$ comme une distribution de probabilité :

$$
f(x) \sim \mathcal{GP}(m(x), k(x, x'))
$$

où :
- $m(x)$ : moyenne a priori
- $k(x, x')$ : fonction de covariance (kernel)

**Prédiction** : Pour de nouvelles données, le GP donne une **distribution** (moyenne + variance) plutôt qu'une valeur unique.

### GP-MPC

**Formulation** :
$$
\begin{aligned}
\min_{\mathbf{u}} \quad & \mathbb{E}[J(\mathbf{x}, \mathbf{u})] + \beta \text{Var}[J(\mathbf{x}, \mathbf{u})] \\
\text{s.t.} \quad & x_{k+1} \sim \mathcal{GP}(m(x_k, u_k), k(x_k, u_k, x_k, u_k)) \\
& \mathbb{P}(g(x_k, u_k) \leq 0) \geq 1 - \epsilon
\end{aligned}
$$

**Interprétation** :
- Optimise l'espérance du coût
- Pénalise la variance (robustesse)
- Contraintes probabilistes

### Avantages

1. **Gestion d'incertitude** : Modélise explicitement l'incertitude
2. **Apprentissage** : Améliore avec nouvelles données
3. **Robustesse** : Contraintes probabilistes naturelles
4. **Efficacité données** : Bonne généralisation avec peu de données

### Défis

1. **Complexité** : Coût computationnel élevé ($O(n^3)$ pour $n$ données)
2. **Dimension** : Performance dégrade avec dimension élevée
3. **Kernel** : Choix du kernel crucial

### Applications

- Systèmes avec incertitudes importantes
- Apprentissage en ligne
- Applications où la robustesse est critique

## 15.5 Data-driven MPC

### Principe

La **Data-driven MPC** construit le contrôleur directement à partir de **données**, sans modèle explicite.

### Méthodes principales

#### 1. MPC avec modèles non-paramétriques

Utiliser des méthodes non-paramétriques (k-NN, régression locale) pour prédire :

$$
\hat{x}_{k+1} = \text{LocalRegression}(\{(x_i, u_i, x_{i+1})\}_{i=1}^N, (x_k, u_k))
$$

**Avantages** : Pas besoin de structure de modèle
**Inconvénients** : Besoin de beaucoup de données, pas de garanties

#### 2. MPC avec modèles de sous-espaces

Les **méthodes de sous-espaces** identifient directement les matrices d'état à partir de données :

**Algorithme** :
1. Collecter données entrée-sortie
2. Construire matrices de Hankel
3. Décomposition SVD pour identifier $(A, B, C)$
4. Utiliser dans MPC standard

**Avantages** : Modèle d'état directement utilisable
**Inconvénients** : Besoin de données riches, dimension à choisir

#### 3. MPC directe (sans modèle)

**Principe** : Optimiser directement sur les données passées.

**Formulation** :
$$
\min_{\mathbf{u}} \sum_{i=1}^N w_i \ell(y_i^{pred}, u_i^{pred})
$$

où les prédictions sont basées sur des trajectoires similaires dans les données historiques.

**Avantages** : Pas de modèle nécessaire
**Inconvénients** : Complexité, garanties limitées

### Défis

1. **Données** : Besoin de données représentatives et en quantité suffisante
2. **Généralisation** : Performance sur situations non vues
3. **Garanties** : Stabilité et robustesse difficiles à garantir
4. **Interprétabilité** : Modèles "boîte noire"

### Applications prometteuses

- Systèmes complexes difficiles à modéliser
- Systèmes avec beaucoup de données disponibles
- Applications où l'apprentissage continu est possible

---

**Points clés du chapitre** :
- L'apprentissage automatique complète la MPC traditionnelle
- Les réseaux de neurones peuvent remplacer ou améliorer les modèles
- Le RL et la MPC sont complémentaires
- Les Gaussian Processes gèrent naturellement l'incertitude
- La data-driven MPC évite le besoin de modèles explicites
- Les approches hybrides combinent le meilleur des deux mondes
