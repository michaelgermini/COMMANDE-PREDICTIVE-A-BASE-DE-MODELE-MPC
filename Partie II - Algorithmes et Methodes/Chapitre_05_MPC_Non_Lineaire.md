# Chapitre 5 : MPC Non-Linéaire (NMPC)

Pour les systèmes présentant des non-linéarités significatives, la MPC linéaire peut être insuffisante. La MPC non-linéaire (NMPC) traite directement les modèles non-linéaires, au prix d'une complexité computationnelle accrue.

## 5.1 Nécessité de l'approche non-linéaire

### Limitations de la MPC linéaire

La MPC linéaire peut échouer lorsque :

1. **Non-linéarités fortes** : Le système opère loin du point de linéarisation
2. **Grandes variations** : Les trajectoires couvrent une large plage de fonctionnement
3. **Comportements non-linéaires essentiels** : Saturation, hystérésis, couplages non-linéaires
4. **Performances optimales requises** : La linéarisation introduit des erreurs inacceptables

### Exemples de systèmes nécessitant NMPC

- **Réacteurs chimiques** : Cinétiques non-linéaires, variations de température importantes
- **Véhicules** : Dynamique non-linéaire aux grandes vitesses, adhérence variable
- **Robots** : Couplages non-linéaires, singularités
- **Systèmes biologiques** : Modèles intrinsèquement non-linéaires

## 5.2 Formulation du problème NLP

Pour un système non-linéaire :

$$
\begin{cases}
x_{k+1} = f(x_k, u_k) \\
y_k = h(x_k, u_k)
\end{cases}
$$

avec $f$ et $h$ non-linéaires, le problème MPC devient un **Programme Non-Linéaire (NLP)** :

$$
\begin{aligned}
\min_{\mathbf{u}, \mathbf{x}} \quad & J(\mathbf{x}, \mathbf{u}) = \sum_{i=1}^{N_p} \ell(x_{k+i|k}, u_{k+i|k}, r_{k+i}) \\
\text{s.t.} \quad & x_{k+i+1|k} = f(x_{k+i|k}, u_{k+i|k}), \quad i = 0, ..., N_p-1 \\
& g(x_{k+i|k}, u_{k+i|k}) \leq 0, \quad i = 0, ..., N_p \\
& x_{k|k} = x_k
\end{aligned}
$$

où $\ell$ est le coût instantané (généralement non-linéaire).

### Défis du NLP

1. **Non-convexité** : Multiples minima locaux possibles
2. **Complexité** : Temps de calcul élevé
3. **Convergence** : Pas de garantie de convergence globale
4. **Temps réel** : Difficulté à respecter les contraintes temporelles

## 5.3 Méthodes de résolution

### 5.3.1 Programmation quadratique séquentielle (SQP)

La méthode SQP résout une séquence de problèmes QP qui approchent le NLP original.

**Algorithme SQP** :

1. **Linéarisation** : Linéariser les contraintes autour de la solution courante
2. **QP approximé** : Résoudre un QP local
3. **Recherche linéaire** : Déterminer un pas le long de la direction obtenue
4. **Itération** : Répéter jusqu'à convergence

**Avantages** :
- Efficace pour les problèmes bien conditionnés
- Exploite la structure QP
- Bonne convergence locale

**Inconvénients** :
- Convergence locale seulement
- Sensible aux mauvaises initialisations

### 5.3.2 Méthodes de points intérieurs

Les méthodes de points intérieurs transforment le problème contraint en une séquence de problèmes non-contraints via des fonctions barrière.

**Principe** :
$$
\min_{\mathbf{u}} J(\mathbf{u}) + \mu \sum_i \log(-g_i(\mathbf{u}))
$$

où $\mu > 0$ est un paramètre de pénalisation qui décroît vers 0.

**Avantages** :
- Traitement uniforme des contraintes
- Bonnes propriétés de convergence
- Efficace pour les problèmes de grande taille

**Inconvénients** :
- Nécessite des contraintes strictement satisfaites initialement
- Paramétrage délicat

### 5.3.3 Méthodes de tir (shooting methods)

Les méthodes de tir transforment le problème d'optimisation en un problème de conditions aux limites.

**Principe** :
- Variables de décision : seulement les commandes $\mathbf{u}$
- États : calculés par intégration du modèle
- Contraintes : évaluées le long de la trajectoire

**Formulation réduite** :
$$
\min_{\mathbf{u}} J(\mathbf{u}, \mathbf{x}(\mathbf{u}))
$$

où $\mathbf{x}(\mathbf{u})$ est obtenu par intégration.

**Avantages** :
- Réduction de la dimension (moins de variables)
- Structure exploitable

**Inconvénients** :
- Intégration numérique nécessaire
- Sensibilité aux erreurs d'intégration

### 5.3.4 Collocation directe

La collocation directe discrétise simultanément la dynamique et l'optimisation.

**Principe** :
- Discrétiser le temps en points de collocation
- Imposer la dynamique aux points de collocation
- Résoudre le problème algébrique résultant

**Formulation** :
$$
\begin{aligned}
\min \quad & \sum_{i=0}^{N} w_i \ell(x_i, u_i) \\
\text{s.t.} \quad & \sum_{j=0}^{N} D_{ij} x_j = f(x_i, u_i), \quad i = 0, ..., N
\end{aligned}
$$

où $D_{ij}$ sont les coefficients de différenciation.

**Avantages** :
- Pas d'intégration nécessaire
- Bonne précision
- Structure exploitable

**Inconvénients** :
- Problème de grande dimension
- Matrices denses

## 5.4 Linéarisation successive

Une approche pragmatique consiste à linéariser le modèle à chaque pas de temps et résoudre un QP.

**Algorithme** :

1. **Linéarisation** : Autour de $(x_k, u_{k-1})$
   $$
   \delta x_{k+1} = A_k \delta x_k + B_k \delta u_k
   $$
   où $A_k = \frac{\partial f}{\partial x}|_{x_k, u_{k-1}}$, $B_k = \frac{\partial f}{\partial u}|_{x_k, u_{k-1}}$

2. **QP linéarisé** : Résoudre le problème QP avec le modèle linéarisé

3. **Application** : Appliquer la commande obtenue

**Avantages** :
- Simplicité d'implémentation
- Temps de calcul réduit (QP au lieu de NLP)
- Bonne performance si les non-linéarités sont modérées

**Inconvénients** :
- Erreurs de linéarisation
- Pas de garanties théoriques fortes
- Peut nécessiter plusieurs itérations par pas de temps

## 5.5 Défis computationnels et solutions

### Défis

1. **Temps de calcul** : Les NLP sont beaucoup plus lents que les QP
2. **Convergence** : Pas de garantie de convergence en temps limité
3. **Initialisation** : Besoin d'une bonne initialisation
4. **Robustesse numérique** : Sensibilité aux erreurs d'arrondi

### Solutions

**Warm starting** : Utiliser la solution de l'instant précédent comme point de départ
$$
\mathbf{u}^{(0)}_k = [u_{k-1|k-1}^*, u_{k|k-1}^*, ..., u_{k+N_c-2|k-1}^*, u_{k+N_c-1|k-1}^*]
$$

**Early termination** : Accepter une solution sous-optimale si le temps alloué est dépassé

**Réduction de l'horizon** : Utiliser un horizon plus court pour réduire la dimension

**Approximation** : Utiliser des modèles simplifiés pour l'optimisation, modèle détaillé pour la simulation

**Parallélisation** : Exploiter le parallélisme (GPU, calcul distribué)

**Explicit NMPC** : Pré-calculer la loi de commande (voir Chapitre 11)

### Heuristiques pratiques

1. **Horizon adaptatif** : Commencer avec un horizon court, l'augmenter si nécessaire
2. **Tolérances adaptatives** : Relâcher les tolérances si le temps presse
3. **Modèles hiérarchiques** : Modèle simple pour optimisation, modèle détaillé pour validation

---

**Points clés du chapitre** :
- La NMPC est nécessaire pour les systèmes fortement non-linéaires
- Le problème devient un NLP, beaucoup plus complexe qu'un QP
- Plusieurs méthodes de résolution existent (SQP, points intérieurs, tir, collocation)
- La linéarisation successive est une approche pragmatique efficace
- Les défis computationnels nécessitent des stratégies adaptatives
