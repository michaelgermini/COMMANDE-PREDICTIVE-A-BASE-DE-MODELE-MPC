# Chapitre 9 : Faisabilité et Récupération

Un problème fondamental en MPC est de garantir que le problème d'optimisation reste **faisable** à chaque instant, même en présence de perturbations importantes.

## 9.1 Faisabilité récursive

### Définition

La **faisabilité récursive** signifie que si le problème MPC est faisable à l'instant $k$, alors il reste faisable à l'instant $k+1$ (sous certaines conditions).

**Importance** : Sans faisabilité récursive, le solveur peut échouer, laissant le système sans commande.

### Conditions pour la faisabilité récursive

**Théorème** : Si :
1. Le problème est faisable à $k=0$
2. Les contraintes terminales sont satisfaites (ensemble terminal invariant)
3. Les perturbations sont bornées

Alors la faisabilité est récursive.

**Preuve intuitive** : Si à l'instant $k$, on a une solution faisable, alors à $k+1$, on peut utiliser la solution de $k$ (décalée d'un pas) comme solution faisable, au moins jusqu'à ce que les contraintes terminales garantissent une continuation.

### Problèmes de faisabilité

**Causes d'infaisabilité** :
1. **Perturbations importantes** : Le système est poussé hors du domaine faisable
2. **Contraintes trop strictes** : Les contraintes sont incompatibles
3. **Horizon trop court** : Pas assez de "marge" pour respecter les contraintes
4. **Erreurs de modèle** : Le modèle ne capture pas la vraie dynamique

**Conséquences** :
- Échec du solveur
- Système sans commande
- Risque d'instabilité ou de violation de contraintes critiques

## 9.2 Contraintes souples et variables de relâchement

### Principe

Les **contraintes souples** permettent des violations temporaires avec pénalisation, garantissant ainsi la faisabilité.

### Variables de relâchement

Au lieu de contraintes dures :
$$
y_{k+i|k} \leq y_{\max}
$$

on introduit des **variables de relâchement** $\epsilon_{k+i} \geq 0$ :
$$
y_{k+i|k} \leq y_{\max} + \epsilon_{k+i}
$$

et on pénalise les violations dans la fonction de coût :
$$
J = ... + \rho \sum_{i=1}^{N_p} \epsilon_{k+i}^2
$$

où $\rho > 0$ est un poids de pénalisation (typiquement très grand).

### Stratégies de pénalisation

**Pénalisation quadratique** :
$$
\rho \sum_i \epsilon_i^2
$$
- Pénale les grandes violations plus que les petites
- Lisse et différentiable

**Pénalisation linéaire** :
$$
\rho \sum_i \epsilon_i
$$
- Pénale proportionnellement
- Moins lisse mais peut être plus intuitif

**Pénalisation à seuil** :
$$
\rho \sum_i \max(0, \epsilon_i - \epsilon_0)^2
$$
- Permet de petites violations sans pénalisation
- Utile pour les contraintes "souples par nature"

### Choix du poids $\rho$

**Grand $\rho$** :
- Approche les contraintes dures
- Mais peut rendre le problème mal conditionné

**Petit $\rho$** :
- Permet plus facilement les violations
- Mais peut dégrader les performances

**Stratégie adaptative** : Ajuster $\rho$ dynamiquement selon la situation.

### Contraintes critiques vs non-critiques

**Contraintes critiques** (sécurité) :
- Toujours dures ou avec $\rho$ très grand
- Exemple : Limite de pression dans un réacteur

**Contraintes non-critiques** (confort, performance) :
- Peuvent être souples
- Exemple : Température de confort dans un bâtiment

## 9.3 Stratégies de récupération en cas d'infaisabilité

Même avec des contraintes souples, l'infaisabilité peut survenir. Il faut des stratégies de **récupération**.

### Stratégie 1 : Relâchement progressif

Si le problème est infaisable, relâcher progressivement les contraintes :

1. **Tentative initiale** : Résoudre avec toutes les contraintes
2. **Si échec** : Relâcher les contraintes non-critiques
3. **Si encore échec** : Relâcher davantage ou augmenter les variables de relâchement

**Algorithme** :
```
pour priorité = critique, importante, normale, faible:
    résoudre MPC avec contraintes de priorité >= priorité
    si faisable:
        retourner solution
sinon:
    utiliser solution de secours (ex: commande précédente)
```

### Stratégie 2 : Commande de secours

Si le problème est infaisable, utiliser une **commande de secours** :

**Options** :
1. **Commande précédente** : $u_k = u_{k-1}$
2. **Loi de commande de secours** : $u_k = \kappa_{backup}(x_k)$ (ex: PID, LQR)
3. **Commande sûre** : Commande qui garantit au moins la stabilité

**Exemple** : Pour un véhicule, en cas d'infaisabilité, utiliser une commande de freinage d'urgence.

### Stratégie 3 : Réduction de l'horizon

Réduire dynamiquement l'horizon $N_p$ si le problème est infaisable :

**Idée** : Un horizon plus court peut être plus facilement faisable.

**Algorithme** :
```
pour N = N_p, N_p-1, ..., N_min:
    résoudre MPC avec horizon N
    si faisable:
        retourner solution
```

**Risque** : Horizon trop court peut compromettre la stabilité.

### Stratégie 4 : Hiérarchisation des contraintes

Organiser les contraintes par **priorité** :

1. **Niveau 1 (critique)** : Sécurité, stabilité
2. **Niveau 2 (important)** : Performance, qualité
3. **Niveau 3 (souhaitable)** : Confort, économie

Résoudre séquentiellement en respectant d'abord les contraintes de niveau 1, puis 2, etc.

### Stratégie 5 : Mode dégradé

Passer en **mode dégradé** avec objectifs simplifiés :

**Exemple** : Pour un drone :
- **Mode normal** : Suivi de trajectoire optimale
- **Mode dégradé** : Stabilisation uniquement (ignorer la trajectoire)

## 9.4 Garanties de faisabilité

### Méthodes théoriques

**Ensemble de faisabilité** : L'ensemble $\mathcal{F}$ des états pour lesquels le problème MPC est faisable.

**Calcul de $\mathcal{F}$** :
- Pour systèmes linéaires : Peut être calculé comme un polytope
- Pour systèmes non-linéaires : Calcul complexe, souvent approché

**Garantie** : Si $x_k \in \mathcal{F}$, alors le problème est faisable.

### Contraintes terminales

Les **contraintes terminales** (voir Chapitre 7) garantissent la faisabilité récursive :

**Théorème** : Si :
- $x_{k+N_p|k} \in \mathcal{X}_f$ (ensemble terminal)
- $\mathcal{X}_f$ est un ensemble invariant

Alors la faisabilité est récursive pour $x_k \in \mathcal{F}$, où $\mathcal{F}$ est calculable.

### Domaines d'attraction

Le **domaine d'attraction** $\mathcal{A}$ est l'ensemble des états initiaux pour lesquels le système converge vers l'équilibre.

**Relation** : $\mathcal{A} \subseteq \mathcal{F}$ (le domaine d'attraction est inclus dans l'ensemble de faisabilité).

**Garantie pratique** : S'assurer que l'état opérationnel reste dans $\mathcal{F}$ via :
- Contraintes d'état supplémentaires
- Surveillance de l'état
- Actions préventives

### Monitoring en ligne

**Surveillance de faisabilité** :
- Vérifier à chaque pas si $x_k \in \mathcal{F}$
- Si proche de la frontière, prendre des actions préventives
- Avertir l'opérateur si nécessaire

**Indicateurs** :
- Distance à la frontière de $\mathcal{F}$
- Marge de faisabilité (slack variables)
- Nombre d'itérations du solveur

---

**Points clés du chapitre** :
- La faisabilité récursive est cruciale pour la robustesse de la MPC
- Les contraintes souples avec variables de relâchement garantissent la faisabilité
- Des stratégies de récupération sont nécessaires en cas d'infaisabilité
- Les contraintes terminales peuvent garantir la faisabilité récursive
- Le monitoring en ligne permet d'anticiper les problèmes de faisabilité
