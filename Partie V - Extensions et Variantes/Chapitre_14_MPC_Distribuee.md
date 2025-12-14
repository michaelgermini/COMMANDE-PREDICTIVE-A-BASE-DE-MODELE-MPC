# Chapitre 14 : MPC Distribuée et Décentralisée

Pour les systèmes à grande échelle (réseaux, systèmes interconnectés), une approche centralisée peut être impossible ou inefficace. La MPC distribuée et décentralisée offre des solutions.

## 14.1 Systèmes à grande échelle

### Défis des systèmes centralisés

Pour un système composé de $M$ sous-systèmes interconnectés :

**Problème centralisé** :
$$
\min_{\mathbf{u}_1, ..., \mathbf{u}_M} \sum_{j=1}^M J_j(\mathbf{x}_j, \mathbf{u}_j)
$$

sous contraintes couplées entre sous-systèmes.

**Problèmes** :
1. **Dimension** : Le problème a $M \times N_c \times m$ variables (très grand)
2. **Communication** : Toutes les données doivent être centralisées
3. **Robustesse** : Défaillance du centre = panne totale
4. **Scalabilité** : Ne s'adapte pas bien à l'ajout de sous-systèmes

### Exemples de systèmes à grande échelle

- **Réseaux électriques** : Multiples générateurs, charges, lignes
- **Systèmes de transport** : Multiples véhicules, intersections
- **Réseaux d'eau** : Multiples réservoirs, stations de pompage
- **Flottes de véhicules** : Multiples drones, robots
- **Bâtiments** : Multiples zones, systèmes HVAC

## 14.2 Décomposition du problème

### Principe

Décomposer le problème global en **sous-problèmes locaux** résolus par chaque sous-système.

**Structure** :
- Chaque sous-système $j$ résout son propre problème MPC
- Les sous-systèmes communiquent pour coordonner
- La solution globale émerge de la coordination

### Types de couplage

**Couplage d'état** :
$$
x_j^{k+1} = f_j(x_j^k, u_j^k, x_{-j}^k)
$$
où $x_{-j}$ représente les états des autres sous-systèmes.

**Couplage de contraintes** :
$$
g(x_1, ..., x_M, u_1, ..., u_M) \leq 0
$$
Contraintes globales impliquant plusieurs sous-systèmes.

**Couplage d'objectif** :
$$
J = \sum_j J_j + J_{coupling}(x_1, ..., x_M)
$$
Objectif global avec termes de couplage.

## 14.3 MPC décentralisée

### Principe

Dans la **MPC décentralisée**, chaque sous-système résout son problème **indépendamment**, sans communication.

**Formulation pour sous-système $j$** :
$$
\begin{aligned}
\min_{\mathbf{u}_j} \quad & J_j(\mathbf{x}_j, \mathbf{u}_j) \\
\text{s.t.} \quad & x_j^{k+1} = f_j(x_j^k, u_j^k, \hat{x}_{-j}^k) \\
& g_j(x_j^k, u_j^k) \leq 0
\end{aligned}
$$

où $\hat{x}_{-j}$ est une **estimation** des états des autres sous-systèmes (souvent la dernière valeur connue).

### Avantages

- **Simplicité** : Pas de communication nécessaire
- **Robustesse** : Défaillance d'un sous-système n'affecte pas les autres
- **Scalabilité** : Ajout facile de nouveaux sous-systèmes

### Inconvénients

- **Performance sous-optimale** : Pas de coordination, solutions locales
- **Stabilité** : Pas de garanties globales
- **Couplages négligés** : Les interactions peuvent causer des problèmes

### Applications

- Systèmes faiblement couplés
- Systèmes où la communication est impossible/coûteuse
- Applications où la performance globale n'est pas critique

## 14.4 MPC distribuée

### Principe

La **MPC distribuée** permet la **communication** entre sous-systèmes pour coordonner les décisions.

### 14.4.1 Architectures de communication

**Architecture centralisée** :
```
Sous-systèmes ←→ Coordinateur central ←→ Sous-systèmes
```
- Un coordinateur central collecte les informations
- Résout un problème de coordination
- Distribue les solutions

**Architecture décentralisée (pair-à-pair)** :
```
Sous-système 1 ←→ Sous-système 2
     ↕                ↕
Sous-système 3 ←→ Sous-système 4
```
- Communication directe entre sous-systèmes
- Pas de coordinateur central
- Plus robuste mais plus complexe

**Architecture hiérarchique** :
```
        Coordinateur
       /    |    \
   Zone 1  Zone 2  Zone 3
    /|\     /|\     /|\
  SS1 SS2 SS3 SS4 SS5 SS6
```
- Structure à plusieurs niveaux
- Coordination par zones puis globale

### 14.4.2 Coordination des sous-systèmes

#### Méthode 1 : Dual Decomposition

**Principe** : Utiliser la décomposition lagrangienne.

**Problème global** :
$$
\min \sum_j J_j \text{ s.t. } \sum_j A_j x_j = b
$$

**Dual** :
$$
\max_{\lambda} \min_{\mathbf{x}} \sum_j J_j + \lambda^T (\sum_j A_j x_j - b)
$$

**Algorithme distribué** :
1. Chaque sous-système résout : $\min_{x_j} J_j + \lambda^T A_j x_j$
2. Mise à jour de $\lambda$ : $\lambda^{k+1} = \lambda^k + \alpha (\sum_j A_j x_j - b)$
3. Répéter jusqu'à convergence

**Convergence** : Sous certaines conditions (convexité, pas adaptatif).

#### Méthode 2 : Consensus

**Principe** : Les sous-systèmes convergent vers un consensus sur les variables partagées.

**Algorithme** :
1. Chaque sous-système $j$ propose une valeur $z_j$ pour les variables partagées
2. Communication avec voisins
3. Mise à jour : $z_j^{k+1} = \sum_{i \in \mathcal{N}_j} w_{ij} z_i^k$
4. Résolution du problème local avec $z_j$ mis à jour

où $\mathcal{N}_j$ sont les voisins de $j$ et $w_{ij}$ sont des poids de consensus.

#### Méthode 3 : Alternating Direction Method of Multipliers (ADMM)

**Principe** : Combinaison de décomposition et méthode des multiplicateurs.

**Formulation** :
$$
\min \sum_j f_j(x_j) \text{ s.t. } x_j = z, \quad j = 1, ..., M
$$

**Algorithme ADMM** :
1. **Mise à jour locale** : $x_j^{k+1} = \arg\min_{x_j} f_j(x_j) + \rho/2 \|x_j - z^k + u_j^k\|^2$
2. **Mise à jour globale** : $z^{k+1} = \frac{1}{M} \sum_j (x_j^{k+1} + u_j^k)$
3. **Mise à jour duale** : $u_j^{k+1} = u_j^k + x_j^{k+1} - z^{k+1}$

**Avantages** :
- Convergence garantie (sous convexité)
- Robuste aux erreurs de communication
- Bonnes performances pratiques

### Exemple : Réseau électrique

**Système** : $M$ générateurs interconnectés

**Problème local pour générateur $j$** :
$$
\min_{P_j} c_j(P_j) \text{ s.t. } P_{j,min} \leq P_j \leq P_{j,max}
$$

**Contrainte globale** : $\sum_j P_j = P_{demand}$ (équilibre offre/demande)

**Coordination ADMM** :
1. Chaque générateur optimise localement
2. Communication de la production proposée
3. Calcul de la moyenne globale
4. Ajustement pour équilibrer offre/demande
5. Répétition jusqu'à convergence

## 14.5 MPC hiérarchique

### Principe

La **MPC hiérarchique** organise le contrôle en **niveaux** :

- **Niveau supérieur** : Planification à long terme, objectifs stratégiques
- **Niveau intermédiaire** : Coordination à moyen terme
- **Niveau inférieur** : Contrôle local à court terme

### Exemple : Usine de production

**Niveau 1 (Supérieur)** :
- Planification de production mensuelle
- Objectifs de production, contraintes de marché
- Horizon : 1 mois, période : 1 jour

**Niveau 2 (Intermédiaire)** :
- Planification hebdomadaire
- Répartition entre lignes de production
- Horizon : 1 semaine, période : 1 heure

**Niveau 3 (Inférieur)** :
- Contrôle en temps réel
- MPC locale pour chaque ligne
- Horizon : 1 heure, période : 1 minute

**Communication** :
- Niveau supérieur → Niveau inférieur : Références, objectifs
- Niveau inférieur → Niveau supérieur : État, contraintes

### Avantages

- **Séparation des échelles de temps** : Chaque niveau gère son horizon
- **Réduction de complexité** : Problèmes plus petits à chaque niveau
- **Flexibilité** : Adaptation à différents objectifs

### Défis

- **Cohérence** : Garantir la cohérence entre niveaux
- **Stabilité** : Analyser la stabilité globale
- **Communication** : Gérer les délais et erreurs de communication

## 14.6 Applications aux réseaux

### Réseaux électriques (Smart Grids)

**Structure** :
- Sous-systèmes : Générateurs, charges, stockage
- Couplage : Équilibre offre/demande, contraintes de réseau
- MPC distribuée : Coordination décentralisée

**Bénéfices** :
- Gestion efficace de la demande
- Intégration des énergies renouvelables
- Réduction des coûts

### Réseaux de transport

**Structure** :
- Sous-systèmes : Véhicules, intersections
- Couplage : Éviter collisions, optimiser flux
- MPC distribuée : Coordination entre véhicules

**Bénéfices** :
- Réduction de la congestion
- Optimisation du trafic
- Sécurité améliorée

### Réseaux d'eau

**Structure** :
- Sous-systèmes : Réservoirs, stations de pompage
- Couplage : Équilibre hydraulique, contraintes de pression
- MPC hiérarchique : Planification + contrôle local

**Bénéfices** :
- Optimisation de la consommation énergétique
- Gestion de la qualité de l'eau
- Réduction des coûts opérationnels

---

**Points clés du chapitre** :
- Les systèmes à grande échelle nécessitent des approches distribuées
- La MPC décentralisée est simple mais sous-optimale
- La MPC distribuée coordonne via communication
- La MPC hiérarchique organise le contrôle en niveaux
- Les applications aux réseaux montrent l'efficacité de ces approches
