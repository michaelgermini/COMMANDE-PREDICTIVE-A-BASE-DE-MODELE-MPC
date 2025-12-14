# Chapitre 11 : Implémentation Temps Réel

L'implémentation en temps réel de la MPC pose des défis spécifiques liés aux contraintes temporelles et aux ressources computationnelles limitées.

## 11.1 Contraintes temporelles

### Définition

En temps réel, le contrôleur doit :
1. **Acquérir** les mesures
2. **Résoudre** le problème d'optimisation
3. **Appliquer** la commande

Tout cela dans un **temps limité** $T_{max}$, typiquement égal à la période d'échantillonnage $T_s$.

### Contraintes typiques

**Systèmes rapides** :
- $T_s = 1$ à $10$ ms
- Exemple : Contrôle de moteur, robotique

**Systèmes moyens** :
- $T_s = 10$ à $100$ ms
- Exemple : Contrôle de processus, véhicules

**Systèmes lents** :
- $T_s = 100$ ms à plusieurs secondes
- Exemple : Contrôle de température, niveau

### Gestion du temps

**Stratégies** :
1. **Early termination** : Arrêter l'optimisation si le temps presse
2. **Horizon adaptatif** : Réduire $N_p$ si nécessaire
3. **Priorisation** : Résoudre d'abord les aspects critiques
4. **Parallélisation** : Utiliser plusieurs cœurs

## 11.2 Architectures matérielles

### 11.2.1 Microcontrôleurs

**Caractéristiques** :
- Faible coût, faible consommation
- Puissance limitée (MHz, Ko de RAM)
- Idéal pour systèmes simples, lents

**Applications** :
- Contrôle de petits systèmes
- MPC avec horizon très court ($N_p \leq 5$)
- Systèmes embarqués bas de gamme

**Limitations** :
- Pas de solveurs complexes
- Nécessite MPC explicite ou très simplifiée
- Précision limitée (arithmétique flottante)

**Exemples** : ARM Cortex-M, PIC, AVR

### 11.2.2 DSP (Digital Signal Processors)

**Caractéristiques** :
- Optimisés pour calculs numériques
- Puissance modérée (GHz, Mo de RAM)
- Bon compromis performance/coût

**Applications** :
- Contrôle de moteurs
- Systèmes audio/vidéo
- MPC linéaire avec horizons moyens

**Avantages** :
- Instructions spécialisées (MAC, FFT)
- Bonne efficacité énergétique
- Support matériel pour arithmétique flottante

**Exemples** : TI C2000, ADI SHARC

### 11.2.3 FPGA (Field-Programmable Gate Arrays)

**Caractéristiques** :
- Reprogrammable, parallélisme massif
- Très haute performance pour algorithmes spécialisés
- Coût et complexité de développement élevés

**Applications** :
- Systèmes ultra-rapides ($T_s < 1$ ms)
- MPC avec parallélisation poussée
- Applications critiques (aéronautique, spatial)

**Avantages** :
- Parallélisation complète
- Déterminisme temporel garanti
- Pas de système d'exploitation (pas de latence)

**Inconvénients** :
- Développement complexe
- Coût élevé
- Moins flexible que CPU

**Exemples** : Xilinx Zynq, Intel/Altera Cyclone

### 11.2.4 GPU (Graphics Processing Units)

**Caractéristiques** :
- Parallélisme massif (milliers de cœurs)
- Très haute performance pour calculs parallèles
- Consommation élevée

**Applications** :
- MPC pour systèmes complexes
- NMPC avec nombreux scénarios
- Simulation en temps réel

**Avantages** :
- Parallélisation efficace
- Bon rapport performance/coût
- Facile à programmer (CUDA, OpenCL)

**Inconvénients** :
- Latence de transfert mémoire
- Moins adapté aux problèmes séquentiels
- Consommation énergétique

**Exemples** : NVIDIA Jetson, GPU embarqués

### Comparaison

| Architecture | Performance | Coût | Complexité | Applications |
|--------------|-------------|------|------------|--------------|
| Microcontrôleur | Faible | Très faible | Faible | Systèmes simples |
| DSP | Modérée | Faible | Modérée | Contrôle moteurs |
| FPGA | Très élevée | Élevé | Très élevée | Temps réel critique |
| GPU | Élevée | Modéré | Modérée | Calculs parallèles |
| CPU | Modérée-Élevée | Modéré | Faible | Applications générales |

## 11.3 Explicit MPC

L'**Explicit MPC** pré-calcule la loi de commande, évitant ainsi l'optimisation en ligne.

### 11.3.1 Programmation multiparamétrique

Pour un système linéaire, le problème QP peut être résolu **analytiquement** en fonction de l'état $x_k$ :

$$
u_k^* = \kappa(x_k)
$$

où $\kappa$ est une **fonction affine par morceaux** (piecewise affine, PWA).

**Principe** :
- L'espace d'état est partitionné en **régions** $\mathcal{R}_i$
- Dans chaque région : $u_k = K_i x_k + g_i$
- Les régions et les gains sont **pré-calculés**

### Formulation

Le problème QP :
$$
\begin{aligned}
\min_{\mathbf{u}} \quad & \frac{1}{2} \mathbf{u}^T H \mathbf{u} + \mathbf{c}^T \mathbf{u} \\
\text{s.t.} \quad & G \mathbf{u} \leq W + S x_k
\end{aligned}
$$

a une solution de la forme :
$$
u_k^* = 
\begin{cases}
K_1 x_k + g_1 & \text{si } x_k \in \mathcal{R}_1 \\
K_2 x_k + g_2 & \text{si } x_k \in \mathcal{R}_2 \\
\vdots
\end{cases}
$$

### 11.3.2 Génération de look-up tables

**Étape 1** : Résoudre le problème multiparamétrique
- Utiliser des outils (MPT Toolbox, MPT3)
- Obtenir les régions $\mathcal{R}_i$ et les gains $K_i$, $g_i$

**Étape 2** : Implémenter la recherche
- Pour un $x_k$ donné, identifier la région $\mathcal{R}_i$ contenant $x_k$
- Appliquer $u_k = K_i x_k + g_i$

**Méthodes de recherche** :
1. **Recherche exhaustive** : Tester toutes les régions (simple mais lent)
2. **Arbre de décision** : Structure hiérarchique (rapide)
3. **Hachage** : Table de hachage (très rapide)

### Avantages et limitations

**Avantages** :
- **Temps de calcul garanti** : Pas d'optimisation, juste une recherche
- **Déterminisme** : Pas d'itérations, pas d'incertitude
- **Faible consommation** : Pas de solveur lourd

**Limitations** :
- **Complexité exponentielle** : Nombre de régions croît exponentiellement avec la dimension
- **Mémoire** : Stockage des régions et gains
- **Applicable seulement aux systèmes linéaires** (ou linéarisés)

**Règle empirique** : Explicit MPC pratique pour :
- Dimension d'état $\leq 4$
- Nombre de contraintes modéré
- Systèmes linéaires

### Exemple

Pour un système 2D avec 2 contraintes :
- Nombre de régions typique : 10-50
- Temps de calcul : < 1 ms
- Mémoire : < 10 Ko

Pour un système 6D avec 20 contraintes :
- Nombre de régions : > 1000
- Temps de calcul : 1-10 ms
- Mémoire : > 100 Ko

## 11.4 Génération automatique de code

Les outils modernes permettent de **générer automatiquement** le code C/C++ à partir d'une spécification MPC.

### Outils disponibles

**MATLAB/Simulink** :
- **MPC Toolbox** : Génération de code pour contrôleurs MPC
- **Embedded Coder** : Génération de code optimisé
- **Code Generation** : Export vers C/C++

**Python** :
- **ACADOS** : Génération de code C pour solveurs MPC
- **do-mpc** : Framework avec génération de code
- **CasADi** : Génération de code pour NMPC

**Workflow typique** :

1. **Développement** : Modélisation et réglage en MATLAB/Python
2. **Validation** : Simulation et tests
3. **Génération** : Export automatique vers C/C++
4. **Intégration** : Compilation et déploiement sur cible

### Avantages

**Rapidité de développement** :
- Pas besoin de réimplémenter l'optimisation
- Code testé et validé
- Maintenance facilitée

**Optimisation** :
- Code optimisé pour la cible
- Exploitation des spécificités matérielles
- Réduction de la consommation mémoire

**Fiabilité** :
- Moins d'erreurs de programmation
- Code généré cohérent
- Tests automatisés possibles

### Exemple avec ACADOS

```python
# Définition du problème
ocp = AcadosOcp()
ocp.model = model
ocp.cost = cost
ocp.constraints = constraints

# Génération du solveur
ocp_solver = AcadosOcpSolver(ocp)

# Génération de code C
ocp_solver.generate_code()
```

Le code C généré peut être compilé et intégré dans l'application cible.

---

**Points clés du chapitre** :
- Les contraintes temporelles sont critiques en temps réel
- Le choix de l'architecture matérielle dépend de l'application
- L'Explicit MPC élimine l'optimisation en ligne mais a des limitations
- La génération automatique de code accélère le développement
- Le compromis performance/complexité guide les choix d'implémentation
