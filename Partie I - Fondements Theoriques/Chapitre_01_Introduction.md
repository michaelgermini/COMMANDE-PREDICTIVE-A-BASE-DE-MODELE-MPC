# Chapitre 1 : Introduction à la Commande Prédictive

## 1.1 Historique et évolution de la MPC

La Commande Prédictive à base de Modèle (Model Predictive Control, MPC) est née dans les années 1970, principalement dans l'industrie pétrochimique où les besoins de contrôle étaient particulièrement complexes. Les premiers développements significatifs incluent :

- **1970s** : Développement du Dynamic Matrix Control (DMC) par Shell Oil Company, considéré comme l'une des premières implémentations industrielles de la MPC.

- **1980s** : Introduction du Model Algorithmic Control (MAC) et du Generalized Predictive Control (GPC) par Clarke et ses collaborateurs, qui ont formalisé mathématiquement l'approche.

- **1990s** : Expansion vers les systèmes non-linéaires avec le développement de la Non-Linear MPC (NMPC), et amélioration des algorithmes d'optimisation pour le temps réel.

- **2000s-2010s** : Intégration de la robustesse, développement de l'Explicit MPC, et applications dans de nouveaux domaines (véhicules autonomes, énergie, biomédical).

- **2020s** : Convergence avec l'apprentissage automatique, développement de la Learning-based MPC et de méthodes data-driven.

L'évolution de la MPC reflète les progrès en optimisation numérique, en puissance de calcul, et en modélisation des systèmes complexes.

## 1.2 Principes fondamentaux

La MPC repose sur trois principes fondamentaux :

### Prédiction
La MPC utilise un **modèle du système** pour prédire son comportement futur sur un horizon de prédiction. Ce modèle peut être linéaire ou non-linéaire, et capture la dynamique du système à contrôler.

### Optimisation
À chaque instant, la MPC résout un **problème d'optimisation** qui minimise une fonction de coût (ou maximise une fonction d'utilité) sous contraintes. Cette optimisation détermine la séquence optimale de commandes futures.

### Récurrence
Seule la **première commande** de la séquence optimale est appliquée au système. À l'instant suivant, le processus est répété avec les nouvelles mesures, ce qui constitue le principe de **receding horizon** (horizon glissant).

### Schéma de fonctionnement

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   Mesures   │────▶│  Optimisation│────▶│  Commande   │
│  actuelles  │     │   MPC        │     │  appliquée  │
└─────────────┘     └──────────────┘     └─────────────┘
                            │
                            ▼
                    ┌──────────────┐
                    │   Modèle de  │
                    │  prédiction  │
                    └──────────────┘
```

Cette approche permet de gérer explicitement les contraintes, d'anticiper le comportement futur, et d'optimiser les performances sur un horizon donné.

## 1.3 Avantages et limitations

### Avantages de la MPC

1. **Gestion explicite des contraintes** : La MPC peut traiter directement les contraintes sur les entrées, sorties et états, ce qui est difficile avec les méthodes classiques.

2. **Optimisation multi-objectif** : La fonction de coût peut combiner plusieurs objectifs (suivi de référence, économie d'énergie, confort, etc.).

3. **Prédiction et anticipation** : La capacité de prédire le comportement futur permet d'anticiper les perturbations et d'améliorer les performances.

4. **Flexibilité** : La MPC s'adapte facilement aux changements d'objectifs ou de contraintes en modifiant simplement la fonction de coût.

5. **Applicabilité large** : Fonctionne pour les systèmes mono-variables et multi-variables, linéaires et non-linéaires.

### Limitations de la MPC

1. **Charge computationnelle** : La résolution d'un problème d'optimisation à chaque pas de temps peut être coûteuse, surtout pour les systèmes non-linéaires.

2. **Nécessité d'un modèle** : Un modèle précis du système est requis, ce qui peut nécessiter des efforts importants d'identification.

3. **Complexité de réglage** : Le choix des horizons, des pondérations et des contraintes nécessite de l'expertise.

4. **Garanties de stabilité** : La stabilité en boucle fermée n'est pas automatiquement garantie et doit être analysée ou imposée.

5. **Sensibilité aux erreurs de modèle** : Les performances peuvent se dégrader significativement si le modèle est imprécis.

## 1.4 Comparaison avec les méthodes de contrôle classiques

### Contrôle PID (Proportionnel-Intégral-Dérivé)

| Aspect | PID | MPC |
|--------|-----|-----|
| **Contraintes** | Gestion difficile (saturation) | Gestion explicite |
| **Multi-variables** | Nécessite plusieurs boucles | Traitement naturel |
| **Prédiction** | Aucune | Prédiction explicite |
| **Complexité** | Faible | Élevée |
| **Temps réel** | Très rapide | Dépend de la complexité |
| **Réglage** | Relativement simple | Plus complexe |

### Contrôle LQR (Linear Quadratic Regulator)

| Aspect | LQR | MPC |
|--------|-----|-----|
| **Contraintes** | Non gérées | Gérées explicitement |
| **Horizon** | Infini | Fini (réglable) |
| **Optimisation** | Offline (gain constant) | Online (optimisation à chaque pas) |
| **Robustesse** | Limitée | Améliorable (MPC robuste) |

### Contrôle par retour d'état

La MPC peut être vue comme une généralisation du contrôle par retour d'état avec :
- Horizon fini au lieu d'infini
- Gestion explicite des contraintes
- Optimisation en ligne

## 1.5 Domaines d'application

La MPC trouve des applications dans de nombreux domaines :

### Industrie
- **Pétrochimie** : Colonnes de distillation, réacteurs chimiques
- **Manufacturing** : Lignes de production, contrôle de qualité
- **Énergie** : Centrales électriques, smart grids

### Transport
- **Véhicules autonomes** : Planification de trajectoire, contrôle latéral/longitudinal
- **Aéronautique** : Pilotage automatique, atterrissage
- **Ferroviaire** : Optimisation de la consommation d'énergie

### Bâtiments et énergie
- **HVAC** : Chauffage, ventilation, climatisation
- **Smart buildings** : Gestion énergétique optimale
- **Énergies renouvelables** : Contrôle d'éoliennes, panneaux solaires

### Biomédical
- **Pancréas artificiel** : Contrôle de la glycémie
- **Anesthésie** : Délivrance automatisée de médicaments
- **Ventilation mécanique** : Assistance respiratoire

### Robotique
- **Manipulateurs** : Contrôle de trajectoire
- **Robots mobiles** : Navigation et évitement d'obstacles
- **Drones** : Stabilisation et suivi de trajectoire

La diversité de ces applications témoigne de la polyvalence et de la puissance de la MPC comme technique de contrôle moderne.

---

**Points clés du chapitre** :
- La MPC combine prédiction, optimisation et récurrence
- Elle excelle dans la gestion des contraintes et des systèmes multi-variables
- Elle nécessite un modèle et une charge computationnelle plus importante que les méthodes classiques
- Ses applications couvrent de nombreux domaines industriels et académiques
