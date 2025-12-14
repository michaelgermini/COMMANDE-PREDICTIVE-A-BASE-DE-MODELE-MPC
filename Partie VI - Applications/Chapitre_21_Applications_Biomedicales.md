# Chapitre 21 : Applications Biomédicales

La MPC trouve des applications importantes en biomédical, où le contrôle précis de processus physiologiques est crucial pour la santé et le bien-être des patients.

## 21.1 Pancréas artificiel (contrôle de la glycémie)

### Problème

Le **pancréas artificiel** automatise la gestion de la glycémie pour les patients diabétiques de type 1, remplaçant les injections manuelles d'insuline.

**Système** :
- **État** : Glycémie $G(t)$ (mesurée par capteur continu)
- **Commande** : Débit d'insuline $u(t)$ (pompe à insuline)
- **Perturbations** : Repas, exercice, stress, maladie

**Modèle de glucose** (simplifié) :
$$
\begin{cases}
\dot{G} = -p_1 G - p_2 X I + \frac{G_{meal}}{V_G} \\
\dot{X} = -p_2 X + p_3 I \\
\dot{I} = -\frac{I}{\tau_I} + \frac{u}{V_I}
\end{cases}
$$

où :
- $G$ : Glycémie
- $X$ : Action de l'insuline
- $I$ : Insuline plasmatique
- $G_{meal}$ : Apport glucidique (perturbation)
- $p_1, p_2, p_3, \tau_I$ : Paramètres patient-spécifiques

### MPC pour pancréas artificiel

**Objectif** :
$$
J = \sum_{i=1}^{N_p} \|G - G_{target}\|^2 + \|u\|^2 + c_{hypo} \max(0, G_{min} - G)^2
$$

où $c_{hypo}$ pénalise fortement l'hypoglycémie (dangereuse).

**Contraintes** :
- **Glycémie** : $70 \leq G \leq 180$ mg/dL (bande sûre)
  - Hypoglycémie : $G < 70$ (dangereux, à éviter absolument)
  - Hyperglycémie : $G > 180$ (moins critique à court terme)
- **Insuline** : $0 \leq u \leq u_{max}$ (débit max de la pompe)
- **Variation insuline** : $|\Delta u| \leq \Delta u_{max}$ (éviter variations brusques)

**Défis** :
- **Sécurité** : L'hypoglycémie peut être mortelle
- **Variabilité inter-patient** : Modèle doit être adapté
- **Perturbations imprévisibles** : Repas, exercice
- **Délais** : Action de l'insuline avec délai (30-60 min)

**Stratégies** :
- **Contraintes souples** : Pour hyperglycémie (moins critique)
- **Contraintes dures** : Pour hypoglycémie (sécurité)
- **Prédiction repas** : Si annoncé, anticipation
- **MPC adaptative** : Ajustement paramètres en ligne

**Résultats** :
- **Temps dans bande** : 70-80% (vs 50-60% manuel)
- **Hypoglycémies** : Réduction de 50-70%
- **Qualité de vie** : Amélioration significative

## 21.2 Anesthésie automatisée

### Problème

Contrôler automatiquement la profondeur d'anesthésie pendant une chirurgie.

**Variables** :
- **Profondeur anesthésie** : BIS (Bispectral Index), $0-100$ (40-60 cible)
- **Commandes** : Débit propofol $u_{prop}(t)$, débit remifentanil $u_{remi}(t)$
- **Perturbations** : Stimulations chirurgicales, variations patient

**Modèle** (pharmacocinétique/pharmacodynamique) :
$$
\begin{cases}
\dot{C}_{prop} = -\frac{C_{prop}}{\tau_{prop}} + \frac{u_{prop}}{V_{prop}} \\
\dot{C}_{remi} = -\frac{C_{remi}}{\tau_{remi}} + \frac{u_{remi}}{V_{remi}} \\
BIS = f(C_{prop}, C_{remi}, \text{stimulation})
\end{cases}
$$

### MPC pour anesthésie

**Objectif** :
$$
J = \sum_{i=1}^{N_p} \|BIS - BIS_{target}\|^2 + \|u_{prop}\|^2 + \|u_{remi}\|^2
$$

**Contraintes** :
- **BIS** : $40 \leq BIS \leq 60$ (bande anesthésie)
  - $BIS > 60$ : Anesthésie trop légère (risque réveil)
  - $BIS < 40$ : Anesthésie trop profonde (risques)
- **Débits** : $0 \leq u_{prop} \leq u_{prop,max}$, $0 \leq u_{remi} \leq u_{remi,max}$
- **Variations** : Limites sur variations brusques

**Défis** :
- **Sécurité** : Erreurs peuvent être graves
- **Variabilité** : Grande variabilité inter-patient
- **Délais** : Action avec délai (2-5 min)
- **Stimulations** : Perturbations imprévisibles

**Résultats** :
- **Stabilité** : Réduction variations BIS de 40-50%
- **Sécurité** : Moins de dépassements hors bande
- **Consommation** : Optimisation médicaments

## 21.3 Ventilation mécanique

### Problème

Assister la respiration de patients en insuffisance respiratoire.

**Variables** :
- **Volume pulmonaire** : $V(t)$
- **Pression** : $P(t)$
- **Commandes** : Pression inspiratoire $P_{insp}(t)$, Fréquence respiratoire $f_{resp}(t)$
- **Perturbations** : Effort patient, changements compliance

**Modèle** (simplifié) :
$$
\begin{cases}
\dot{V} = \frac{P_{insp} - P_{exp} - P_{muscle}}{R} \\
P = \frac{V}{C} + P_{exp}
\end{cases}
$$

où :
- $R$ : Résistance des voies aériennes
- $C$ : Compliance pulmonaire
- $P_{muscle}$ : Effort musculaire patient (perturbation)

### MPC pour ventilation

**Objectif** :
$$
J = \sum_{i=1}^{N_p} \|V - V_{target}\|^2 + \|P - P_{safe}\|^2 + \|P_{insp}\|^2
$$

**Contraintes** :
- **Volume** : $V_{min} \leq V \leq V_{max}$ (tidal volume)
- **Pression** : $P \leq P_{max}$ (sécurité, éviter barotraumatisme)
- **Pression inspiratoire** : $0 \leq P_{insp} \leq P_{insp,max}$
- **Fréquence** : $f_{min} \leq f_{resp} \leq f_{max}$

**Modes de ventilation** :
- **Assistée** : Aide au patient qui respire
- **Contrôlée** : Ventilation complète si patient ne respire pas
- **Adaptative** : Passage automatique selon état patient

**Défis** :
- **Synchronisation** : Détecter et s'adapter à l'effort patient
- **Variabilité** : Paramètres variant dans le temps
- **Sécurité** : Éviter surpression, hypoventilation

**Résultats** :
- **Comfort patient** : Amélioration synchronisation
- **Sécurité** : Réduction risques barotraumatisme
- **Efficacité** : Optimisation paramètres ventilation

## 21.4 Systèmes de délivrance de médicaments

### Problème

Contrôler la concentration plasmatique d'un médicament à une valeur thérapeutique.

**Variables** :
- **Concentration** : $C(t)$ (mesurée ou estimée)
- **Commande** : Débit d'administration $u(t)$
- **Objectif** : Maintenir $C$ dans une fenêtre thérapeutique

**Modèle pharmacocinétique** :
$$
\begin{cases}
\dot{C}_1 = -k_{12} C_1 + k_{21} C_2 - k_{el} C_1 + \frac{u}{V_1} \\
\dot{C}_2 = k_{12} C_1 - k_{21} C_2
\end{cases}
$$

(modèle à deux compartiments)

### MPC pour délivrance

**Objectif** :
$$
J = \sum_{i=1}^{N_p} \|C - C_{target}\|^2 + \|u\|^2
$$

**Contraintes** :
- **Concentration** : $C_{min} \leq C \leq C_{max}$ (fenêtre thérapeutique)
  - $C < C_{min}$ : Efficacité insuffisante
  - $C > C_{max}$ : Risque toxicité
- **Débit** : $0 \leq u \leq u_{max}$
- **Dose totale** : $\sum u \leq D_{max}$ (dose journalière max)

**Applications** :
- **Analgésie** : Morphine, fentanyl
- **Sédation** : Propofol, midazolam
- **Chimiothérapie** : Doses contrôlées

**Défis** :
- **Variabilité inter-patient** : Pharmacocinétique variable
- **Adaptation** : Ajustement selon réponse
- **Sécurité** : Éviter surdosage

**Résultats** :
- **Précision** : Concentration plus stable
- **Efficacité** : Meilleure efficacité thérapeutique
- **Sécurité** : Réduction risques surdosage

### Exemple : Contrôle de la douleur

**Système** : Administration d'analgésiques (morphine) pour contrôler la douleur.

**Variables** :
- **Score douleur** : $0-10$ (échelle visuelle)
- **Concentration** : $C_{morphine}(t)$
- **Commande** : Débit morphine $u(t)$

**MPC** :
$$
J = \sum_{i=1}^{N_p} \|Pain - Pain_{target}\|^2 + \|C - C_{safe}\|^2 + \|u\|^2
$$

**Contraintes** :
- Douleur : $Pain \leq Pain_{max}$ (tolérable)
- Concentration : $C \leq C_{max}$ (sécurité, éviter dépression respiratoire)
- Débit : $0 \leq u \leq u_{max}$

**Résultats** :
- Contrôle douleur : Amélioration
- Sécurité : Respect limites
- Consommation : Optimisation

---

**Points clés du chapitre** :
- Le pancréas artificiel utilise la MPC pour automatiser la gestion de la glycémie
- L'anesthésie automatisée contrôle la profondeur d'anesthésie avec sécurité
- La ventilation mécanique assiste la respiration de manière adaptative
- Les systèmes de délivrance de médicaments maintiennent des concentrations thérapeutiques
- La sécurité est primordiale dans toutes les applications biomédicales
- Les résultats montrent des améliorations significatives en contrôle et sécurité
