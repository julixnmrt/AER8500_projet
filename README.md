# AER8500 — Mini-projet : Simulateur avionique embarqué

Cours AER8500 — Informatique embarquée de l'avionique
Polytechnique Montréal, H2026

---

## Présentation

Ce projet consiste en la simulation d'un système avionique embarqué simplifié, développé dans le cadre du cours AER8500. L'objectif est de modéliser le comportement d'un aéronef en vol en intégrant deux protocoles de communication réels utilisés dans l'industrie aéronautique : **ARINC 429** et **AFDX**.

Le système gère trois états de vol (au sol, changement d'altitude, croisière) et simule de manière physiquement cohérente l'évolution de l'altitude, du taux de montée, de l'angle d'attaque et de la vitesse sol. Une interface graphique permet d'interagir avec le simulateur en temps réel.

---

## Structure du projet

```
AER8500_projet/
├── main.py            # Interface graphique (Tkinter) et boucle principale
├── calculateur.py     # Moteur physique et machine à états
├── aggregateur.py     # Pont entre les couches communication et GUI
├── constantes.py      # Paramètres partagés (limites physiques, états)
├── demo_tests.py      # Tests de validation (40+ cas)
└── communication/
    ├── arinc429.py    # Encodage/décodage ARINC 429 (32 bits, BCD, parité)
    └── afdx.py        # Simulation du réseau AFDX (double canal redondant)
```

Le projet suit une architecture en couches : le calculateur produit des données physiques, les encode en ARINC 429 et les transmet via le réseau AFDX simulé. L'agrégateur reçoit ces trames, les décode et alimente l'interface graphique.

---

## Protocoles implémentés

**ARINC 429** : trois labels sont utilisés pour transmettre l'altitude (label 001, binaire 16 bits, résolution 1 ft), le taux de montée (label 002, BCD 4 chiffres, résolution 0,1 m/min) et l'angle d'attaque (label 003, BCD 3 chiffres, résolution 0,1°). Chaque mot de 32 bits inclut un bit de parité impaire.

**AFDX** : le réseau est simulé avec une redondance double canal (A et B). Les trames sont sérialisées en JSON avec horodatage, source, destination et payload. Un tampon circulaire de 50 trames est maintenu par canal.

---

## Installation et exécution

### Dépendances

- Python 3.10 ou supérieur
- Bibliothèque `psutil` pour la surveillance des ressources système

```bash
pip install psutil
```

Tkinter est inclus dans la distribution standard de Python. Sous Linux, il peut être nécessaire d'installer `python3-tk` séparément.

### Lancer le simulateur

```bash
python main.py
```

### Lancer les tests

```bash
python demo_tests.py
```

---

## Utilisation

L'interface permet de définir une altitude cible, un taux de montée et un angle d'attaque, ainsi que de contrôler la puissance moteur via un curseur. Un facteur d'accélération (×1 à ×100) permet de visualiser rapidement l'évolution du vol. L'interface affiche en temps réel les valeurs physiques, les mots ARINC 429 en hexadécimal et le journal des trames AFDX.

Une détection de décrochage est implémentée : si l'angle d'attaque dépasse ±15° (ou descend sous −9°), l'aéronef entre en chute libre et les commandes sont bloquées jusqu'à l'impact.

---

## Tests

La suite de tests dans `demo_tests.py` couvre les exigences formelles de la machine à états, l'encodage ARINC 429, les formules physiques (vitesse sol, taux de montée), la robustesse face aux cas limites (décrochage, altitude maximale, impact), la validation des entrées et la synchronisation AFDX.
