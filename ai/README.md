# Projet Mini-Apterros / IA

Ce répertoire contient le code IA pour entraîner un contrôleur (agent) sur plusieurs environments (CartPole, Mini-Apterros...) en utilisant des méthodes d'apprentissage par transfert.

Pour tout ce qui concerne l'apprentissage par renforcement (*Deep Reinforcement learning : DRL*), nous utilisons les programmes et la documentation du site  *stable-baselines3* (<https://github.com/DLR-RM/stable-baselines3>).

Pour la simulations des environnement contrôlés, nous utilisons :

* *OpenAI/Gym* (<https://gym.openai.com/>) pour des simulations simples,
* *Coppelia Sim* (<https://www.coppeliarobotics.com/>) pour des simulations plus réalistes, avec un rendu graphique possible.

Pour l'atelier de développement (*Integrated Developement Environment : IDE*), nous conseillons l'IDE **Visual Studio Code** (*aka* VSCode):

* multi-plateforme : Windows, Mac & GNU/Linux,
* code source libre distribué sous licence  MIT, disposant d'une communauté active de contributeurs,
* faible empreinte mémoire et CPU,
* compatible avec les *Environnements Virtuels Python* et le système *git* pour le stockage/versionnage du code source sur les plateformes Github ou GitLab,
* compatible avec les fichiers *notebook jupyter*, avec rendus graphiques,
* doté d'un mécanisme d'extensions puissant simple à utiliser.

Il existe d'autres IDE Python qui proposent des fonctionnalités analogues  (pycharm, atom...). Le choix de VSCode est motivé par sa simplicité d'utilisation et sa faible empreinte mémoire.

## Création de l'environnement de travail

La racine du projet (*root*) désigne le répertoire contenant ce Readme (le répertoire de plus haut niveau du projet).

### 1) Créer l'Environnement Virtuel Python (EVP) nommé **pyml**

* Télécharge et installe la dernière version de **miniconda3** pour ton OS <https://docs.conda.io/en/latest/miniconda.html>

    **Attention** : le chemin complet du répertoire d'installation `miniconda3`  [ `C:\...\miniconda3` pour Wndows, ou `/.../miniconda3` pour Mac & Linux ] doit être choisi de façon à ne comporter aucun catactère accentué ni aucun espace !!!

* Crée l'EVP **pyml** en tapant dans un terminal (Mac, GNU/Linux) ou dans une fenêtre "Anconda prompt" (Windows) :

    `conda create -n pyml python=3.6`

    => Attention à bien utiliser le nom **pyml** : la suite du tutoriel, les fichiers de configurations et les exemples fournis, sont basés sur ce nom.

### 3) Activer l'EVP **pyml**

Deux situations se rencontrent :

* Dans un terminal (ou une fenêtre "Anaconda prompt") => on active l'EVP avec la commande :

  `conda activate pyml`

* Avec VSCode, on utilise le raccourci clavier `SHIFT + CTRL + P` puis on continue en tapant P`ython: Select Interp` :

  ![bbbb](docs/Images/VSC_selectEVP_3.png)

  puis **Find...** pour sélectionner l'**interpréteur Python** dans l'arborescence de l'EVP (par exemple pour Linux : `/.../miniconda3/envs/pyml/bin/python3`)
  ![bbbb](docs/Images/VSC_selectEVP_2.png)

  Idem pour le chemin de l'**Interpréteur IPython pour le serveur jupyter**.

### 4) Compléter l'installation des modules Python

* Le plus simple est de taper les commandes dans le terminal ("fenêtre Anaconda" pour Windows), **avec l'EVP pyml activé** :

    `pip install stable-baselines3[extra]`<br>
    `pip install pyyaml pyqtgraph qdarkstyle`<br>
    `conda install ffmpeg=4.2`

(ffmpeg sert pour sauvegarder les videos des tests)

### 5) Tester le bon fonctionnement

* Charger le script de test `ai/src/test/test_ppo.py` dans l'atelier VSCode, et le faire excéuter.
* La vidéo du test sur le modèle entrainé est auvegardée à la fin de l'entraînement dans le répertoire `ia/out`.

## Installation Coppelia Sim

Pour installer Coppelia Sim (sur Linux):

* Aller sur <https://www.coppeliarobotics.com/downloads> et télécharger la version Coppelia Sim **EDU** pour Ubuntu.
* Décompresser le fichier téléchargé.
* Placer le dossier ainsi obtenu  ("CoppeliaSim_Edu_V4_1_0_Ubuntu20_04") à la racine du projet.
* si nécessaire (en cas d'utilisation d'une version différente), mettre à jour le path dans `coppeliasim_api/env/constants.py`.

## Utilisation des scripts

* les scripts d'entraînement et tests doivent etre executés à partir de la racine du projet.
* dans pycharm, cela peut se faire facilement en paramétrant une configuration ("edit configuration" en haut à droite) pour que cela se fasse de manière automatique

## Utilisation notebooks

* Installer "jupyter notebook" :
  * dans le terminal (ou fenêtre "Anaconda prompt"), installe le module Python `jupyter` avec la commande :

    `pip install jupyter`

* Pour lancer :
  * dans la console de pycharm : lancer "jupyter notebook"
    ouvrir le lien http (<http://localhost:888/?token>=....) qui apparait dans un navigateur internet
* Pour créer un nouveau notebook :
    il est conseillé de partir d'une copie du notebook "Template.ipynb" qui offre l'avantage :

    a) de s'executer à la racine du projet (évite les problemes d'import relatif des modules

    b) de prendre en compte automatiquement toute modification des environnements ou des sources python du projet sans avoir à re-importer les packages et relancer le kernel.





# Comment compiler test_reseau.py

lancer la commande : 

conda run -n pyml --no-capture-output --live-stream python [chemin menant a test_reseau.py]

exemple de chemin : /home/flopsy/Bureau/free-balancing-robot/ai/test_reseau.py
