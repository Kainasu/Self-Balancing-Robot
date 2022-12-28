## Créer de l'environnement de travail


### 1) Environnement Virtuel Python (EVP) nommé **pyml**

* Télécharge et installe la dernière version de **miniconda3** pour ton OS : <https://docs.conda.io/en/latest/miniconda.html>

    **Attention** : le chemin complet du répertoire d'installation `miniconda3`  [ `C:\..chemin..\miniconda3` pour Wndows, ou `/..chemin../miniconda3` pour Mac & Linux ] doit être choisi de façon à ne comporter aucun caractère accentué ni espace !!!

* Crée l'EVP **pyml** en tapant dans une console (un terminal Mac/Linux ou une fenêtre "Anaconda prompt" Windows) :

    `conda create -n pyml python=3.6`

    => Attention à bien utiliser le nom **pyml** : la suite du tutoriel, les fichiers de configurations et les exemples fournis, sont basés sur ce nom.

### 2) Activer l'EVP **pyml**

Deux situations se rencontrent :

* Dans une console => on active l'EVP en tapant la commande :

  `conda activate pyml`

* Avec VSCode :<br> Installer l'extension `Python` depuis la section `Extensions` à gauche dans VScode <br> => on utilise le raccourci clavier `SHIFT+CTRL+P` puis on continue en tapant P`ython: Select Interp` 

  puis **Find...** pour sélectionner l'**interpréteur Python** dans l'arborescence de l'EVP (par exemple pour Linux : `/..chemin../miniconda3/envs/pyml/bin/python3`)<br>

  Idem pour le chemin de l'**Interpréteur IPython pour le serveur jupyter**.

### 3) Compléter l'installation des modules Python

* Le plus simple est de taper les commandes dans la console **avec l'EVP pyml activé** :

    `conda install pytorch`<br>
    `pip install stable-baselines3`<br>
    `pip install stable-baselines3[extra]`<br>
    `pip install pyyaml pyqtgraph qdarkstyle`<br>
    `pip install box2d`<br>
    `conda install ffmpeg=4.2`

### 4) Lancer un script (doit être fait avant de lancer n'importe quel code python, n'a besoin d'être fait qu'une fois)
[en] Execute a script (must be done before executing any python code ; only needs to be done once)

* À la racine du project executez la commande suivante dans votre terminal:
* [en] At the project's root please execute the following command:
    `chmod 770 ./prereq_root_as_cwd.sh`<br>  //may not be necessary
    `./prereq_root_as_cwd.sh`<br>

lorsque vous voudrez appeller la fonction "prereq" il faudra importer le fichier "root_as_cwd", pour celà il faudra inclure cette ligne de code "sys.path.append("/tmp/temp/")" avant de faire "import root_as_cwd"
ie en tête de fichier il faudra copie ces lignes de code : 
    `sys.path.append("/tmp/temp/")`<br>  //may not be necessary
    `import root_as_cwd`<br>
    

* Ce script permet de mettre une fichier à un emplacement dont le chemin absolu est connu. Ce fichier contient une fonction qui place le current working directory à la racine du projet. Appeler cette fonction permet d'effectuer des "import" de fichiers du projet. Cette étape ne doit donc pas être sautée.

* [en] This script moves a file to somwhere where the absolute path is known. This files contains a function that sets the project's root as the current working directory. Calling this function allows for easier imports in all files, this step can therefore not be skipped.

(ffmpeg sert pour sauvegarder les videos des tests)

## Installer le simulateur CoppeliaSim

Pour installer CoppeliaSim (Linux):

* Va sur <https://www.coppeliarobotics.com/downloads> et télécharge la version Coppelia Sim **EDU** pour Ubuntu 20.
* Dézippe le fichier téléchargé et extrait le dossier `CoppeliaSim_Edu_V4_xx_Ubuntu20_04` à la racine du projet.
* Met à jour le chemin  `COPSIM_DIR` dans le fichier `copsim_env/constants.py`, correspondant à la version téléchargée.

## Utilisation des notebooks

Les notebooks Jupyter peuvent être utilisés de 2 façons :
* Avec l'EDI Jupyter :
  * si besoin, installe `jupyter` en tapant dans le terminal (ou console "Anaconda prompt") la commande : `conda install jupyter`.<br>
  Pour lancer jupyte, tape `jupyter notebook` dans le terminal (ou console "Anaconda prompt"), **avec l'EVP `pyml` activé**.
* avec VSCode : l'utilisation des notebooks est intégré à l'atelier.

## Configuration de l'EDI Viual Studio Code

* `Workbench`
  * `Apparence` 
    * `Tree:indent` : 12 
* `Features`
  * `terminal`
    * `External: Windows Exe`: C:\Windows\System32\cmd.exe
    * `Integrated: Inherit Env`: Ccoché
* `Extensions`
  * `Jupyter`
    * `Always trust Notebook`: coché