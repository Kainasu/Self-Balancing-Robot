# Balancing Robot
 
Ce répertoire contient le code IA pour entraîner un contrôleur (agent) sur le balancing robot en utilisant des méthodes d'apprentissage par transfert. Il correspond à une partie du répertoire nommé `miniApterros` (obtenu par clonage du dépôt GitLab https://gitlab.com/jlc/miniapterros.git) et qui se focalise uniquement sur le balancing robot.

Pour tout ce qui concerne l'apprentissage par renforcement (*Deep Reinforcement learning : DRL*), nous utilisons les programmes et la documentation du site  *stable-baselines3* (<https://github.com/DLR-RM/stable-baselines3>).

Pour la simulations des environnement contrôlés, nous utilisons :

* *OpenAI/Gym* (<https://gym.openai.com/>) pour des simulations simples, (inutilisé pour le balancing robot)
* *Coppelia Sim* (<https://www.coppeliarobotics.com/>) pour des simulations plus réalistes, avec un rendu graphique possible.

Pour l'atelier de développement (*Integrated Developement Environment : IDE*), nous conseillons l'IDE **Visual Studio Code** (*aka* VSCode):

* multi-plateforme : Windows, Mac & GNU/Linux,
* code source libre distribué sous licence  MIT, disposant d'une communauté active de contributeurs,
* faible empreinte mémoire et CPU,
* compatible avec les *Environnements Virtuels Python* et le système *git* pour le stockage/versionnage du code source sur les plateformes Github ou GitLab,
* compatible avec les fichiers *notebook jupyter*, avec rendus graphiques,
* doté d'un mécanisme d'extensions puissant simple à utiliser.

Il existe d'autres IDE Python qui proposent des fonctionnalités analogues  (pycharm, atom...). Le choix de VSCode est motivé par sa simplicité d'utilisation et sa faible empreinte mémoire.


## Racine du projet

La racine du projet (*root*) désigne le répertoire contenant ce README.<br>

## Tester l'API Coppelia SIM - Python

* Ouvre le notebook du dossier `coppelia_sim_api/GettingStarted_with_API`. Ils contiennent :
  * le lancement automatique du simulateur CoppeliaSIm (uniquement sous Linux)
  * la mise en oeuvre des fonctions de l'API Python.
L'exécution de ces notebook ne doit pas générer d'erreur.
La notice d'installation se trouve dans `HowToInstall.md` à la racine du dépot.

## Utilisation des scripts Python

* les scripts d'entraînement et tests doivent etre executés à partir de la racine du projet :
  * VSCode : vérifie que le ichier `<racine_projet>/.vscode/launch.json` contient : 
  ```json
  {
    // Use IntelliSense to learn about possible attributes.
    // Hover to view descriptions of existing attributes.
    // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Python : Fichier actuel",
            "type": "python",
            "request": "launch",
            "program": "${file}",
            "console": "integratedTerminal",
            "env": {"PYTHONPATH": "${cwd}" }
        }
    ]
  }
  ```
  * *Si vous avez l'erreur 'no module named ai' tapper sur le terminal* : export PYTHONPATH='/home/am/miniapterros/':$PYTHONPATH

## Entrainement du réseau de neurones pour le balancing robot

Pour entraîner le réseau de neurones, deux possibilités :

* Utiliser le notebook `./ai/notebooks/run_train.ipynb` (recommandé):
* Lancer le script python `./ai/src/run/train_model_copsim.py` avec l'option `--vehicule balancingrobot`

Cela va créer un répertoire `balancingrobot` dans `./ai/models/` contenant les dossiers `"BalancingRobotEnv_CopSim_..."` correspondant aux models qui seront testés dans la simulation dans copsim.

A noter que l'entraînement peut prendre plusieurs heures.

## Lancement de la simulation avec le réseau de neurones entrainé

Pour lancer la simulation avec le réseau de neuronnes préalablement entraîné

* Utiliser le notebook `./ai/notebooks/run_tests.ipynb`

Penser à changer la valeur de `first_train_dir` pour effectuer le test sur les models souhaités (les models testés allant être ceux suivant `first_train_dir` dans l'ordre alphabétique). \
Les tests enregistrent alors dans le dossier du model un dossier `TEST` contenant le résultat de l'exécution. Ensuite le notebook analyse ces résultats et renvoie le graphique de l'évolution de la reward, et la note de la métrique attribuée à chaque model. \
Si le model a déjà été testé, le notebook ne le retestera pas tant qu'il existe le dossier `TEST` et utilisera les résultats enregistrés précédemment. Supprimez le dossier `TEST` pour retester le model.