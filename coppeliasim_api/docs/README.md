## Comment utiliser l'API python de copeliasim.

Une documentation non exhaustive des fonctions utilisables : https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm

Si cette documentation ne suffit pas, voici un petit guide qui a pour ambition d'être complémentaire : 

- Le fichier sim.py  contient l'ensemble des fonctions de l'API
- Le fichier simConst.py contient toutes les constantes faisant reférence au paramètres  modifiables de copéliasim.

Exemple : on veut changer le moteur physique en utilisant l'api

```python
sim.simxSetIntegerParameter(clientID, sim_intparam_dynamic_engine, 3 , blocking)
```

- simxSetIntegerParameter définie dans sim.py
- sim_intparam_dynamic_engine définie dans simConst.py
- (3 mais correspond  au moteur newton)



Pour l'utilisation de certaines constantes, une recherche sur google plutot que dans la doc sera souvent plus fructueuse.