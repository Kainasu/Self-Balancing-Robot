# Balancing robot

par Alexis Boisseau et Mohamed Amine Amghar

Balancing Robot : Robot à deux roues tenant en équilibre tout seul
1ère solution implémentée : régulateur PID
2ème solution implémentée ici : pilotage par réseaux de neurones
Entraînement au préalable sur simulateur Coppeliasim


*Si vous avez cette l'erreur 'no module named ai' tapper sur le terminal* : export PYTHONPATH='/home/am/miniapterros/':$PYTHONPATH

# Remarques dans la Simulation avec Coppeliasim :
-   On a coché Motor enabled et décoché Control loop enabled
-   On a coché l'option 'lock motor when target velocity is zero' car c'est ce que font les moteurs pas à pas utilisés (lorsque la vitesse ordonnée est nulle, les liaisons pivots ne doivent pas tourner, sinon le robot tourne)
-   Pour le maximum torque (en N*m), nous avons choisi la valeur spécifiée dans la fiche technique du moteur pas à pas
-   Les objets sont groupés pour respecter la physique de l'appareil. Si on souhaite modifier un élément du robot, il faut sélectionner "Planche4" et faire "Ungroup". 

# Pour avoir la vitesse des deux joints :
self.joint_l.get_object_floatParameter(2012)