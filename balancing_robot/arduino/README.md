
## Dossier : Balancing_robot_PID
    ### Contient le programme arduino du pilotage du Balancing Robot par PID, c'est la première version que nous avons utilisé
    **Les variables PIN_DIR_R, PIN_DIR_L, PIN_STEP_R, PIN_STEP_L sont à changer selon les pins utilisés dans le circuit éléctrique**
    **Les variables pid_p_gain, pid_k_gain, pid_i_gain sont à changer selon les dimensions et poids des composantes du robot**
    ## lien du code source : http://www.brokking.net/yabr_main.html

## Dossier : YABR_hardware_test
    ### Programme utilisé pour obtenir l'offset de la MPU6050 utilisé en position initiale
    ### Remarque : Il est utilisé pour calibrer la MPU6050 dans le dossier Balancing_robot_PID

## Dossier : Balancing_robot_PID_1
    ### Contient le programme arduino du pilotage du Balancing Robot par PID, c'est la version finale : nous avons constaté que le calcul des angles dans la premiere version était erroné, ansi nous avons utilisé un filtre qui donne des mesures d'angles plus précis de la MPU6050

## Dossier : initialising_offset_MPU6050
    ### Programme utilisé pour obtenir l'offset de la MPU6050 utilisé en position initiale
    ### Remarque : Il est utilisé pour calibrer la MPU6050 dans le dossier Balancing_robot_PID_1 (utilisant le bon filtre de mesures d'angles)

## Dossier : balancingrobot_driver
    ### Ce dossier contient les programmes arduino utilisés pour communiquer avec le modele du RL, elle lui communique les valeurs [ x : position du robot, x' : vitesse du robot, theta : angle de le MPU6050, theta' : la vitesse de rotation de la MPU6050 ] 

## Dossier : BlueTooth_comm
    ### Pas encore utilisé, il sert à faire la communication du robot avec un périphérique bluetooth

## Dossier : MPU6050
    ### dossier qui contient les fichiers pour lancer la MPU6050

## Dossier : stepper_motor
    ### dossier pour faire marcher le moteur pas-à-pas
