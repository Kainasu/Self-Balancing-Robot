import sys, os

# initialize default path values:
target_dir = "coppeliasim_api/env"
root_dir = os.getcwd()
copsim_env_path = target_dir

if not os.path.isdir(target_dir):
    while not os.path.isdir(copsim_env_path):
        copsim_env_path = os.path.join('..', copsim_env_path)
    root_dir = copsim_env_path.replace(target_dir, "")

# run notebook in root dir and add the required paths to sys.path:
if  root_dir !=  os.getcwd():  
    os.chdir(root_dir)
    sys.path.append(root_dir)
    sys.path.append(target_dir)

print(f"\troot directory: <{root_dir}>")
print(f"\tworking directory is now: <{os.getcwd()}>")


try:
    import coppeliasim_api.env.sim as sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported.')
    print ('--------------------------------------------------------------')
    print ('')
    raise

import time, sys
import numpy as np
from coppeliasim_api.env.CopSim import Simulator

error_mess = 'Remote API error code: '
OK = sim.simx_return_ok

opmode_blocking    = sim.simx_opmode_blocking
opmode_oneshot     = sim.simx_opmode_oneshot
opmode_buffer      = sim.simx_opmode_buffer
opmode_streaming   = sim.simx_opmode_streaming
opmode_discontinue = sim.simx_opmode_discontinue

######################################################



print(" \033[0;34;47m Avant l'exécution de ce fichier, assurez vous d'avoir exécuter la commande \033[0m : \033[0;33;47m source activate pyml \033[0m \n \n \n")


chemin_model =''
print("\033[0;34;47m"+"Fournir le chemin menant à un model.zip : du type \n"+"/home/flopsy/Documents/pfa/free-balancing-robot/ai/models/balancingrobot/BalancingRobotEnv_CopSim_PPO_22-05-04_16-01-12/ZIP/model\n"+'\033[0m')
chemin_model=input()
from ai.src.visu.copsim_render import model_render
from stable_baselines3 import PPO
from balancing_robot.copsim.BalancingRobotEnv_CopSim import BalancingRobotEnv_CopSim


#print("\033[0;34;47m"+"Fournir le chemin menant à la scene du robot du type \n"+"/home/flopsy/Bureau/free-balancing-robot/balancing_robot/copsim/Balancing-robot.ttt\n"+'\033[0m')
chemin_scene = "./balancing_robot/copsim/Balancing-robot.ttt"
scene = chemin_scene
model = PPO.load(chemin_model)
env   = BalancingRobotEnv_CopSim('v2', scene=scene, headless=False, veloc_mag=3, dt=0.05, reward= 0, x_lim=0.1)

model_render(model, env)
