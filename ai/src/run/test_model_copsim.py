#
# In this file we define two functions :
# evaluate_model : to display and save how a model perform on a given environment
# test_model     : to run an experiment described in cfg_file, save the results and config for reproducibility
# If the file run directly with Python, the "main bloc" at the end is run.
#

import numpy as np
import gym
import os, sys, time, shutil, yaml
from numpy.core.numeric import outer
from os.path import realpath

from stable_baselines3.common.base_class import BaseAlgorithm
from ai.src.run.constants import OUTPUT_DIR, TEST_DIR, EXPERIMENT_CONFIG_FILENAME, ENVIRONMENT_CONFIG_FILENAME
from ai.src.run.constants import PERFORMANCE_REPORT_FILENAME

# import custom gym envs using CopSim as render 
#from cartpole.copsim.CartpoleEnv_CopSim import CartPoleEnv_CopSim
from balancing_robot.copsim.BalancingRobotEnv_CopSim import BalancingRobotEnv_CopSim
# TODO add apterros env when available

def evaluate_model(model: BaseAlgorithm, 
                   env: gym.Env, 
                   deterministic: bool = True, 
                   steps_nb: int = 1000,
                   verbose=True) -> dict:
    """
    Computes how a model performs on a given environment

    :param model:   instance of a stable_baselines3 model
    :param env:     instance of a gym environment for a Coppelia Sim scene
    :param deterministic: if random noise is applied to the model during evaluation
    :param steps_nb: nb of simulation steps for evaluation
    :return: dict of {metric names:metric values}
    """
    # Display the results of the training ###
    env._max_episode_steps = steps_nb+1
    obs = env.reset(theta_deg=0.005)
    done, step_count, rewards, actions, states = False, 0, [], [], []

    # at each step, will make the agent predict the next action and update the environment consequently
    while not done and step_count < steps_nb:
        # deterministic=False add random noise to test with an harder task
        action, _ = model.predict(obs, deterministic=deterministic)
        obs, reward, done, info = env.step(action)
        rewards.append(float(reward))
        actions.append(float(action))
        states.append(obs)
        step_count += 1
        if verbose and (step_count % 10 == 0) : print("\r", step_count, end="")
        #print("------------------------------------------", obs, states)
    if verbose: print(" ...done!")
    if step_count == steps_nb+1: step_count -= 1
    state = np.array(states) #Making it an array makes it easier to slice
    return {"actions": actions,
            "mean_abs_actions": float(np.mean(np.abs(actions))),
            "rewards" : rewards,
            "reward_cum": sum(rewards),
            "rewards_mean": float(np.mean(rewards)),
            "rewards_std": float(np.std(rewards)),
            "last_step_count": step_count,
            "percent_completion": step_count/steps_nb,
            "x": state[:, 0].tolist(),  #We have to make them lists so they can be saved in a file by yaml
            "x_dot": state[:,1].tolist(),
            "theta": state[:,2].tolist(),
            "theta_dot": state[:,3].tolist()}


def test_model(cfg_loc: str=None, copsim_port=19997, display_plots: bool=False, always_test=False, verbose=True):
    """
    Run an experiment described in cfg_file, and save the results and config for reproducibility

    :param cfg_loc: path of the directory that contains the yaml files
    :return: None
    """

    assert os.path.isdir(cfg_loc)
    real_dir  = realpath(cfg_loc)

    cfg_file = os.path.join(real_dir, EXPERIMENT_CONFIG_FILENAME)

    with open(cfg_file, 'r') as f:
        cfg = yaml.safe_load(f.read())
 
    try:
        environment     = cfg['env']['environment']
        scene           = cfg['env']['scene']

        agent_type      = cfg['train']['agent_type']
        
        total_timesteps = cfg['eval']['total_timesteps']
        headless        = cfg['eval']['headless']
        deterministic   = cfg['eval']['deterministic']
        model_name      = cfg['eval']['model_name']
    except:
        raise RuntimeError("Parametres missing in file <{cfg_file}>")
    
    # make the relative pathname of the file 'model.zip':
    full_model_name = os.path.join(real_dir, 'ZIP', model_name)
    if not os.path.exists(full_model_name):
        # the raining was done before the end: no link 'model.zip'
        zip_file_list = os.listdir(os.path.join(real_dir, 'ZIP'))
        zip_file_list.sort()
        full_model_name = os.path.join(real_dir, 'ZIP', zip_file_list[-1])
        print(f"\t 'model.zip' not found, using <{zip_file_list[-1]}>")
    
    # load the saved environment config file:
    if verbose: print(f"env. parameters loaded from <{os.path.basename(real_dir)}/{ENVIRONMENT_CONFIG_FILENAME }>")
    cfg_file_env = real_dir/ENVIRONMENT_CONFIG_FILENAME         # cfg['env']['cfg_env']
    with open(cfg_file_env, 'r') as f:
        cfg_env = yaml.safe_load(f.read())

    # create the test directory if needed:
    test_dir = real_dir/TEST_DIR
    if always_test or not test_dir.exists(): 

        # import agent
        if  agent_type == 'PPO':
            from stable_baselines3 import PPO as agent
        else:
            raise Exception("Not implemented agent : <{agent_type}>")
        
        
        # env for evaluation
        
        if environment == 'CartPoleEnv_CopSim':
            # load the saved parameters:
            veloc     = cfg_env['velocity']
            dt        = cfg_env['dt']
            version   = cfg_env['version']
            theta_lim = cfg_env['theta_lim']
            x_lim     = cfg_env['x_lim']   
            reward    = cfg_env['reward']       
            if verbose: print(f"Will run CartPoleEnv_CopSim with veloc:{veloc}, dt:{dt} and version:{version}")
            eval_env = CartPoleEnv_CopSim(version, 
                                 headless=headless, 
                                 scene=scene, 
                                 verbose=0, 
                                 veloc_mag=veloc, 
                                 dt=dt,
                                 theta_lim_deg = theta_lim,
                                 x_lim = x_lim,
                                 reward=reward,
                                 coppelia_sim_port=copsim_port)
        elif environment == 'BalancingRobotEnv_CopSim':
            # load the saved parameters:
            veloc    = cfg_env['velocity']
            dt       = cfg_env['dt']
            version  = cfg_env['version']
        
            eval_env = BalancingRobotEnv_CopSim(version, 
                                                headless=headless, 
                                                scene=scene, 
                                                verbose=0,
                                                veloc_mag=veloc, 
                                                dt=dt)
        else:
            raise Exception("Not implemented environment: <{environment}>")
            
        
        if verbose: print(f"Loading model <{model_name}> from directory <{os.path.basename(real_dir)}>")
        model = agent.load(full_model_name)

        if not test_dir.exists(): test_dir.mkdir(parents=True)
    
        # Evaluate the trained agent    
        perf = evaluate_model(model, 
                            eval_env,
                            steps_nb=total_timesteps,
                            deterministic=deterministic)

        perf["deterministic"] = deterministic

        
        with open(test_dir/PERFORMANCE_REPORT_FILENAME, 'w') as outfile:
            yaml.dump(perf, outfile, default_flow_style=True)

        eval_env.close()

    else:
            if verbose: print(f"TEST directory already exists: reading file <{os.path.basename(real_dir)}/TEST/{PERFORMANCE_REPORT_FILENAME}>")
            with open(test_dir/PERFORMANCE_REPORT_FILENAME, 'r') as f:
                perf = yaml.safe_load(f.read())

    for key, fmt in zip(('deterministic', 'mean_abs_actions', 'reward_cum', 'rewards_mean', 'last_step_count', 'percent_completion', 'x', 'x_dot', 'theta', 'theta_dot'),
                        ('', '.2f','.2f', '.2f', 'd', '.1f', '.2f', '.2f', '.2f', '.2f')):
        if verbose: print(f"{key:20s}: {perf[key]:{fmt}}")
    
    # plot rewards = f(time_step)
    import matplotlib.pyplot as plt
    actions = perf["actions"]
    rewards = perf['rewards']
    
    plt.figure(real_dir, figsize=(10,6))
    plt.subplots_adjust(hspace=.4)
    plt.subplot(2,1,1)
    plt.title(f"Graph of rewards <{os.path.basename(real_dir)}>")
    plt.plot(range(perf['last_step_count']), rewards, 'b', linewidth=0.4)
    #plt.xlabel("steps")
    plt.ylabel("reward", color='b')
    plt.ylim(0,2.)
    plt.grid()
    plt.twinx()
    
    cum = rewards[0]
    reward_cum = [cum]

    for r in rewards[1:]:
        cum = cum + r
        reward_cum.append(cum)
    plt.plot(range(perf['last_step_count']), reward_cum, 'm')
    plt.ylabel("cumulated reward", color='m')
    plt.ylim(0, perf['last_step_count'])
    

    plt.subplot(2,1,2)
    plt.title("Actions")
    plt.plot(range(perf['last_step_count']), actions, 'g', marker='.', markersize=3, linewidth=0.4)
    plt.xlabel("steps")
    plt.ylabel("network action in [-1. ; 1.]")
    plt.ylim(-1, 1)
    plt.grid()

    plt.savefig(test_dir/"rewards.png")

    if display_plots: plt.show()
    plt.close()
    
    print(perf.keys())
    return perf

if __name__ == "__main__":
    # main bloc
    import argparse, sys


    parser = argparse.ArgumentParser()

    group  = parser.add_mutually_exclusive_group()

    group.add_argument('--vehicule', action="store", dest='vehicule', 
                         help="keyword in (cartpole,balancingrobot,miniapterroos)")

    group.add_argument('--configloc', action="store", dest='configloc', 
                         help="location (relative path dir) of the file '..._ppo_copsim.ymaml' to read")

    parser.add_argument('--port', action="store", dest='port', default="20000", type=int, 
                         help="coppeliasim port, default is 20000")

    parser.add_argument('--displayplot', action="store_true", dest='displayplot', default=False, 
                         help="wether to display rewards graph or not")

    parser.add_argument('--quiet', action="store_true", dest='quiet', 
                         help="wether to print informations or not")                         

    parser.add_argument('--alwaystest', action="store_true", dest='alwaystest', default=False, 
                         help="wether to force test computation or not ")                         

    args = parser.parse_args()

    if args.vehicule:
        config_path = f"ai/models/{args.vehicule}/last"
    elif args.configloc:
        config_path = args.configloc
    else:
        parser.print_help()
        sys.exit(1)

    displayplot = args.displayplot
    verbose = not args.quiet
    always_test = args.alwaystest

    #debug print(f"running: test_model('{config_path}', display_plots={displayplot}, verbose={verbose})")

    test_model(config_path, display_plots=displayplot, always_test=always_test, verbose=verbose)
