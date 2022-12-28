import os, sys, time, shutil, yaml, pathlib

from ai.src.run.constants import OUTPUT_DIR, EXPERIMENT_CONFIG_FILENAME, ENVIRONMENT_CONFIG_FILENAME

#from cartpole.copsim.CartpoleEnv_CopSim import CartPoleEnv_CopSim
from balancing_robot.copsim.BalancingRobotEnv_CopSim import BalancingRobotEnv_CopSim

from stable_baselines3.common.callbacks import CheckpointCallback


def train_model(cfg_file: str, copsim_port=20000, training_dir=None):
    """
    Train a RL model on a given environment as described in cfg_file,
    and save the results and config for reproductibility

    :param cfg_file: path of the yaml config file
    :return: None
    """
    print(cfg_file)
    # import parameters from config files
    with open(cfg_file, 'r') as f:
        cfg = yaml.safe_load(f.read())

    try:
        environment  = cfg['env']['environment']
        cfg_file_env = cfg['env']['cfg_env']
        scene        = cfg['env']['scene']

        out_path     = cfg['train']['output_model_path']
        agent_type   = cfg['train']['agent_type']
        policy       = cfg['train']['policy']
        tot_steps    = cfg['train']['total_timesteps']
        save_freq    = cfg['train']['save_freq']
        nb_steps     = cfg['train']['n_steps']
        b_size       = cfg['train']['batch_size']
        nb_epochs    = cfg['train']['n_epochs']
        seed         = cfg['train']['seed']
        headless     = cfg['train']['headless']
        
    except:
        raise RuntimeError("Parametres missing in file <{cfg_file}>")
    
    with open(cfg_file_env, 'r') as f:
        cfg_env = yaml.safe_load(f.read())

    # import agent
    if  agent_type == 'PPO':
        from stable_baselines3 import PPO as agent
    else:
        raise Exception("Not implemented agent : <{agent_type}>")

    if training_dir is None:
        experiment_time = time.localtime()
        # prepare directory for output
        experiment_id = "_".join([environment, agent_type,
                                time.strftime("%y-%m-%d_%H-%M-%S", experiment_time)])
        training_dir = pathlib.Path(out_path)/experiment_id
    
    training_dir = pathlib.Path(training_dir)
    training_dir.mkdir(parents=True, exist_ok=True)
    

    # Create env for training
    if environment == 'CartPoleEnv_CopSim':

        veloc     = cfg_env['velocity']
        dt        = cfg_env['dt']
        version   = cfg_env['version'] 
        theta_lim = cfg_env['theta_lim']
        x_lim     = cfg_env['x_lim']   
        reward    = cfg_env['reward']       

        env = CartPoleEnv_CopSim(version, 
                                 headless=headless, 
                                 scene=scene, 
                                 verbose=0, 
                                 veloc_mag=veloc, 
                                 dt=dt,
                                 theta_lim_deg = theta_lim,
                                 x_lim = x_lim,
                                 reward=reward,
                                 coppelia_sim_port=copsim_port)

        shutil.copyfile('cartpole/copsim/rewards.py', training_dir/'rewards.py')


    elif environment == 'BalancingRobotEnv_CopSim':

        veloc   = cfg_env['velocity']
        dt      = cfg_env['dt']
        version = cfg_env['version']
        theta_lim = cfg_env['theta_lim']
        x_lim = cfg_env['x_lim']   
        reward= cfg_env['reward']     
        
        env = BalancingRobotEnv_CopSim(version, 
                                       headless=headless, 
                                       scene=scene, 
                                       verbose=0,
                                       veloc_mag=veloc, 
                                       dt=dt,
                                       theta_lim = theta_lim,
                                       x_lim = x_lim,
                                       reward=reward,
                                       coppelia_sim_port=copsim_port)

        shutil.copyfile('balancing_robot/copsim/rewards.py', training_dir/'rewards.py')
    else:
        raise Exception("Not implemented environment: <{environment}>")

    
    # copy precious files in experiment_dir
    shutil.copyfile(cfg_file, training_dir/EXPERIMENT_CONFIG_FILENAME)
    shutil.copyfile(cfg_file_env, training_dir/ENVIRONMENT_CONFIG_FILENAME)
    
    # prepare agent for training
    model = agent(policy, 
                  env, 
                  n_epochs=nb_epochs,
                  n_steps=nb_steps,
                  batch_size=b_size,
                  seed=seed,
                  tensorboard_log=training_dir,
                  verbose=1)

    checkpoint_callback = CheckpointCallback(save_freq=save_freq, 
                                             save_path=training_dir/'ZIP')

    # train agent
    t0 = time.time()

    model.learn(total_timesteps=tot_steps, 
                callback=checkpoint_callback)
    
    t = int(time.time()-t0)
    h = int(t//3600)
    m = int((t - h*3600)//60)
    print(f"Training elapsed time: {h:02d}h {m:02d}m")
  
    # save trained agent
    target_zip = os.path.join(training_dir, 'ZIP', 'model.zip')
    print(f"saving trained model in <{target_zip}>")
    model.save(target_zip)
    env.close()

    os.system(f"cd {out_path} && ln -s -f {os.path.basename(training_dir)} last")

    return training_dir
    

if __name__ == "__main__":
    # bloc main
    import argparse, sys
    parser = argparse.ArgumentParser()
    group  = parser.add_mutually_exclusive_group()

    group.add_argument('--vehicule', action="store", dest='vehicule', 
                         help="keyword in (cartpole,balancingrobot,miniapterros)")
    group.add_argument('--config', action="store", dest='config', 
                         help="relative path name of the file '..._ppo_copsim.ymaml'")

    parser.add_argument('--port', action="store", dest='port', default="20000", type=int, 
                         help="coppeliasim port, default is 20000")
    
    parser.add_argument('--traindir', action="store", dest='traindir', 
                         help="Optional, the relative pathname of the training directory")
    
    args = parser.parse_args()

    if args.vehicule:
        config_path = f"ai/config/{args.vehicule}_ppo_copsim.yaml"
    elif args.config:
        config_path = args.config
    else:
        parser.print_help()
        sys.exit(1)

    traindir = None
    if args.traindir:
        traindir = args.traindir
       
    print(f"running: train_model('{config_path}', coppsim_port={args.port}, training_dir='{traindir}')")

    train_model(config_path, copsim_port=args.port, training_dir=traindir)
