import yaml, multiprocessing, subprocess, pathlib, datetime, time, os

from ai.src.run.constants import CONFIG_DIR

def build_yaml_files_and_arguments_list(vehicule, ppo_params, env_params, start_port=20000):
    
    port = start_port
    
    config_env_name = vehicule + "_copsim.yaml"
    config_ppo_name = vehicule + "_ppo_copsim.yaml"

    config_ppo_path = os.path.join(CONFIG_DIR, config_ppo_name)
    config_env_path = os.path.join(CONFIG_DIR, vehicule+'_copsim', config_env_name)

    path_to_train_python_script = "ai/src/run/train_model_copsim.py"

    cmd_list = []

    for seed in env_params['seeds']:
        for theta in env_params['theta_deg']:
            for x in env_params['x_lim_m']:
                for veloc in env_params['velocity']:
                    for reward in env_params['rewards']:

                        port += 1

                        # Open the <ppo config yaml> file, change the name of the <env config ymal> file
                        # and write it back with a custom name:
                        #print(f"reading <{config_ppo_path}>")
                        with open(config_ppo_path, 'r') as f: 
                            ppo_cfg = yaml.safe_load(f.read())
                        
                        custom_cfg_env_name = f"{port}_{vehicule}_copsim.yaml"
                        custom_cfg_env_path = os.path.join(CONFIG_DIR, vehicule+'_copsim', custom_cfg_env_name)

                        #print(f"custom_cfg_env_path: <{custom_cfg_env_path}>")
                        ppo_cfg['env']['cfg_env'] = custom_cfg_env_path
                        ppo_cfg['train']['seed']  = seed
                        custom_cfg_ppo_name = f"{port}_{vehicule}_ppo_copsim.yaml"
                        custom_cfg_ppo_path = os.path.join(CONFIG_DIR, custom_cfg_ppo_name)

                        #print(f"writing  <{custom_cfg_ppo_path}>")
                        with open(custom_cfg_ppo_path, "w", encoding="utf8") as f: 
                            yaml.dump(ppo_cfg, f, default_flow_style=False)
                        
                        # Now prepare the env parameters dictionnary and write the custom <config env yaml> file:
                        #print(f"reading <{config_env_path}>")
                        with open(config_env_path) as f: 
                            env = yaml.safe_load(f.read())                        
                        env["theta_lim"] = theta
                        env["x_lim"]     = x
                        env["velocity"]  = veloc
                        env["reward"]    = reward       
                        #print(f"writing <{custom_cfg_env_path}>")
                        with open(custom_cfg_env_path, "w", encoding="utf8") as f: 
                            yaml.dump(env, f, default_flow_style=False)

                        args = ['python', 
                                path_to_train_python_script,
                                '--config', custom_cfg_ppo_path,
                                '--port',  str(port)]               
                        cmd_list.append(args)
    return cmd_list

def run_training_multithread(vehicule, cmd_list, nb_max_proc=None):

    if nb_max_proc is None:
        nb_cores = multiprocessing.cpu_count()
        print(f"number of cores : {nb_cores}")
        nb_subprocess = nb_cores - 2
    else:
        print(f"number of processes requested: {nb_max_proc}")
        nb_subprocess = nb_max_proc


    process_list  = [0]*nb_subprocess # The running processes queue
    proc_dict = {}                # The process parameters dictionnary

    s, still_working, t0 = 0, False, int(time.time())

    # some constants needed to build paths:
    config_ppo_name = vehicule + "_ppo_copsim.yaml"
    config_ppo_path = os.path.join(CONFIG_DIR, config_ppo_name)
    with open(config_ppo_path, 'r') as f: ppo_cfg = yaml.safe_load(f.read())
    environment  = ppo_cfg['env']['environment']
    agent_type   = ppo_cfg['train']['agent_type']
    out_path     = ppo_cfg['train']['output_model_path']

    # the loop to launch each training process in a thread:
    while s < len(cmd_list) or still_working:
        still_working = False
        #
        # Scan the processList queue to see wether to create a new process or 
        # to wait for an existing process termination:
        #
        for i, sub_proc in enumerate(process_list):
            if sub_proc == 0:  
                # Processing a new process:
                if s < len(cmd_list):
                    
                    experiment_time = time.localtime()
                    experiment_id = "_".join([environment, agent_type,
                                            time.strftime("%y-%m-%d_%H-%M-%S", experiment_time)])
                    training_dir = pathlib.Path(out_path)/experiment_id
                    cmd = cmd_list[s]
                    training_dir_str = str(training_dir)
                    cmd.extend(['--traindir', training_dir_str])

                    training_dir.mkdir(parents=True, exist_ok=True)
                    redirection_file = open(os.path.join(training_dir_str, "training.log"), 'w')
                    #print("cmd:", cmd)
                    proc = subprocess.Popen(cmd, 
                                            stdout=redirection_file, 
                                            stderr=redirection_file,
                                            env=dict(os.environ, PYTHONPATH=f'{os.getcwd()}:$PYTHONPATH'))
                    print(f"process <{i}:{proc.pid}> starts in <{training_dir_str}>")
                    process_list[i] = proc
                    s += 1
                    still_working = True
                    proc_dict[proc.pid] = [training_dir, redirection_file, int(time.time()), cmd]
                    time.sleep(1)
            else: 
                # Processing an existing process:
                if sub_proc.poll() != None:
                    # the process has finished:
                    # retrieve info associated to the process:
                    training_dir, redirection_file, tStart, cmd = proc_dict.get(sub_proc.pid, [None, None, None, None])
                    assert(training_dir is not None)
                    duration=  str(datetime.timedelta(seconds=int(time.time())-tStart))
                    # move or close some files, as needed:
                    redirection_file.close()
                    print(f"process {sub_proc.pid} has finished (duration: {duration} sec")
                    
                    # remove remaining unused files:
                    custom_cfg_ppo_path = cmd[3]
                    if custom_cfg_ppo_path.endswith('ppo_copsim.yaml') and os.path.exists(custom_cfg_ppo_path):
                        with open(custom_cfg_ppo_path, 'r') as f: 
                                ppo_cfg = yaml.safe_load(f.read())
                        custom_cfg_env_path = ppo_cfg['env']['cfg_env']
                        os.remove(custom_cfg_ppo_path)
                        os.remove(custom_cfg_env_path)
                            
                    # mark the process as finished in list processList:
                    process_list[i] = 0
                else:
                    still_working = True

        time.sleep(10)

    totDuration=str(datetime.timedelta(seconds=int(time.time())-t0))
    print(f"End of trainings! total duration: {totDuration}")

#Sorts the models by their grade, and prints them
def print_ranking(results):
    a = sorted(results.items(), key=lambda x: x[1], reverse=True)  
    for (model, grade) in a:
        print("Model " + model + " got the grade :   " + str(grade))
