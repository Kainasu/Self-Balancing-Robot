import gym, os
from stable_baselines3.common.base_class import BaseAlgorithm
import gym.wrappers as wrap
from stable_baselines3.common.monitor import Monitor


def model_render(model: BaseAlgorithm, 
                 env: gym.Env, 
                 output_dir: str = "ai/out", 
                 step_nb: int = 1000,
                 deterministic: bool = True):
    """
    Display and save how a model perform on a given environment

    :param model: instance of a stable_baselines3 model
    :param env: instance of a gym environment
    :param output_dir: where to save the output video (def: "ai/out")
    :param step_nb: nb of simulation steps to display (def: 100)
    :param deterministic: wether to add add random noise to test the env with an harder task (def: True)
    :return: None
    """
    # save or display the results of the training ###
    env._max_episode_steps = step_nb

    # create output directory if needed:
    if not os.path.exists(output_dir): os.makedirs(output_dir)

    # wrapper to monitor and save in a movie what is happening
    env = wrap.Monitor(env, output_dir, force=True)
    #env = Monitor(env, output_dir)

    obs = env.reset()
    done, step_count = False, 0
    # at each step, will make the robot predict the next movement, and update the environment consequently
    print("   step  action done")
    while not done and step_count < step_nb:
        action, _states = model.predict(obs, deterministic=deterministic)
        print(f"{step_count:6d}     {action}    {done}")
        obs, reward, done, info = env.step(action)
        step_count += 1

    # output is saved in the current working directory
    env.close()
    