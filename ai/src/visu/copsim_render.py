import gym
from stable_baselines3.common.base_class import BaseAlgorithm


def model_render(model: BaseAlgorithm, 
                 env: gym.Env, 
                 output_dir: str = "ai/out", 
                 deterministic: bool = True,
                 steps_nb: int = 1000):
    """
    Display how a model perform on a given environment.
    Notice that the Python API for Coppelia Sim does not easily allow to
    automatically save the video of a simulation, hence at the moment
    it must be done manually during the run.

    :param model: instance of a stable_baselines3 model
    :param env: instance of a gym environment
    :param output_dir: where to save the output video
    :param steps_nb: nb of simulation steps to display and save
    :param deterministic: wether to add or not some noise to test with an harder task
    :return: None
    """
    # display the results of the training ###
    obs = env.reset()
    done, step_count = False, 0

    # at each step, will make the robot predict the next movement, and update the environment consequently
    while not done and step_count < steps_nb:
        # deterministic=False add random noise (random wind ?) to test with an harder task
        action, _ = model.predict(obs, deterministic=deterministic)
        obs, reward, done, info = env.step(action)
        step_count += 1
        if step_count % 10 == 0 : print("\r", step_count, end="")
        
    print("Episode finished after {} steps".format(step_count + 1))
    env.close()
