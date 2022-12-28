# v0 01/07/2021 Mohamed Amine Amghar et Alexis Boisseau

from matplotlib.pyplot import xlim
import numpy as np
from math import pi, radians
from gym import Env, spaces
import random
import sys
sys.path.append("/tmp/temp/")
import root_as_cwd as rcwd
rcwd.prereq()
from coppeliasim_api.env.CopSim import Simulator
from coppeliasim_api.env import sim
from coppeliasim_api.env.constants import COPSIM_DIR
blocking    = sim.simx_opmode_blocking

from balancing_robot.copsim.rewards import computed_reward

dir_copsim = COPSIM_DIR

class BalancingRobotEnv_CopSim(Env):
    def __init__(self,
                 vers: str,
                 scene: str,
                 dt: float,
                 headless: bool   = False,
                 coppelia_sim_port = 19997,
                 veloc_mag: float = None,
                 displ_mag: float = None,
                 verbose: int     = 1,
                 clip_action: bool = False,
                 theta_lim: int = 7,
                 x_lim:str =  0.1,
                 reward: int = 0,
                 open_sim: bool = False):
        '''Parameters of the constructor of BalancingRobotEnv_CopSim:
        :vers:      "v0","v1" or "v2", to specify the BalancingRobot version
                    ('v2' for continuous BalancingRobot).
        :scene:     the CopSIM scene to load (file *.ttt)
        :dt:        timestep in seconds for the CopSIM simulation (default: 10 ms)
        :headless:  to control the 3d rendering display. For computin only,
                    take headless=True. Default value: True.
        :veloc_mag: linear velocity max in m/s for driving the BalancingRobot (default: 0.1 m/s).
        :displ_mag: displacement increment in meters for disp. driving (default : 1 mm).
        :verbose:   verbosity rate (displaying info), possible values:
                    0,1,2. Default value : 1.
        :clip_action: wether to clip or not action when outside the interval value'''

        super().__init__()

        self.dt = dt
        
        self.diameter_m = 9e-2    # diameter in meter

        if veloc_mag is None and displ_mag is None:            
            raise("You must specify one of 'veloc-mag' or 'displ_mag' !")
        elif veloc_mag is not None and displ_mag is not None:
            raise("You must specify ONMY ONE of 'veloc-mag' or 'displ_mag' !")
            
        self.venv = Simulator(coppelia_sim_port, dir_copsim, headless=headless, verbose=1)
        self.venv.start()    # start the CopSIM simulator
        self.venv.load_scene(scene)

        self.joint_r = self.venv.get_object_by_name('Joint_R')
        self.joint_l = self.venv.get_object_by_name('Joint_L')
        self.robot = self.venv.get_object_by_name('Body')


        ###Changement apporté par BalancingBot#################
        from coppeliasim_api.env.simConst import sim_intparam_dynamic_engine
        #from coppeliasim_api.env.simConst import sim_shapefloatparam_mass

        clientID=self.venv.cid
        sim.simxSetIntegerParameter(clientID, sim_intparam_dynamic_engine, 3 , blocking)
        #sim.simxSetFloatingParameter(clientID, sim_shapefloatparam_mass, 20 , blocking)

        ###Changement apporté par BalancingBot#################
        
        
        # Angle at which to fail the episode :
        self.theta_threshold_radians = radians(theta_lim)
        
        ###Changement apporté par BalancingBot#################
        from coppeliasim_api.env.simConst import sim_intparam_dynamic_engine

        clientID=self.venv.cid
        sim.simxSetIntegerParameter(clientID, sim_intparam_dynamic_engine, 3 , blocking)

        ###Changement apporté par BalancingBot#################

        # Angle at which to fail the episode :
        self.theta_threshold_radians = radians(theta_lim)

        # The model will recieve reward while theta between 0 and theta_max_radians
        self.theta_max_radians = radians(theta_lim)

        # Position at which to fail the episode (Gym: 2.4 m):
        self.x_threshold = x_lim      # [m]

        # The model will recieve reward while x between 0 and x_max
        self.x_max = x_lim    # [m]

        self.reward_id = reward

        # Angle limit set to 2*theta_threshold_radians so failing observation
        # is still within bounds:
        high = np.array([self.x_threshold*2, np.finfo(np.float32).max,
                         self.theta_threshold_radians*2, np.finfo(np.float32).max])

        self.min_action = -1.0
        self.max_action =  1.0

        self.action_space = spaces.Box(low=self.min_action,
                                       high=self.max_action,
                                       shape=(1,))
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self.nb_step = 0                  # the number of steps done
        self.steps_beyond_done = None
        self.viewer  = None
        self.state   = None
        self.clip_action = clip_action
        
        self.vers    = "EnvSpec(BalancingRobot-"+vers.lower()+")"
        if self.vers == "EnvSpec(BalancingRobot-v0)" :
            self._max_episode_steps = 200
        elif self.vers == "EnvSpec(BalancingRobot-v1)" :
            self._max_episode_steps = 500
        elif self.vers == "EnvSpec(BalancingRobot-v2)" :
            self._max_episode_steps = 500
        else:
            raise RuntimeError("spec must be 'v0', 'v1' or 'v2'")

        if verbose >= 0 :
            print(f'[BalancingRobot-{vers}] initialized, _max_episode_steps:{self._max_episode_steps}')



        # we define loading as a displacement because this will be the case
        # for our material BalancingRobot with the step motor driving the robot:
        # Defining gym step
        if displ_mag is not None :
            self.displ_mag = displ_mag
            self.step = self.step_displacement #outdated
        elif veloc_mag is not None :
            self.veloc_mag = veloc_mag
            self.step = self.step


    def seed(self, s): self.seed = s

    def getState(self, step=False):
        '''Get the balancing robot state [x, x', theta, theta'], and
           update the attribute 'state'.
           If step is True, request the CopSIM simulation to make a time step.'''

        if step:
            self.venv.step_synchro_simulation()
            self.nb_step += 1
            
        alpha = (self.joint_l.get_joint_position() + self.joint_r.get_joint_position())/2
        #_, alpha_dot_1 = self.joint_l.get_object_velocity()  
        #_, alpha_dot_2 = self.joint_r.get_object_velocity()
        #print(f"Joint_R velocity : {alpha_dot_2[0]:.2f}, {alpha_dot_2[1]:.2f},{alpha_dot_2[2]:.2f}")
        #print(f"Joint_L velocity : {alpha_dot_1[0]:.2f}, {alpha_dot_1[1]:.2f},{alpha_dot_1[2]:.2f}")
        
        #JLC>>> float parameter 2012 : see https://www.coppeliarobotics.com/helpFiles/en/objectParameterIDs.htm
        (alpha_dot_1,) = self.joint_l.get_object_floatParameter(2012)
        (alpha_dot_2,) = self.joint_r.get_object_floatParameter(2012)
        alpha_dot   = (alpha_dot_1 + alpha_dot_2)/2
        #print(f"alpha_dot  : {alpha_dot:.2f} rd/s")
        orientation = self.robot.get_object_orientation()
        lin_vel, ang_vel = self.robot.get_object_velocity() 
        
        R = self.diameter_m/2
        x = R*alpha
        # print("x",x)
        x_dot = R*alpha_dot
        # print("x_dot",x_dot)
        theta = orientation[0]
        # print("theta",theta * 180/np.pi)
        theta_dot = ang_vel[0]
        # print("theta_dot",theta_dot * 180/np.pi)

        self.state = [x, x_dot, theta, theta_dot]

        return np.array(self.state).astype('float32')


    def imposeState(self, state):
        '''To set CopSim positions and velocities as asked'''

        assert len(state) == 4
        x, x_dot, theta, theta_dot = state
        self.joint_r.set_joint_position(theta)
        self.joint_l.set_joint_position(theta)
        self.joint_r.set_joint_target_velocity(theta_dot)
        self.joint_l.set_joint_target_velocity(theta_dot)

        self.state = state
        
        # request the CopSim simulation to make a time step:
        self.venv.step_synchro_simulation()
        self.nb_step += 1
        
    def step(self, action, disturb= True, disturbance_mag=0.1):
        '''Make one time step in the CopSim simulation under the input 'action'.'''
        try :
            assert self.action_space.contains(action), \
                "%r (%s) invalid" % (action, type(action))
        except :
            if self.clip_action: np.clip(action, self.min_action, self.max_action)
        
        ######### Non testé ########
        # a pour but de 
        disturbance = 0

        if(disturb == True):
            rand = random.randint(0,100)
            if( rand < 5): 
                disturbance = self.theta_max_radians*disturbance_mag

        angular_veloc = self.veloc_mag*float(action) + disturbance

        ############################

        # step
        self.joint_r.set_joint_target_velocity(angular_veloc)
        self.joint_l.set_joint_target_velocity(angular_veloc)
        self.venv.step_synchro_simulation()
        self.nb_step += 1
        
        _, alpha_dot_1 = self.joint_l.get_object_velocity()  
        _, alpha_dot_2 = self.joint_r.get_object_velocity()
        # print("Joint_R velocity : ",alpha_dot_2[2])
        # print("Joint_L velocity : ",alpha_dot_1[2])

        # observe (sets the attribute self.state):
        self.getState()
        done, reward = self.compute_done_reward(action)
        return np.array(self.state), reward, done, {}

    def compute_done_reward(self, action):
        
        x, x_dot, theta, theta_dot = self.state

        done = abs(theta) > self.theta_threshold_radians 
        done = done or abs(x) > self.x_threshold
        #done = done or self.joint_l.get_object_velocity() != self.joint_r.get_object_velocity()
        done = done or (self._max_episode_steps is not None and self.nb_step >= self._max_episode_steps)


        #JLC: improved reward computation:
        if not done:
            # if self.prev_action is None: self.prev_action = action
            reward = computed_reward(self, action)
            # self.previous_action = action

        else:
            # pole has fell or cart is outside limits:
            if self.steps_beyond_done is None:
                # Pole just fell!
                self.steps_beyond_done = 0
                reward = 1.0
            else:
                if self.steps_beyond_done == 0:
                    print("You are calling 'step()' even though this environment "+\
                          "has already returned done = True. \nYou should always "+\
                          "call 'reset()' once you receive 'done = True' "+\
                          "-- any further steps are undefined behavior.")
                self.steps_beyond_done += 1
                reward = 0.0

        return done, reward

    def reset(self, dt=None, theta_deg=None):

        print(f"BalancingRobot reset - np steps : {self.nb_step}")
        self.venv.stop_simulation()
        self.nb_step = 0

        if dt is not None : self.dt = dt
        self.venv.start_synchro_simulation(self.dt)
        if theta_deg is None:
            self.state = np.random.uniform(low=-0.05, high=0.05, size=(4,))
        else:
            self.state = [0, 0, np.radians(theta_deg), 0]
            
        x, x_dot, theta, theta_dot = self.state
        self.state = 0, 0, theta, 0
        # set positions and velocities in CopSIM:
        self.imposeState(self.state)
        self.steps_beyond_done = None
        
        return np.array(self.state)


    def close(self):
        self.venv.stop_simulation()
        self.venv.end()

    def runVeloc(self, nb_step=200, velocity=0.1, dt=None):
        '''To run the BalancingRobot with velocity imposed.'''

        if dt is not None : self.dt = dt
        print(f"Ready to run {nb_step} steps of velocity with timestep={self.dt}")
        States = []
        state = self.reset(self.dt, theta_deg=0)
        print("state:",state)

        for _ in range(nb_step):
            self.joint_r.set_joint_target_velocity(velocity)
            self.joint_l.set_joint_target_velocity(velocity)
            
            self.venv.step_synchro_simulation()
            self.nb_step += 1            
            # observe (sets the attribute self.state):
            state = self.getState()            
            States.append(state)
            
        return np.array(States)

    def plot(self, data, y1_max=None, y2_max=None, y3_max=None, y4_max=None, title=""):

        M = np.array(data)
        if M.ndim != 2 or M.shape[1] !=  4 :
            print("plot :data must be a numpy.array with shape=(x,4)... sorry!")
            return

        import matplotlib.pyplot as plt
        
        nb_step = M.shape[0]
        T = np.arange(nb_step)*self.dt

        fig = plt.figure(title, figsize=(13,6))
        plt.subplots_adjust(wspace=0.4)
        
        #fig.suptitle(title)
        ax1 = plt.subplot(121,label='ax1')
        plt1 = ax1.plot(T, M[:,0]*1000,'.-b', label='x [mm]')
        y1max = np.abs(M[:,0]*1000).max()*1.1 if y1_max is None else y1_max
        ax1.set_ylim(-y1max, y1max)
        ax1.set_xlabel('time [s]')
        ax1.tick_params(axis='y', labelcolor='b')
        ax1.set_title('Linear displacement')
        ax1.grid()
        ax2 = ax1.twinx()
        plt2 = ax2.plot(T,M[:,1]*1000,'.-m', label="x' [mm/s]")
        y2max = np.abs(M[:,1]*1000).max()*1.1 if y2_max is None else y2_max
        ax2.set_ylim(-y2max, y2max)
        ax2.tick_params(axis='y', labelcolor='m')
        all_plots = plt1+plt2
        labs = [p.get_label() for p in all_plots]
        ax1.legend(all_plots, labs, loc='best', fontsize=10)

        ax3 = plt.subplot(122,label='ax1')
        plt3 = ax3.plot(T,np.degrees(M[:,2]),'.-g',label=r'$\theta$'+' [°]')
        y3max = np.abs(np.degrees(M[:,2])).max()*1.1 if y3_max is None else y3_max
        ax3.set_ylim(-y3max, y3max)
        ax3.set_xlabel('time [s]')
        ax3.tick_params(axis='y', labelcolor='g')
        ax3.set_title('Angular displacement')
        ax3.grid()
        ax4 = ax3.twinx()
        plt4 = ax4.plot(T,np.degrees(M[:,3]),'.-r',label=r"$\theta'$"+ ' [°/s]')
        y4max = np.abs(np.degrees(M[:,3])).max()*1.1 if y4_max is None else y4_max
        ax4.set_ylim(-y4max, y4max)
        ax4.tick_params(axis='y', labelcolor='r')
        all_plots = plt3+plt4
        labs = [p.get_label() for p in all_plots]
        ax3.legend(all_plots, labs, loc='best', fontsize=10);
        
        plt.show()
        
if __name__ == '__main__':

    # since september 2020 the source tree is stored on the ENSAM gitlab server:
    # the editor VisualStudio code is configured tu run python programmes from the <project_root> directory : 
    # all the pathes are relative to this directory.

    # scene path relative to the 'project root' directory:
    #scene = "balancing_robot/copsim/Balancing-robot.ttt"
    scene = "balancing_robot/copsim/Balancing-robot-no-batteries(all-weights-modified).ttt"
    
    env   = BalancingRobotEnv_CopSim('v2', scene=scene, headless=False, veloc_mag=0.2, dt=0.075)    
    print(f"starting new simulation with <{scene}>")

    # 1/ run in the 'free run' mode:

    # 2/ run in displacement mode: displ. step=0.2 cm, dt=0.02s => speed=10cm/sec
    #data = env.runDispl(nb_step=250, dx=0.002, dt=0.02)
    #title = '250 steps of displ. run with dx:0.2 cm and dt:20 ms'
    #env.plot(data, title=title)

    # 3/ run in velocity mode
    #data = env.runVeloc(nb_step=100, velocity=4, dt=0.05)
    #title = '100 steps of free run with dt:70 ms'
    #env.plot(data, title=title)
    steps, veloc, dt = 100, np.pi, 0.1
    data = env.runVeloc(nb_step=steps, velocity=veloc, dt=0.05)
    title = f'{steps} steps of velocity run with velocity:{veloc:.2f} rad/sec and dt:{dt} ms'
    env.plot(data, title=title)

    env.close()
