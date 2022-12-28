import numpy as np
def computed_reward(self, action):
        """
        it chooses the right reward function to call using the self.reward_id
        """
        if self.reward_id == 0:
                return reward(self, action)
                
        elif self.reward_id == 1:
                return reward_1(self, action)
                

def reward(self, action):
        
        x, x_dot, theta, theta_dot = self.state

        reward = 1.0 
        reward -= 5 if (abs(x) > self.x_max) else -5 * ( self.x_max - abs(x) ) / self.x_max    
        reward -= 5 if (abs(theta) > self.theta_max_radians) else -5 * ( self.theta_max_radians - abs(theta) ) / self.theta_max_radians
        reward /= 9        

        return reward

def reward_1(self, action):
        """"
        Same as reward function, but usong cos functions and penalising the agent when using high velocities
        """
        x, x_dot, theta, theta_dot = self.state

        reward = 1.0  # robot is still alive and running 
        reward += 5 * np.cos(abs(x)*np.pi/self.x_max)*(abs(x)<=self.x_threshold)    
        reward += 5 * np.cos(abs(theta)*np.pi/self.theta_max_radians)*(abs(theta)<=self.theta_threshold_radians)
        reward += np.cos(abs(x_dot)*np.pi/self.veloc_mag)    # we want the robot to not reach max velocity in order to not loose acceleration power
        reward /= 12

        return reward