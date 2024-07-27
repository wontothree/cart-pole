
# tested on     
# gym==0.26.2
# gym-notices==0.0.8

#gymnasium==0.27.0
#gymnasium-notices==0.0.1

# classical gym 
import gym
# instead of gym, import gymnasium 
#import gymnasium as gym
import numpy as np
import time


# create environment / human is for animation simulation
env=gym.make('CartPole-v1',render_mode='human')

# # reset the environment, 
# # returns an initial state
# # states are cart position, cart velocity, pole angle, pole angular velocity
# (state, _)=env.reset()

# # render the environment
# env.render()

# environment information
# env.step(0) # push cart in left direction
# print(env.observation_space) # observation space limits
# print(env.observation_space.high) # upper limit
# print(env.observation_space.low) # lower limit
# print(env.action_space) # action space
# print(env.spec) # all the specs
# print(env.spec.max_episode_steps) # maximum number of steps per episode
# print(env.spec.reward_threshold) # reward threshold per episode

# simulate the environment
episodeNumber=50
timeSteps=100
for episodeIndex in range(episodeNumber):
    initial_state=env.reset()
    print(episodeIndex)
    env.render()
    appendedObservations=[]
    for timeIndex in range(timeSteps):
        print(timeIndex)
        random_action=env.action_space.sample()
        observation, reward, terminated, truncated, info =env.step(random_action)
        appendedObservations.append(observation)
        time.sleep(0.1)
        if (terminated):
            time.sleep(1)
            break

# close the environment
env.close()   
