# import gym
# import numpy as np
# import time

# # create environment
# env=gym.make('CartPole-v1',render_mode='human')

# # simulate the environment
# episodeNumber=50
# timeSteps=100
# for episodeIndex in range(episodeNumber):
#     initial_state=env.reset()
#     print("Episode Index : " + str(episodeIndex))
#     env.render()

#     appendedObservations=[]
#     for timeIndex in range(timeSteps):
#         print("Time Step : " + str(timeIndex))
#         random_action=env.action_space.sample()
#         observation, reward, terminated, truncated, info =env.step(random_action)
#         appendedObservations.append(observation)
#         time.sleep(0.1)
#         if (terminated):
#             time.sleep(1)
#             break

# # close the environment
# env.close()

