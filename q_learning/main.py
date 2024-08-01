import gym
import numpy as np

from functions_for_q_learning import Q_Learning

# 환경 생성
env = gym.make('CartPole-v1')
(state, _) = env.reset()

# 상태 이산화 파라미터 설정
upperBounds = env.observation_space.high
lowerBounds = env.observation_space.low
cartVelocityMin = -3
cartVelocityMax = 3
poleAngleVelocityMin = -10
poleAngleVelocityMax = 10
upperBounds[1] = cartVelocityMax
upperBounds[3] = poleAngleVelocityMax
lowerBounds[1] = cartVelocityMin
lowerBounds[3] = poleAngleVelocityMin

numberOfBinsPosition = 30
numberOfBinsVelocity = 30
numberOfBinsAngle = 30
numberOfBinsAngleVelocity = 30
numberOfBins = [numberOfBinsPosition, numberOfBinsVelocity, numberOfBinsAngle, numberOfBinsAngleVelocity]

# Q-Learning 파라미터 설정
alpha = 0.1
gamma = 1
epsilon = 0.2
numberEpisodes = 100000


########################################################################################################################################

# # Q-Learning 객체 생성 및 학습
# Q1 = Q_Learning(env, alpha, gamma, epsilon, numberEpisodes, numberOfBins, lowerBounds, upperBounds)
# Q1.simulateEpisodes()

# # Q-테이블 저장
# np.save('./q_table.npy', Q1.Qmatrix)

# # 학습된 전략 시뮬레이션
# (obtainedRewardsOptimal, env1) = Q1.simulateLearnedStrategy()

# env1.close()

########################################################################################################################################



# Q-테이블 로드 후 새로운 Q-Learning 객체 생성
Q1_loaded = Q_Learning(env, alpha, gamma, epsilon, numberEpisodes, numberOfBins, lowerBounds, upperBounds)
Q1_loaded.Qmatrix = np.load('/Users/kevinliam/Desktop/Kevin’s MacBook Air/development/cart-pole/q_learning/q_table.npy')

# 학습된 전략 시뮬레이션
(obtainedRewardsOptimal, env1) = Q1_loaded.simulateLearnedStrategy()

# 환경 종료
env1.close()
