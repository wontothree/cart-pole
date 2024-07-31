import numpy as np
import random
import gym
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.optimizers import Adam
from collections import deque
import matplotlib.pyplot as plt

# 하이퍼파라미터
rho = 0.9  # 학습률
lamda = 0.99  # 할인율
eps = 0.9  # 엡실론
eps_decay = 0.999  # 엡실론 감소 비율
batch_siz = 64  # 리플레이 메모리에서 샘플링할 배치 크기
n_episode = 100  # 학습에 사용할 에피소드 개수

# 신경망 설계 함수
def deep_network():
    mlp = Sequential()
    mlp.add(Dense(32, input_dim=env.observation_space.shape[0], activation='relu'))
    mlp.add(Dense(32, activation='relu'))
    mlp.add(Dense(env.action_space.n, activation='linear'))
    mlp.compile(loss='mse', optimizer=Adam(learning_rate=rho))
    return mlp

# DQN 학습
def model_learning():
    mini_batch = random.sample(D, batch_siz)
    
    state_batch = np.array([experience[0] for experience in mini_batch])
    action_batch = np.array([experience[1] for experience in mini_batch])
    reward_batch = np.array([experience[2] for experience in mini_batch])
    next_state_batch = np.array([experience[3] for experience in mini_batch])
    done_batch = np.array([experience[4] for experience in mini_batch])
    
    target = model.predict(state_batch)
    target_next = model.predict(next_state_batch)
    
    for i in range(batch_siz):
        if done_batch[i]:
            target[i][action_batch[i]] = reward_batch[i]
        else:
            target[i][action_batch[i]] = reward_batch[i] + lamda * np.amax(target_next[i])
    
    model.fit(state_batch, target, batch_size=batch_siz, epochs=1, verbose=0)

# env = gym.make("CartPole-v0")
env = gym.make("CartPole-v1")

model = deep_network()
D = deque(maxlen=2000)
scores = []
max_steps = env.spec.max_episode_steps

# 신경망 학습
for i in range(n_episode):
    s = env.reset()
    if isinstance(s, tuple):
        s = s[0]
    long_reward = 0

    while True:
        r = np.random.random()
        eps = max(0.01, eps * eps_decay)
        if r < eps:
            a = np.random.randint(0, env.action_space.n)
        else:
            q = model.predict(np.reshape(s, [1, env.observation_space.shape[0]]))
            a = np.argmax(q[0])
        
        s1, r, done, _, _ = env.step(a)
        if done and long_reward < max_steps - 1:
            r = -100
        
        if isinstance(s1, tuple):
            s1 = s1[0]
        
        D.append((s, a, r, s1, done))
        
        if len(D) > batch_siz * 3:
            model_learning()
        
        s = s1
        long_reward += r
        
        if done:
            long_reward = long_reward if long_reward == max_steps else long_reward + 100
            print(i, "번째 에피소드의 점수:", long_reward)
            scores.append(long_reward)
            break
    if i > 10 and np.mean(scores[-5:]) > (0.95 * max_steps):
        break

# 신경망 저장
model.save('./cartpole_by_DQN.keras')
env.close()

# 결과 시각화
plt.plot(range(1, len(scores) + 1), scores)
plt.title('DQN scores for CartPole-v1')
plt.ylabel('Score')
plt.xlabel('Episode')
plt.grid()
plt.show()
