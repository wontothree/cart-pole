import gym

# env = CartPoleEnv(render_mode="human")  # render_mode를 "human"으로 설정하여 화면에 시각적으로 출력
env=gym.make('CartPole-v1',render_mode='human')
observation, info = env.reset()

for _ in range(1000):
    action = env.action_space.sample()
    observation, reward, done, info, _ = env.step(action)
    print(f"Observation: {observation}, Reward: {reward}, Terminated: {done}")

    env.render()
    if done:
        break

env.close()

