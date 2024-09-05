from gym.envs.classic_control.cartpole import CartPoleEnv



if __name__ == "__main__":
    # initialize CartPoleEvc
    env = CartPoleEnv(render_mode="human")  # human mode rendering
    obs, _ = env.reset()

    for _ in range(1000):
        # control
        action = env.action_space.sample()

        # one step setting
        obs, reward, terminated, _, _ = env.step(action)
        print(obs)

        # rendering
        env.render()

        if terminated:
            # reset
            obs, _ = env.reset()

    # close environment
    env.close()
