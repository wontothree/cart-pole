from gym.envs.classic_control.cartpole import CartPoleEnv

if __name__ == "__main__":
    # initialize CartPoleEvc
    env = CartPoleEnv(render_mode="human")  # human mode rendering
    states, _ = env.reset()

    for _ in range(1000):
        # control
        action = env.action_space.sample()

        # one step setting
        states, reward, terminated, _, _ = env.step(action)

        print(f"Current State: {states}, Action: {action}, Reward: {reward}, Done: {terminated}")

        # rendering
        env.render()

        if terminated:
            # reset
            states, _ = env.reset()

    # close environment
    env.close()
