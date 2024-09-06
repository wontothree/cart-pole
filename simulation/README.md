# Simulation

# Dependencies

```bash
git clone https://github.com/openai/gym.git
```

```bash
pip install pygame
```

# Changes

- gym/envs/classic_control/cartpole.py에 있는 CartPoleEnv 클래스에서 terminal 조건을 변경했다.

- action_space를 연속적으로 변경했다.

```py
self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
```

- 제어 주기를 변경한다.

```py
self.tau = 0.1 # 0.02  # seconds between state updates
```

```py
self.gravity = 9.81
self.masscart = 0.123
self.masspole = 0.089
self.total_mass = self.masspole + self.masscart
self.length = 0.2  # actually half the pole's length
self.polemass_length = self.masspole * self.length
self.force_mag = 1.0
self.tau = 0.01 # 0.02  # seconds between state updates
self.kinematics_integrator = "euler"
```