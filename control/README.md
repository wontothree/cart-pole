# Nonlinear Model Predictive Control vs Deep Reinforcement Learning

[Nonlinear Model Predictive Control](https://wontothree.github.io/cartpole/mpc/)

- Input : initial state, current state
- Output : next control input

# Functions

|Function|Description|
|---|---|
|set_reference_state||
|set_reference_control||
|set_initial_state||
|set_current_state||
|define_dynamic_model||
|define_constraints||
|define_cost_function||
|solve||
|||

|Function Name|Input|Return|Description|
|---|---|---|---|
|calc_continuous_dynamics|||nonlinear state space model의 함수를 반환한다. $\dot{x} = f(x, u)$에서 f를 반환한다. 이 함수는 continuous state space model을 반환한다.|
|calc_discrete_dynamics|||4차 룽게-쿠타(RK4) 방법을 사용하여 주어진 상태와 제어 입력에 대해 시스템의 다음 상태를 예측하는 함수를 반환한다. Runge-Kutta 4차 방법은 discrete state space model을 생성한다. calc_continuous_dynamics를 이용한다.|
|calc_state_cost|||state cost를 계산한다.|
|calc_terminal_cost|||terminal cost를 계산한다.|
|solve_nlp|||nonlinear programming solver. calc_discrete_dynamics, calc_state_cost, calc_terminal_cost를 이용한다.|
|calc_optimal_control|||최적의 제어 입력을 계산한다. solve_nip, update_next_state을 이용한다.|
|run_mpc||||
|update_next_state|||주어진 상태와 제어 입력을 기반으로 시스템의 동역학을 수치적으로 통합하여 다음 상태를 계산한다. calc_continuous_dynamics를 이용한다. 여기서는 센서를 통해 상태 실측값 대신에 사용된다.|

# Variables

|Variable Name|Initial Value|Description|
|---|---|---|
|grivitation_accel|9.81|gravitational acceleration (m/s^2)|
|cart_mass|0.123|mass of cart (kg)|
|pole_mass|0.089|mass of pole (kg)|
|pole_inertia|||
|pole_length|0.4|length of pole (m)|
|states_dim|4||
|ctrls_dim|1||
||||

# NMPC Dependencies

[CasADi](https://web.casadi.org/)

```bash
pip install casadi
```
