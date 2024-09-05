# Nonlinear Model Predictive Control vs Deep Reinforcement Learning

|Return|Function Name|Input|Description|
|---|---|---|---|
||make_state_space_equations||nonlinear state space model의 함수를 반환한다. $\dot{x} = f(x, u)$에서 f를 반환한다.|
||predict_next_state||4차 룽게-쿠타(RK4) 방법을 사용하여 주어진 상태와 제어 입력에 대해 시스템의 다음 상태를 예측한다.|
||update_next_state||주어진 상태와 제어 입력을 기반으로 시스템의 동역학을 수치적으로 통합하여 다음 상태를 계산한다.|
||compute_state_cost||state cost를 계산한다.|
||compute_terminal_cost||terminal cost를 계산한다.|
||solve_nlp||nonlinear programming solver|
||comput_optimal_control||최적의 제어 입력을 계산한다.|
||update_figure||visualization|

|Return|Function Name|Input|Description|
|---|---|---|---|
||make_f||nonlinear state space model의 함수를 반환한다. $\dot{x} = f(x, u)$에서 f를 반환한다.|
||make_F_RK4||4차 룽게-쿠타(RK4) 방법을 사용하여 주어진 상태와 제어 입력에 대해 시스템의 다음 상태를 예측한다.|
||make_integrator||주어진 상태와 제어 입력을 기반으로 시스템의 동역학을 수치적으로 통합하여 다음 상태를 계산한다.|
||compute_state_cost||state cost를 계산한다.|
||compute_terminal_cost||terminal cost를 계산한다.|
||make_nlp||nonlinear programming solver|
||comput_optimal_control||최적의 제어 입력을 계산한다.|
||update_figure||visualization|

# Dependencies

```bash
pyenv install 3.10.6
```

[CasADi](https://web.casadi.org/)

```bash
pip install casadi
```

https://github.com/acados/acados/tree/master

https://github.com/casadi/casadi/blob/main/docs/examples/python/rocket.py