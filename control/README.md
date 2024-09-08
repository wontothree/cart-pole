# Nonlinear Model Predictive Control vs Deep Reinforcement Learning

[Nonlinear Model Predictive Control](https://wontothree.github.io/cartpole/mpc/)

- Input : initial state, current state
- Output : next control input

# Functions

|Function|Description|
|---|---|
|set_reference_state||
|set_reference_control||
|define_dynamic_model||
|define_constraints||
|define_cost_function||
|solve||
|||

|Function Name|Description|
|---|---|
|


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
