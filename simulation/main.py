# from gym.envs.classic_control.cartpole import CartPoleEnv

# if __name__ == "__main__":
#     # initialize CartPoleEvc
#     env = CartPoleEnv(render_mode="human")  # human mode rendering
#     states, _ = env.reset()

#     for _ in range(1000):
#         # control
#         action = env.action_space.sample()

#         # one step setting
#         states, reward, terminated, _, _ = env.step(action)

#         print(f"Current State: {states}, Action: {action}, Reward: {reward}, Done: {terminated}")

#         # rendering
#         env.render()

#         if terminated:
#             # reset
#             states, _ = env.reset()

#     # close environment
#     env.close()






# import casadi

# from mpc_ver7 import CartPoleNMPC
# from gym.envs.classic_control.cartpole import CartPoleEnv

# if __name__ == "__main__":
#     # CartPole 환경 초기화
#     env = CartPoleEnv(render_mode="human")
#     env.reset()
    
#     # NMPC 객체 생성
#     cart_pole_nmpc = CartPoleNMPC()
    
#     # 초기 상태 설정 (환경에서 얻은 상태 사용)
#     x_init = casadi.DM(env.state)  # 환경에서 초기 상태 얻기

#     # MPC 실행
#     X, U, t_eval = cart_pole_nmpc.run_mpc(x_init)
    
#     # 시뮬레이션 진행
#     for t in t_eval:
#         # 현재 상태를 기반으로 제어 입력 계산
#         u_opt, _ = cart_pole_nmpc.compute_optimal_control(cart_pole_nmpc.make_nlp(), x_init, casadi.DM.zeros(cart_pole_nmpc.total))
#         print(u_opt)
#         # 환경에서 제어 입력을 사용하여 한 단계 진행
#         state, reward, terminated, _, _ = env.step(u_opt.full().ravel()[0])
#         env.render()
        
#         if terminated:
#             state = env.reset()
        
#         # 업데이트된 상태
#         x_init = casadi.DM(state)

#     # 환경 종료
#     env.close()

# import casadi
# from mpc_ver7 import CartPoleNMPC
# from gym.envs.classic_control.cartpole import CartPoleEnv

# def reorder_state(state):
#     """
#     상태 변수 순서를 맞추기 위해 상태를 재배열합니다.
#     Gym: [position, velocity, angle, angular_velocity]
#     MPC: [position, angle, velocity, angular_velocity]
#     """
#     return [state[0], state[2], state[1], state[3]]

# if __name__ == "__main__":
#     # CartPole 환경 초기화
#     env = CartPoleEnv(render_mode="human")
#     env.reset()
    
#     # NMPC 객체 생성
#     cart_pole_nmpc = CartPoleNMPC()
    
#     # 초기 상태 설정 (환경에서 얻은 상태 사용)
#     x_init = casadi.DM(reorder_state(env.state))  # 상태 재배열

#     # MPC 실행
#     X, U, t_eval = cart_pole_nmpc.run_mpc(x_init)
    
#     # 시뮬레이션 진행
#     for t in t_eval:
#         # 현재 상태를 기반으로 제어 입력 계산
#         u_opt, _ = cart_pole_nmpc.compute_optimal_control(cart_pole_nmpc.make_nlp(), x_init, casadi.DM.zeros(cart_pole_nmpc.total))
#         print(u_opt)
        
#         # 환경에서 제어 입력을 사용하여 한 단계 진행
#         action = u_opt.full().ravel()[0]
#         state, reward, terminated, _, _ = env.step(action)
#         env.render()
        
#         if terminated:
#             state = env.reset()
        
#         # 상태 업데이트 및 재배열
#         x_init = casadi.DM(reorder_state(state))

#     # 환경 종료
#     env.close()

import casadi
import numpy as np
from mpc_ver7 import CartPoleNMPC
from gym.envs.classic_control.cartpole import CartPoleEnv

def reorder_state(state):
    """
    상태 변수 순서를 맞추기 위해 상태를 재배열합니다.
    Gym: [position, velocity, angle, angular_velocity]
    MPC: [position, angle, velocity, angular_velocity]
    """
    return [state[0], state[2], state[1], state[3]]

def clip_action(action, action_space):
    """
    제어 입력을 환경의 액션 스페이스에 맞게 클리핑합니다.
    """
    return np.clip(action, action_space.low, action_space.high)

if __name__ == "__main__":
    # CartPole 환경 초기화
    env = CartPoleEnv(render_mode="human")
    env.reset()
    
    # NMPC 객체 생성
    cart_pole_nmpc = CartPoleNMPC()
    
    # 초기 상태 설정 (환경에서 얻은 상태 사용)
    x_init = casadi.DM(reorder_state(env.state))  # 상태 재배열

    # MPC 실행
    X, U, t_eval = cart_pole_nmpc.run_mpc(x_init)
    
    # 시뮬레이션 진행
    for t in t_eval:
        # 현재 상태를 기반으로 제어 입력 계산
        u_opt, _ = cart_pole_nmpc.compute_optimal_control(cart_pole_nmpc.make_nlp(), x_init, casadi.DM.zeros(cart_pole_nmpc.total))
        u_opt = u_opt.full().ravel()
        
        # 연속적인 액션을 환경의 액션 스페이스에 맞게 클리핑 및 변환
        action = clip_action(u_opt[0], env.action_space)
        
        # 환경에서 제어 입력을 사용하여 한 단계 진행
        state, reward, terminated, _, _ = env.step(action)
        env.render()
        
        if terminated:
            state = env.reset()
        
        # 상태 업데이트 및 재배열
        x_init = casadi.DM(reorder_state(state))

    # 환경 종료
    env.close()
