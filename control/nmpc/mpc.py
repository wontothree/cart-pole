from abc import ABC, abstractmethod
import numpy as np

class MPC(ABC):
    def __init__(self, state_dim, control_dim, prediction_horizon, control_sampling_time):
        self.state_dim = state_dim
        self.control_dim = control_dim
        self.prediction_horizon = prediction_horizon
        self.control_sampling_time = control_sampling_time

        # number of optimization variables
        self.total_variables = (prediction_horizon + 1) * state_dim + prediction_horizon * control_dim

    def set_reference_state(self, reference_state):
        self.reference_state = reference_state

    def set_reference_control(self, reference_control):
        self.reference_control = reference_control

    def set_initial_state(self, initial_state):
        self.initial_state = initial_state

    def set_current_state(self, current_state):
        self.current_state = current_state

    @abstractmethod
    def define_dynamic_model(self, state, control):
        pass

    @abstractmethod
    def define_constraints(self):
        pass

    @abstractmethod
    def define_cost_function(self):
        pass

    @abstractmethod
    def solve(self):
        pass

