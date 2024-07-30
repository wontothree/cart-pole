import numpy as np

# Load the Q-table
try:
    q_table = np.load('./q_table.npy')
    print("Q-table loaded successfully.")
    print(f"Q-table shape: {q_table.shape}")
except Exception as e:
    print(f"Error loading Q-table: {e}")
    exit(1)

# Print Q-table content
print("Q-table contents:")
print(q_table)

# Optionally, print Q-table for specific states
# This part assumes you want to see specific state values and their corresponding actions
sample_states = [
    (0, 0, 0, 0),
    (9, 9, 9, 9),
    (5, 5, 5, 5),
    (3, 7, 2, 8)
]

print("\nQ-values for sample states:")
for state in sample_states:
    print(f"State: {state}, Q-values: {q_table[state]}")
