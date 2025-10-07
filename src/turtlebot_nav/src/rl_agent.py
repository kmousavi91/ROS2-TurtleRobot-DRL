import numpy as np
import random
# TODO: Import your ML framework here (e.g., import torch.nn as nn, torch.optim as optim)

class RLAgent:
    def __init__(self, state_dim, action_dim, learning_rate=0.001, gamma=0.99, epsilon=1.0):
        # Configuration
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.gamma = gamma          # Discount factor
        self.epsilon = epsilon      # Exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.9995

        # TODO: Initialize your Q-Network (e.g., self.q_network)
        # TODO: Initialize your Replay Buffer (e.g., self.memory)

    def select_action(self, state):
        """Epsilon-greedy action selection."""
        if np.random.rand() <= self.epsilon:
            # Explore: Choose a random action
            return random.randrange(self.action_dim)
        # Exploit: Choose the best action based on Q-values
        # TODO: Implement Q-network prediction here
        # return np.argmax(self.q_network.predict(state))
        return 0 # Placeholder: Always move forward for now

    def train(self, state, action, reward, next_state, done):
        """Train the Q-network using experience."""
        # TODO: Implement the training step (sampling from buffer, calculating loss, backpropagation)
        pass # Placeholder
        
    def decay_epsilon(self):
        """Reduce the exploration rate."""
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
