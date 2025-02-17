import gym
import numpy as np
import serial
import time
from gym import spaces

# Connect to Arduino
ser = serial.Serial('COM3', 115200, timeout=1)

class DopamineLaserEnv(gym.Env):
    """Custom OpenAI Gym Environment for Dopamine Control"""

    def __init__(self):
        super(DopamineLaserEnv, self).__init__()

        # Define observation space (current + past 3 dopamine levels)
        self.observation_space = spaces.Box(low=0, high=1, shape=(4,), dtype=np.float32)

        # Define action space (0: Do nothing, 1: Excitation, 2: Inhibition)
        self.action_space = spaces.Discrete(3)

        # Initial dopamine state
        self.dopamine_levels = [0.5] * 4  # Start with a stable history
        self.setpoint = 0.5  # Target dopamine level

    def step(self, action):
        """Takes an action and updates the state"""
        if action == 1:  # Excitation
            ser.write(b'EXCITE\n')  # Send excitation command to Arduino
        elif action == 2:  # Inhibition
            ser.write(b'INHIBIT\n')  # Send inhibition command to Arduino
        else:
            ser.write(b'NONE\n')  # No action

        time.sleep(0.5)  # Wait for new data

        # Read updated dopamine level from Arduino
        dopamine_level = self.get_dopamine_level()
        self.dopamine_levels.append(dopamine_level)
        self.dopamine_levels.pop(0)  # Maintain history of last 4 steps

        # Reward Calculation
        error = abs(dopamine_level - self.setpoint)
        reward = -error  # Negative reward for deviation
        if 0.48 <= dopamine_level <= 0.52:
            reward += 1  # Bonus for staying close to setpoint

        done = False  # Continue indefinitely
        return np.array(self.dopamine_levels, dtype=np.float32), reward, done, {}

    def reset(self):
        """Resets the environment"""
        self.dopamine_levels = [0.5] * 4  # Reset history
        return np.array(self.dopamine_levels, dtype=np.float32)

    def get_dopamine_level(self):
        """Reads dopamine level from Arduino"""
        ser.flushInput()
        data = ser.readline().decode().strip()
        try:
            return float(data)
        except ValueError:
            return 0.5  # Default if no data


# Train a PPO model using Stable Baselines3
from stable_baselines3 import PPO

# Create environment
env = DopamineLaserEnv()

# Train PPO model
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=50000)

# Save trained model
model.save("ppo_dopamine")

# Train a SAC model using Stable Baselines3
from stable_baselines3 import SAC

# Create environment
env = DopamineLaserEnv()

# Train SAC model
model = SAC("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=50000)

# Save trained model
model.save("sac_dopamine")


# After training, you can load the model and run it in real-time
from stable_baselines3 import PPO

# Load trained PPO model
model = PPO.load("ppo_dopamine")

# Run in real-time
env = DopamineLaserEnv()

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, reward, done, _ = env.step(action)
    print(f"Action: {action}, Reward: {reward}")
    time.sleep(0.5)  # Control cycle


