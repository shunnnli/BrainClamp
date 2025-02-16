import optuna
import serial
import time
import numpy as np

# Connect to Arduino (Change port as needed)
ser = serial.Serial('COM3', 115200, timeout=1)

# Desired dopamine level
setpoint = 0.5

# Function to read dopamine activity from Arduino
def get_dopamine_activity():
    ser.flushInput()
    data = ser.readline().decode().strip()
    try:
        return float(data)
    except ValueError:
        return None  # Handle empty reads

# Function to send six PID parameters to Arduino
def send_pid_to_arduino(Kp1, Ki1, Kd1, Kp2, Ki2, Kd2):
    ser.write(f"{Kp1},{Ki1},{Kd1},{Kp2},{Ki2},{Kd2}\n".encode())

# Optuna objective function to minimize error
def pid_objective(trial):
    # Suggest reasonable PID values
    Kp1 = trial.suggest_float("Kp1", 0.1, 10.0)
    Ki1 = trial.suggest_float("Ki1", 0.001, 1.0)
    Kd1 = trial.suggest_float("Kd1", 0.001, 1.0)

    Kp2 = trial.suggest_float("Kp2", 0.1, 10.0)
    Ki2 = trial.suggest_float("Ki2", 0.001, 1.0)
    Kd2 = trial.suggest_float("Kd2", 0.001, 1.0)

    # Send updated PID parameters to Arduino
    send_pid_to_arduino(Kp1, Ki1, Kd1, Kp2, Ki2, Kd2)

    time.sleep(0.5)  # Allow Arduino to process and update
    dopamine_activity = get_dopamine_activity()

    if dopamine_activity is None:
        return np.inf  # Penalize if no data is received

    # Minimize the absolute difference between measured and desired dopamine level
    return abs(dopamine_activity - setpoint)

# Run Optuna optimization
study = optuna.create_study(direction="minimize")
study.optimize(pid_objective, n_trials=100)  # Adjust number of trials as needed

# Get best PID parameters
best_params = study.best_params
print(f"Best PID Parameters: {best_params}")

# Close Serial Connection
ser.close()
