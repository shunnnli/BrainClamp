import optuna
import serial
import time
import numpy as np

# -----------------------
# Set up serial connection
# -----------------------
# Replace 'COM3' with your port (e.g., '/dev/ttyUSB0' on Linux)
ser = serial.Serial('COM4', 115200, timeout=1)
time.sleep(2)  # Wait a moment for the serial connection to establish

def objective(trial):
    """
    Objective function for Optuna.
    Suggests PID parameters, sends them to the Arduino in online tuning mode,
    then reads the error measurements and returns the average absolute error.
    """
    # Suggest PID parameters for inhibition (REVERSE) and excitation (DIRECT)
    kp_inhib = trial.suggest_float("Kp_inhib", 1, 20)
    ki_inhib = trial.suggest_float("Ki_inhib", 0, 1)
    kd_inhib = trial.suggest_float("Kd_inhib", 1, 200)
    kp_excite = trial.suggest_float("Kp_excite", 1, 20)
    ki_excite = trial.suggest_float("Ki_excite", 0, 1)
    kd_excite = trial.suggest_float("Kd_excite", 1, 200)
    
    # Build the command string to send to the Arduino.
    # Format expected by Arduino: "T<kp_inhib>,<ki_inhib>,<kd_inhib>,<kp_excite>,<ki_excite>,<kd_excite>\n"
    cmd = "T" + f"{kp_inhib},{ki_inhib},{kd_inhib},{kp_excite},{ki_excite},{kd_excite}\n"
    ser.write(cmd.encode())
    ser.flush()  # Ensure data is sent immediately
    
    # Give the Arduino time to process the new parameters and settle
    time.sleep(2)
    
    # Now, collect error measurements from the Arduino for a fixed duration
    measure_duration = 5  # seconds
    start_time = time.time()
    error_sum = 0.0
    count = 0
    
    # Clear any old data in the input buffer
    ser.reset_input_buffer()
    
    while time.time() - start_time < measure_duration:
        if ser.in_waiting:
            try:
                line = ser.readline().decode().strip()
                # The Arduino prints error as a float value.
                error_val = float(line)
                error_sum += abs(error_val)
                count += 1
            except Exception as e:
                # If parsing fails, ignore the line
                continue
    
    # If no valid measurements were collected, return a high penalty.
    if count == 0:
        return float('inf')
    
    avg_error = error_sum / count
    print(f"Trial {trial.number}: Avg Error = {avg_error:.4f} | Params: "
          f"Kp_inhib={kp_inhib:.2f}, Ki_inhib={ki_inhib:.2f}, Kd_inhib={kd_inhib:.2f}, "
          f"Kp_excite={kp_excite:.2f}, Ki_excite={ki_excite:.2f}, Kd_excite={kd_excite:.2f}")
    
    return avg_error

if __name__ == '__main__':
    # Create and run the Optuna study
    study = optuna.create_study(direction='minimize')
    study.optimize(objective, n_trials=100)
    
    print("Best parameters found:")
    print(study.best_params)
    
    # Close the serial connection
    ser.close()
