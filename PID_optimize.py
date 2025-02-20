import optuna
import serial
import time
import numpy as np
import pandas as pd
import os

# -----------------------
# Set up serial connection
# -----------------------
try:
    ser = serial.Serial('COM4', 115200, timeout=1)
    print("Serial connection opened")
except Exception as e:
    print("Error opening serial port:", e)
    exit(1)

time.sleep(2)  # Wait for the serial connection to initialize

def objective(trial):
    """
    Objective function for Optuna.
    Suggests PID parameters, sends them to the Arduino in online tuning mode,
    then reads the error measurements and returns the average squared error.
    """
    print(f"\n========= Starting trial {trial.number} =========")
    # Suggest PID parameters for inhibition (REVERSE) and excitation (DIRECT)
    kp_inhib = trial.suggest_float("Kp_inhib", 1, 50, log=True)
    ki_inhib = trial.suggest_float("Ki_inhib", 1, 20, log=True)
    kd_inhib = trial.suggest_float("Kd_inhib", 50, 200, log=True)

    # For excitation, always suggest, but if bidirectional is False, fix them to 0
    if bidirectional:
        kp_excite = trial.suggest_float("Kp_excite", 1, 50, log=True)
        ki_excite = trial.suggest_float("Ki_excite", 1, 20, log=True)
        kd_excite = trial.suggest_float("Kd_excite", 50, 200, log=True)
    else:
        # Use a degenerate search space to fix the values at 1
        kp_excite = trial.suggest_float("Kp_excite", 1, 1, log=True)
        ki_excite = trial.suggest_float("Ki_excite", 1, 1, log=True)
        kd_excite = trial.suggest_float("Kd_excite", 1, 1, log=True)
    
    # Build the command string to send to the Arduino.
    # Expected format: "T<kp_inhib>,<ki_inhib>,<kd_inhib>,<kp_excite>,<ki_excite>,<kd_excite>\n"
    cmd = "T" + f"{kp_inhib},{ki_inhib},{kd_inhib},{kp_excite},{ki_excite},{kd_excite}\n"
    # print("Sending parameters to Arduino...", cmd.strip())
    
    try:
        ser.write(cmd.encode())
        ser.flush()  # Ensure data is sent immediately
    except Exception as e:
        print("Error sending parameters:", e)
        return float('inf')
    
    # Give the Arduino time to process the new parameters and settle
    time.sleep(2)
    
    # Now, collect error measurements from the Arduino for a fixed duration
    measure_duration = 15  # seconds
    start_time = time.time()
    error_sum = 0.0
    count = 0
    
    # Clear any old data in the input buffer
    ser.reset_input_buffer()
    
    print(f"\nTrial {trial.number}: collecting error measurements...")
    while time.time() - start_time < measure_duration:
        if ser.in_waiting:
            try:
                line = ser.readline().decode().strip()
                # print("Received line:", line)
                error_val = float(line)
                error_sum += abs(error_val)
                count += 1
            except Exception as e:
                # Ignore lines that cannot be parsed
                continue

    if count == 0:
        print("No valid measurements received during trial. Returning high error.")
        return float('inf')
    
    avg_error = error_sum / count
    print(f"\nTrial {trial.number}: Avg Squared Error = {avg_error:.4f} based on {count} samples")
    return avg_error

# ---- Callback for custom formatted trial output ----
def trial_callback(study, trial):
    params = trial.params
    inhib = (f"Kp={params.get('Kp_inhib', 0):>6.2f}, "
             f"Ki={params.get('Ki_inhib', 0):>6.2f}, "
             f"Kd={params.get('Kd_inhib', 0):>6.2f}")
    excite = (f"Kp={params.get('Kp_excite', 0):>6.2f}, "
              f"Ki={params.get('Ki_excite', 0):>6.2f}, "
              f"Kd={params.get('Kd_excite', 0):>6.2f}")
    print(f"--> Trial {trial.number} finished: value = {trial.value:.4f}")
    print(f"    Inhib:  {inhib}")
    print(f"    Excite: {excite}\n")

# Suppress default Optuna info messages
optuna.logging.set_verbosity(optuna.logging.WARNING)

if __name__ == '__main__':
    # Define study name: PID_<animal_name>_<today>
    animal_name = input("Enter the animal name: ")
    study_name = f"PID_{animal_name}_{time.strftime('%Y%m%d')}"

    user_input = input("Does animal contain excitatory & inhibitory opsin? (y/n): ").strip().lower()
    if user_input in ['y', 'yes']:
        bidirectional = True
    elif user_input in ['n', 'no']:
        bidirectional = False
    else:
        print("Invalid input. Defaulting to False.")
        bidirectional = False
    
    study = optuna.create_study(direction='minimize',
                                 study_name=study_name,
                                 storage='sqlite:///pid_optimize.db',
                                 load_if_exists=True)
    
    # Print progress update for each trial
    study.optimize(objective, n_trials=100, callbacks=[trial_callback], show_progress_bar=True)
    
    print("\nOptimization completed!")
    print("Best parameters found:")
    print(study.best_params)

    # Send the best commands to arduino
    best_params = study.best_params
    command = "T{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
        best_params["Kp_inhib"],
        best_params["Ki_inhib"],
        best_params["Kd_inhib"],
        best_params["Kp_excite"],
        best_params["Ki_excite"],
        best_params["Kd_excite"]
    )
    ser.write(command.encode())
    ser.flush()
    print("Best parameters sent to Arduino.")

    # Save the study results to a CSV file
    df = study.trials_dataframe()
    results_folder = "Optuna results"
    os.makedirs(results_folder, exist_ok=True)
    csv_path = os.path.join(results_folder, f"{study_name}.csv")
    df.to_csv(csv_path, index=False)
    print(f"Study results saved to {study_name}.csv")
    
    # Close the serial connection
    ser.close()
    print("Serial connection closed.")