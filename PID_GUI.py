import tkinter as tk
import threading
import time
import serial
import optuna

# -----------------------
# Set up serial connection
# -----------------------
try:
    ser = serial.Serial('COM5', 115200, timeout=1)
    print("Serial connection opened")
except Exception as e:
    print("Error opening serial port:", e)
    exit(1)

time.sleep(2)  # Wait for the serial connection to initialize

# Global state variables
pid_on = False
optuna_running = False
current_pid = {
    "Kp_inhib": 10.0,
    "Ki_inhib": 10.0,
    "Kd_inhib": 60.0,
    "Kp_excite": 1.0,
    "Ki_excite": 1.0,
    "Kd_excite": 20.0
}

# Helper function to send commands to Arduino
def send_command(cmd):
    try:
        ser.write(cmd.encode())
        ser.flush()
        print("Sent command:", cmd.strip())
    except Exception as e:
        print("Error sending command:", e)

# Toggle PID on/off and update the visual indicator
def toggle_pid():
    global pid_on
    pid_on = not pid_on
    if pid_on:
        send_command("8\n")  # "8" turns PID on
        print("Command sent: PID ON")
        start_reset_timer()  # Automatically start the 60s reset window
        pid_status_label.config(text="PID is ON", bg="green", fg="white")
    else:
        send_command("9\n")  # "9" turns PID off
        print("Command sent: PID OFF")
        pid_status_label.config(text="PID is OFF", bg="red", fg="white")

# Update PID parameters based on input boxes
def set_parameters():
    try:
        kp_inhib = float(entry_kp_inhib.get())
        ki_inhib = float(entry_ki_inhib.get())
        kd_inhib = float(entry_kd_inhib.get())
        kp_excite = float(entry_kp_excite.get())
        ki_excite = float(entry_ki_excite.get())
        kd_excite = float(entry_kd_excite.get())
        
        # Update global dictionary
        current_pid.update({
            "Kp_inhib": kp_inhib,
            "Ki_inhib": ki_inhib,
            "Kd_inhib": kd_inhib,
            "Kp_excite": kp_excite,
            "Ki_excite": ki_excite,
            "Kd_excite": kd_excite
        })
        
        # Build command string (same format as original code)
        cmd = "T" + f"{kp_inhib},{ki_inhib},{kd_inhib},{kp_excite},{ki_excite},{kd_excite}\n"
        send_command(cmd)
        update_current_info()
    except Exception as e:
        print("Error reading parameters:", e)

# Start a 60-second reset window: sends a command that sets PID outputs to zero and updates a progress counter.
def start_reset_timer():
    def reset_sequence():
        send_command("R\n")  # Command the Arduino to force PID outputs to zero for 60s
        print("Baseline reset mode initiated for 60 seconds.")
        for i in range(60, 0, -1):
            # Use root.after to update the progress label from the thread
            root.after(0, progress_label.config, {"text": f"Reset time remaining: {i} s"})
            time.sleep(1)
        root.after(0, progress_label.config, {"text": "Reset window finished"})
    threading.Thread(target=reset_sequence, daemon=True).start()

# Toggle online optuna optimization
def toggle_optimization():
    global optuna_running
    if not optuna_running:
        optuna_running = True
        threading.Thread(target=run_optimization, daemon=True).start()
    else:
        print("Stopping optimization is not implemented in this demo.")

# Update the optimization progress text box (thread-safe)
def update_optimization_progress(trial_number, loss):
    def update():
        opt_text = f"Trial {trial_number} - Loss: {loss:.4f}\n"
        opt_text_box.insert(tk.END, opt_text)
        opt_text_box.see(tk.END)
    root.after(0, update)

# Run online optimization using optuna in a separate thread
def run_optimization():
    study = optuna.create_study(direction='minimize')
    
    def objective(trial):
        # Suggest PID parameters
        kp_inhib = trial.suggest_float("Kp_inhib", 1, 20, log=True)
        ki_inhib = trial.suggest_float("Ki_inhib", 1, 20, log=True)
        kd_inhib = trial.suggest_float("Kd_inhib", 20, 200, log=True)
        kp_excite = trial.suggest_float("Kp_excite", 1, 20, log=True)
        ki_excite = trial.suggest_float("Ki_excite", 1, 20, log=True)
        kd_excite = trial.suggest_float("Kd_excite", 20, 200, log=True)
        
        # Send parameters to Arduino
        cmd = "T" + f"{kp_inhib},{ki_inhib},{kd_inhib},{kp_excite},{ki_excite},{kd_excite}\n"
        send_command(cmd)
        time.sleep(2)
        
        # Collect error measurements for a fixed duration
        measure_duration = 15
        start_time = time.time()
        error_sum = 0.0
        count = 0
        
        ser.reset_input_buffer()
        while time.time() - start_time < measure_duration:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode().strip()
                    error_val = float(line)
                    error_sum += abs(error_val)
                    count += 1
                except:
                    continue
        
        if count == 0:
            return float('inf')
        avg_error = error_sum / count
        
        update_optimization_progress(trial.number, avg_error)
        return avg_error

    study.optimize(objective, n_trials=10)
    best_params = study.best_params
    print("Optimization completed!")
    print("Best parameters:", best_params)
    
    def update_gui_best():
        entry_kp_inhib.delete(0, tk.END)
        entry_kp_inhib.insert(0, str(best_params["Kp_inhib"]))
        entry_ki_inhib.delete(0, tk.END)
        entry_ki_inhib.insert(0, str(best_params["Ki_inhib"]))
        entry_kd_inhib.delete(0, tk.END)
        entry_kd_inhib.insert(0, str(best_params["Kd_inhib"]))
        entry_kp_excite.delete(0, tk.END)
        entry_kp_excite.insert(0, str(best_params["Kp_excite"]))
        entry_ki_excite.delete(0, tk.END)
        entry_ki_excite.insert(0, str(best_params["Ki_excite"]))
        entry_kd_excite.delete(0, tk.END)
        entry_kd_excite.insert(0, str(best_params["Kd_excite"]))
        update_current_info()
    root.after(0, update_gui_best)

# Update the display of current PID parameters in the GUI
def update_current_info():
    info_text = (f"PID Parameters:\n"
                 f"  Inhibit PID: Kp: {current_pid.get('Kp_inhib')}, Ki: {current_pid.get('Ki_inhib')}, Kd: {current_pid.get('Kd_inhib')}\n"
                 f"  Excite  PID: Kp: {current_pid.get('Kp_excite')}, Ki: {current_pid.get('Ki_excite')}, Kd: {current_pid.get('Kd_excite')}")
    info_label.config(text=info_text)

# -----------------------
# Build the GUI using tkinter
# -----------------------
root = tk.Tk()
root.title("PID Controller GUI")

# Configure grid columns so they expand equally
for col in range(3):
    root.grid_columnconfigure(col, weight=1)

# Row 0: Place the three toggle buttons in separate columns to center them
pid_button = tk.Button(root, text="Toggle PID On/Off", command=toggle_pid)
pid_button.grid(row=0, column=0, padx=5, pady=5)
reset_button = tk.Button(root, text="60s Reset Window", command=start_reset_timer)
reset_button.grid(row=0, column=1, padx=5, pady=5)
opt_button = tk.Button(root, text="Toggle Online Optimization", command=toggle_optimization)
opt_button.grid(row=0, column=2, padx=5, pady=5)

# Row 1: PID visual indicator label, spanning all columns
pid_status_label = tk.Label(root, text="PID is OFF", bg="red", fg="white", font=("Helvetica", 12, "bold"))
pid_status_label.grid(row=1, column=0, columnspan=3, padx=5, pady=5)

# Row 2: Progress indicator for reset window (counter), spanning all columns
progress_label = tk.Label(root, text="", font=("Helvetica", 10))
progress_label.grid(row=2, column=0, columnspan=3, padx=5, pady=5)

# Rows 3-5: Input fields for PID_inhibit
tk.Label(root, text="Kp_inhib:").grid(row=3, column=0, padx=5, pady=5)
entry_kp_inhib = tk.Entry(root)
entry_kp_inhib.grid(row=3, column=1, padx=5, pady=5)
tk.Label(root, text="Ki_inhib:").grid(row=4, column=0, padx=5, pady=5)
entry_ki_inhib = tk.Entry(root)
entry_ki_inhib.grid(row=4, column=1, padx=5, pady=5)
tk.Label(root, text="Kd_inhib:").grid(row=5, column=0, padx=5, pady=5)
entry_kd_inhib = tk.Entry(root)
entry_kd_inhib.grid(row=5, column=1, padx=5, pady=5)

# Rows 3-5: Input fields for PID_excite
tk.Label(root, text="Kp_excite:").grid(row=3, column=2, padx=5, pady=5)
entry_kp_excite = tk.Entry(root)
entry_kp_excite.grid(row=3, column=3, padx=5, pady=5)
tk.Label(root, text="Ki_excite:").grid(row=4, column=2, padx=5, pady=5)
entry_ki_excite = tk.Entry(root)
entry_ki_excite.grid(row=4, column=3, padx=5, pady=5)
tk.Label(root, text="Kd_excite:").grid(row=5, column=2, padx=5, pady=5)
entry_kd_excite = tk.Entry(root)
entry_kd_excite.grid(row=5, column=3, padx=5, pady=5)

# Row 6: Button to update PID parameters from the entries, spanning two columns
set_param_button = tk.Button(root, text="Set PID Parameters", command=set_parameters)
set_param_button.grid(row=6, column=0, columnspan=2, padx=5, pady=5)

# Row 7: A label to display current PID parameter values, spanning all columns
info_label = tk.Label(root, text="PID Parameters: Not Set", justify=tk.LEFT)
info_label.grid(row=7, column=0, columnspan=4, padx=5, pady=5)

# Row 8: Text box for displaying optimization progress, spanning all columns
opt_text_box = tk.Text(root, height=10, width=50)
opt_text_box.grid(row=8, column=0, columnspan=4, padx=5, pady=5)

# Initialize the entry fields with default values
entry_kp_inhib.insert(0, "10.0")
entry_ki_inhib.insert(0, "10.0")
entry_kd_inhib.insert(0, "60.0")
entry_kp_excite.insert(0, "1.0")
entry_ki_excite.insert(0, "1.0")
entry_kd_excite.insert(0, "20.0")
update_current_info()

# Optionally, start with PID turned on and start the reset window automatically.
pid_on = True
send_command("8\n")
pid_status_label.config(text="PID is ON", bg="green", fg="white")
start_reset_timer()

# Run the tkinter main loop
root.mainloop()

# Close the serial connection when the GUI is closed.
ser.close()
