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
# Global event used to cancel an ongoing reset timer thread
reset_timer_event = None

# -----------------------
# Helper Functions
# -----------------------
def log_message(msg):
    print(msg)
    opt_text_box.insert(tk.END, msg + "\n")
    opt_text_box.see(tk.END)

def send_command(cmd):
    try:
        ser.write(cmd.encode())
        ser.flush()
        log_message("Sent command: " + cmd.strip())
    except Exception as e:
        log_message("Error sending command: " + str(e))

def toggle_pid():
    global pid_on
    pid_on = not pid_on
    if pid_on:
        send_command("8\n")  # "8" turns PID on
        log_message("Command sent: PID ON")
        pid_status_label.config(text="PID is ON", bg="green", fg="white")
    else:
        send_command("9\n")  # "9" turns PID off
        log_message("Command sent: PID OFF")
        pid_status_label.config(text="PID is OFF", bg="red", fg="white")

def set_parameters():
    try:
        kp_inhib = float(entry_kp_inhib.get())
        ki_inhib = float(entry_ki_inhib.get())
        kd_inhib = float(entry_kd_inhib.get())
        kp_excite = float(entry_kp_excite.get())
        ki_excite = float(entry_ki_excite.get())
        kd_excite = float(entry_kd_excite.get())
        
        current_pid.update({
            "Kp_inhib": kp_inhib,
            "Ki_inhib": ki_inhib,
            "Kd_inhib": kd_inhib,
            "Kp_excite": kp_excite,
            "Ki_excite": ki_excite,
            "Kd_excite": kd_excite
        })
        
        cmd = "T" + f"{kp_inhib},{ki_inhib},{kd_inhib},{kp_excite},{ki_excite},{kd_excite}\n"
        send_command(cmd)
        update_current_info()
    except Exception as e:
        log_message("Error reading parameters: " + str(e))

def start_reset_timer():
    global reset_timer_event
    if reset_timer_event is not None:
        reset_timer_event.set()
    reset_timer_event = threading.Event()
    
    def reset_sequence(cancel_event):
        send_command("R\n")  # Force PID outputs to zero for 60s
        log_message("Baseline reset mode initiated for 60 seconds.")
        for i in range(60, 0, -1):
            if cancel_event.is_set():
                return
            root.after(0, progress_label.config, {"text": f"Reset time remaining: {i} s"})
            time.sleep(1)
        if not cancel_event.is_set():
            root.after(0, progress_label.config, {"text": "Reset window finished"})
    threading.Thread(target=reset_sequence, args=(reset_timer_event,), daemon=True).start()

# -----------------------
# Optimization Functions
# -----------------------
def trial_callback(study, trial):
    params = trial.params
    inhib = (f"Kp={params.get('Kp_inhib', 0):>6.2f}, "
             f"Ki={params.get('Ki_inhib', 0):>6.2f}, "
             f"Kd={params.get('Kd_inhib', 0):>6.2f}")
    excite = (f"Kp={params.get('Kp_excite', 0):>6.2f}, "
              f"Ki={params.get('Ki_excite', 0):>6.2f}, "
              f"Kd={params.get('Kd_excite', 0):>6.2f}")
    log_message(f"--> Trial {trial.number} finished: value = {trial.value:.4f}")
    log_message(f"    Inhib:  {inhib}")
    log_message(f"    Excite: {excite}\n")

def run_optimization_custom(trials, measure_duration, kp_inhib_range, ki_inhib_range, kd_inhib_range,
                            kp_excite_range, ki_excite_range, kd_excite_range, bidirectional, logscale):
    study = optuna.create_study(direction='minimize')
    
    def objective(trial):
        log_message(f"\n========= Starting trial {trial.number} =========")
        kp_inhib = trial.suggest_float("Kp_inhib", kp_inhib_range[0], kp_inhib_range[1], log=logscale)
        ki_inhib = trial.suggest_float("Ki_inhib", ki_inhib_range[0], ki_inhib_range[1], log=logscale)
        kd_inhib = trial.suggest_float("Kd_inhib", kd_inhib_range[0], kd_inhib_range[1], log=logscale)
        if bidirectional:
            kp_excite = trial.suggest_float("Kp_excite", kp_excite_range[0], kp_excite_range[1], log=logscale)
            ki_excite = trial.suggest_float("Ki_excite", ki_excite_range[0], ki_excite_range[1], log=logscale)
            kd_excite = trial.suggest_float("Kd_excite", kd_excite_range[0], kd_excite_range[1], log=logscale)
        else:
            kp_excite = 1.0
            ki_excite = 1.0
            kd_excite = 1.0

        log_message(f"Testing parameters: Inhib -> Kp:{kp_inhib:.2f}, Ki:{ki_inhib:.2f}, Kd:{kd_inhib:.2f}; " +
                    f"Excite -> Kp:{kp_excite:.2f}, Ki:{ki_excite:.2f}, Kd:{kd_excite:.2f}")
        cmd = "T" + f"{kp_inhib},{ki_inhib},{kd_inhib},{kp_excite},{ki_excite},{kd_excite}\n"
        send_command(cmd)
        time.sleep(2)
        
        start_time = time.time()
        error_sum = 0.0
        count = 0
        
        ser.reset_input_buffer()
        log_message(f"\nTrial {trial.number}: collecting error measurements...")
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
            log_message("No valid measurements received during trial. Returning high error.")
            return float('inf')
        avg_error = error_sum / count
        log_message(f"\nTrial {trial.number}: Avg Squared Error = {avg_error:.4f} based on {count} samples")
        return avg_error

    study.optimize(objective, n_trials=trials, callbacks=[trial_callback])
    best_params = study.best_params
    log_message("Optimization completed!")
    log_message("Best parameters: " + str(best_params))
    
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

# -----------------------
# Optimization Settings Pop-up
# -----------------------
def open_optimization_popup():
    popup = tk.Toplevel(root)
    popup.title("Optimization Settings")
    
    # Configure popup grid columns for equal weight
    for col in range(6):
        popup.grid_columnconfigure(col, weight=1)
    
    # Row 0: Number of Trials and Measurement Duration
    tk.Label(popup, text="Number of Trials:").grid(row=0, column=0, sticky="e", padx=5, pady=5)
    trials_entry = tk.Entry(popup, width=5)
    trials_entry.insert(0, "10")
    trials_entry.grid(row=0, column=1, sticky="w", padx=5, pady=5)
    
    tk.Label(popup, text="Measurement Duration (s):").grid(row=0, column=2, sticky="e", padx=5, pady=5)
    duration_entry = tk.Entry(popup, width=5)
    duration_entry.insert(0, "15")
    duration_entry.grid(row=0, column=3, sticky="w", padx=5, pady=5)
    
    # Row 1: Bidirectional and Log Scale checkboxes
    bidirectional_var = tk.BooleanVar(value=True)
    tk.Label(popup, text="Bidirectional Tuning:").grid(row=1, column=0, sticky="e", padx=5, pady=5)
    bidirectional_cb = tk.Checkbutton(popup, variable=bidirectional_var)
    bidirectional_cb.grid(row=1, column=1, sticky="w", padx=5, pady=5)
    
    logscale_var = tk.BooleanVar(value=True)
    tk.Label(popup, text="Log Scale:").grid(row=1, column=2, sticky="e", padx=5, pady=5)
    logscale_cb = tk.Checkbutton(popup, variable=logscale_var)
    logscale_cb.grid(row=1, column=3, sticky="w", padx=5, pady=5)
    
    # Row 2: Container for Inhibition and Excitation Parameter Frames
    container = tk.Frame(popup)
    container.grid(row=2, column=0, columnspan=6, sticky="nsew", padx=5, pady=5)
    container.grid_columnconfigure(0, weight=1)
    container.grid_columnconfigure(1, weight=1)
    
    inhib_frame = tk.LabelFrame(container, text="Inhibition PID Parameters", labelanchor="n")
    inhib_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
    # Kp row
    tk.Label(inhib_frame, text="Kp:").grid(row=0, column=0, padx=5, pady=5)
    kp_inhib_lower_entry = tk.Entry(inhib_frame, width=5)
    kp_inhib_lower_entry.insert(0, "1")
    kp_inhib_lower_entry.grid(row=0, column=1, padx=5, pady=5)
    tk.Label(inhib_frame, text="-").grid(row=0, column=2, padx=5, pady=5)
    kp_inhib_upper_entry = tk.Entry(inhib_frame, width=5)
    kp_inhib_upper_entry.insert(0, "20")
    kp_inhib_upper_entry.grid(row=0, column=3, padx=5, pady=5)
    # Ki row
    tk.Label(inhib_frame, text="Ki:").grid(row=1, column=0, padx=5, pady=5)
    ki_inhib_lower_entry = tk.Entry(inhib_frame, width=5)
    ki_inhib_lower_entry.insert(0, "1")
    ki_inhib_lower_entry.grid(row=1, column=1, padx=5, pady=5)
    tk.Label(inhib_frame, text="-").grid(row=1, column=2, padx=5, pady=5)
    ki_inhib_upper_entry = tk.Entry(inhib_frame, width=5)
    ki_inhib_upper_entry.insert(0, "20")
    ki_inhib_upper_entry.grid(row=1, column=3, padx=5, pady=5)
    # Kd row
    tk.Label(inhib_frame, text="Kd:").grid(row=2, column=0, padx=5, pady=5)
    kd_inhib_lower_entry = tk.Entry(inhib_frame, width=5)
    kd_inhib_lower_entry.insert(0, "20")
    kd_inhib_lower_entry.grid(row=2, column=1, padx=5, pady=5)
    tk.Label(inhib_frame, text="-").grid(row=2, column=2, padx=5, pady=5)
    kd_inhib_upper_entry = tk.Entry(inhib_frame, width=5)
    kd_inhib_upper_entry.insert(0, "200")
    kd_inhib_upper_entry.grid(row=2, column=3, padx=5, pady=5)
    
    excite_frame = tk.LabelFrame(container, text="Excitation PID Parameters", labelanchor="n")
    excite_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
    # Kp row
    tk.Label(excite_frame, text="Kp:").grid(row=0, column=0, padx=5, pady=5)
    kp_excite_lower_entry = tk.Entry(excite_frame, width=5)
    kp_excite_lower_entry.insert(0, "1")
    kp_excite_lower_entry.grid(row=0, column=1, padx=5, pady=5)
    tk.Label(excite_frame, text="-").grid(row=0, column=2, padx=5, pady=5)
    kp_excite_upper_entry = tk.Entry(excite_frame, width=5)
    kp_excite_upper_entry.insert(0, "20")
    kp_excite_upper_entry.grid(row=0, column=3, padx=5, pady=5)
    # Ki row
    tk.Label(excite_frame, text="Ki:").grid(row=1, column=0, padx=5, pady=5)
    ki_excite_lower_entry = tk.Entry(excite_frame, width=5)
    ki_excite_lower_entry.insert(0, "1")
    ki_excite_lower_entry.grid(row=1, column=1, padx=5, pady=5)
    tk.Label(excite_frame, text="-").grid(row=1, column=2, padx=5, pady=5)
    ki_excite_upper_entry = tk.Entry(excite_frame, width=5)
    ki_excite_upper_entry.insert(0, "20")
    ki_excite_upper_entry.grid(row=1, column=3, padx=5, pady=5)
    # Kd row
    tk.Label(excite_frame, text="Kd:").grid(row=2, column=0, padx=5, pady=5)
    kd_excite_lower_entry = tk.Entry(excite_frame, width=5)
    kd_excite_lower_entry.insert(0, "20")
    kd_excite_lower_entry.grid(row=2, column=1, padx=5, pady=5)
    tk.Label(excite_frame, text="-").grid(row=2, column=2, padx=5, pady=5)
    kd_excite_upper_entry = tk.Entry(excite_frame, width=5)
    kd_excite_upper_entry.insert(0, "200")
    kd_excite_upper_entry.grid(row=2, column=3, padx=5, pady=5)
    
    # Row 3: OK and Cancel buttons placed side-by-side in a button frame
    buttons_frame = tk.Frame(popup)
    buttons_frame.grid(row=3, column=0, columnspan=6, pady=10)
    ok_button = tk.Button(buttons_frame, text="OK", command=lambda: on_ok())
    ok_button.pack(side="left", expand=True, fill="x", padx=5)
    cancel_button = tk.Button(buttons_frame, text="Cancel", command=lambda: on_cancel())
    cancel_button.pack(side="left", expand=True, fill="x", padx=5)
    
    def on_ok():
        try:
            trials = int(trials_entry.get())
            duration = float(duration_entry.get())
            kp_inhib_range = (float(kp_inhib_lower_entry.get()), float(kp_inhib_upper_entry.get()))
            ki_inhib_range = (float(ki_inhib_lower_entry.get()), float(ki_inhib_upper_entry.get()))
            kd_inhib_range = (float(kd_inhib_lower_entry.get()), float(kd_inhib_upper_entry.get()))
            kp_excite_range = (float(kp_excite_lower_entry.get()), float(kp_excite_upper_entry.get()))
            ki_excite_range = (float(ki_excite_lower_entry.get()), float(ki_excite_upper_entry.get()))
            kd_excite_range = (float(kd_excite_lower_entry.get()), float(kd_excite_upper_entry.get()))
            bidirectional = bidirectional_var.get()
            logscale = logscale_var.get()
        except Exception as e:
            log_message("Error reading optimization settings: " + str(e))
            return
        popup.destroy()
        threading.Thread(target=run_optimization_custom, args=(trials, duration, 
                                                                kp_inhib_range, ki_inhib_range, kd_inhib_range, 
                                                                kp_excite_range, ki_excite_range, kd_excite_range,
                                                                bidirectional, logscale),
                         daemon=True).start()
    
    def on_cancel():
        popup.destroy()
    
    popup.grab_set()
    root.wait_window(popup)
    

def toggle_optimization():
    open_optimization_popup()

# -----------------------
# Update Display Functions
# -----------------------
def update_current_info():
    info_text = (f"                      PID Parameters\n"
                 f"  Inhibit PID: Kp: {current_pid.get('Kp_inhib')}, Ki: {current_pid.get('Ki_inhib')}, Kd: {current_pid.get('Kd_inhib')}\n"
                 f"  Excite  PID: Kp: {current_pid.get('Kp_excite')}, Ki: {current_pid.get('Ki_excite')}, Kd: {current_pid.get('Kd_excite')}")
    info_label.config(text=info_text)

# -----------------------
# Build the Main GUI
# -----------------------
root = tk.Tk()
root.title("PID Controller GUI")

for col in range(4):
    root.grid_columnconfigure(col, weight=1)

toggle_frame = tk.Frame(root)
toggle_frame.grid(row=0, column=0, columnspan=4, pady=5)
for col in range(3):
    toggle_frame.grid_columnconfigure(col, weight=1)

pid_button = tk.Button(toggle_frame, text="Toggle PID On/Off", command=toggle_pid)
pid_button.grid(row=0, column=0, padx=5, pady=5)
reset_button = tk.Button(toggle_frame, text="60s Reset Window", command=start_reset_timer)
reset_button.grid(row=0, column=1, padx=5, pady=5)
opt_button = tk.Button(toggle_frame, text="Toggle Online Optimization", command=toggle_optimization)
opt_button.grid(row=0, column=2, padx=5, pady=5)

pid_status_label = tk.Label(root, text="PID is OFF", bg="red", fg="white", font=("Helvetica", 12, "bold"))
pid_status_label.grid(row=1, column=0, columnspan=4, padx=5, pady=5)

progress_label = tk.Label(root, text="", font=("Helvetica", 10))
progress_label.grid(row=2, column=0, columnspan=4, padx=5, pady=5)

tk.Label(root, text="Kp_inhib:").grid(row=3, column=0, padx=5, pady=5)
entry_kp_inhib = tk.Entry(root)
entry_kp_inhib.grid(row=3, column=1, padx=5, pady=5)
tk.Label(root, text="Ki_inhib:").grid(row=4, column=0, padx=5, pady=5)
entry_ki_inhib = tk.Entry(root)
entry_ki_inhib.grid(row=4, column=1, padx=5, pady=5)
tk.Label(root, text="Kd_inhib:").grid(row=5, column=0, padx=5, pady=5)
entry_kd_inhib = tk.Entry(root)
entry_kd_inhib.grid(row=5, column=1, padx=5, pady=5)

tk.Label(root, text="Kp_excite:").grid(row=3, column=2, padx=5, pady=5)
entry_kp_excite = tk.Entry(root)
entry_kp_excite.grid(row=3, column=3, padx=5, pady=5)
tk.Label(root, text="Ki_excite:").grid(row=4, column=2, padx=5, pady=5)
entry_ki_excite = tk.Entry(root)
entry_ki_excite.grid(row=4, column=3, padx=5, pady=5)
tk.Label(root, text="Kd_excite:").grid(row=5, column=2, padx=5, pady=5)
entry_kd_excite = tk.Entry(root)
entry_kd_excite.grid(row=5, column=3, padx=5, pady=5)

set_param_button = tk.Button(root, text="Set PID Parameters", command=set_parameters)
set_param_button.grid(row=6, column=0, columnspan=4, padx=5, pady=5)

info_label = tk.Label(root, text="PID Parameters: Not Set", justify=tk.LEFT)
info_label.grid(row=7, column=0, columnspan=4, padx=5, pady=5)

opt_text_box = tk.Text(root, height=10, width=50)
opt_text_box.grid(row=8, column=0, columnspan=4, padx=5, pady=5)

entry_kp_inhib.insert(0, "10.0")
entry_ki_inhib.insert(0, "10.0")
entry_kd_inhib.insert(0, "60.0")
entry_kp_excite.insert(0, "1.0")
entry_ki_excite.insert(0, "1.0")
entry_kd_excite.insert(0, "20.0")
update_current_info()

# Set PID to be OFF at startup.
pid_on = False
send_command("9\n")
pid_status_label.config(text="PID is OFF", bg="red", fg="white")
start_reset_timer()

root.mainloop()
ser.close()
