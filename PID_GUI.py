import tkinter as tk
import threading
import time
import serial
import optuna

# -----------------------
# Set up serial connection
# -----------------------
start_time = time.time()
connected = False
lastFlushTime = time.time()
wait_time = 30

while time.time() - start_time <= wait_time:
    try:
        ser = serial.Serial('COM5', 115200, timeout=1)
        print("\nSerial connection opened")
        connected = True
        break
    except Exception as e:
        countdown = int(wait_time - (time.time() - start_time))
        msg = f"Trying to connect to Arduino... {countdown} seconds remaining"
        print(msg.ljust(70), end="\r")
        
if not connected:
    print("\nError: Failed to connect to Arduino after 10 seconds.")
    exit(1)

time.sleep(2)  # Wait for the serial connection to initialize

# -----------------------
# Set up global state variables
# -----------------------
pid_on = False
optuna_running = False
debugModeGUI = False
current_pid = {
    "Kp_inhib": 10.0,
    "Ki_inhib": 10.0,
    "Kd_inhib": 60.0,
    "Kp_excite": 10.0,
    "Ki_excite": 10.0,
    "Kd_excite": 40.0,
    "Max_inhib": 255,
    "Max_excite": 255
}
# Global event used to cancel an ongoing reset timer thread
reset_timer_event = None
baseline_median = None
baseline_std = None

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
        pid_button.config(text="Turn PID Off", bg="red", fg="white")
        set_parameters()
    else:
        send_command("9\n")  # "9" turns PID off
        log_message("Command sent: PID OFF")
        pid_button.config(text="Turn PID On", bg="green", fg="white")

def toggle_debug():
    global debugModeGUI
    debugModeGUI = debug_var.get()
    send_command("D1\n" if debugModeGUI else "D0\n")  # Still send command to Arduino if desired

def toggle_fix(channel):
    # This function disables (or enables) all PID text boxes for a channel if fix is checked.
    if channel == "inhib":
        if fix_inhib_var.get():
            # Disable inhibition PID tuning entries
            entry_kp_inhib.config(state="disabled")
            entry_ki_inhib.config(state="disabled")
            entry_kd_inhib.config(state="disabled")
            entry_max_inhib.config(state="normal")
        else:
            entry_kp_inhib.config(state="normal")
            entry_ki_inhib.config(state="normal")
            entry_kd_inhib.config(state="normal")
            entry_max_inhib.config(state="normal")
    elif channel == "excite":
        if fix_excite_var.get():
            entry_kp_excite.config(state="disabled")
            entry_ki_excite.config(state="disabled")
            entry_kd_excite.config(state="disabled")
            entry_max_excite.config(state="normal")
        else:
            entry_kp_excite.config(state="normal")
            entry_ki_excite.config(state="normal")
            entry_kd_excite.config(state="normal")
            entry_max_excite.config(state="normal")

# -----------------------
# Button to send PID Settings
# -----------------------
def set_parameters():
    try:
        # For Inhibition:
        if fix_inhib_var.get():
            kp_inhib = 0.0
            ki_inhib = 0.0
            kd_inhib = 0.0
            max_inhib = float(entry_max_inhib.get())
            fixFlagInhib = 1
        else:
            kp_inhib = float(entry_kp_inhib.get())
            ki_inhib = float(entry_ki_inhib.get())
            kd_inhib = float(entry_kd_inhib.get())
            max_inhib = float(entry_max_inhib.get())
            fixFlagInhib = 0

        # For Excitation:
        if fix_excite_var.get():
            kp_excite = 0.0
            ki_excite = 0.0
            kd_excite = 0.0
            max_excite = float(entry_max_excite.get())
            fixFlagExcite = 1
        else:
            kp_excite = float(entry_kp_excite.get())
            ki_excite = float(entry_ki_excite.get())
            kd_excite = float(entry_kd_excite.get())
            max_excite = float(entry_max_excite.get())
            fixFlagExcite = 0

        # Read expo-k and deadband
        expo_k_inhib = float(entry_expo_k_inhib.get())
        expo_k_excite = float(entry_expo_k_excite.get())
        
        current_pid.update({
            "Kp_inhib": kp_inhib,
            "Ki_inhib": ki_inhib,
            "Kd_inhib": kd_inhib,
            "Kp_excite": kp_excite,
            "Ki_excite": ki_excite,
            "Kd_excite": kd_excite,
            "Max_inhib": max_inhib,
            "Max_excite": max_excite,
            "Expo_k_inhib": expo_k_inhib,
            "Expo_k_excite": expo_k_excite,
        })

        # Build the T command with 12 parameters.
        cmd = "T" + f"{kp_inhib},{ki_inhib},{kd_inhib},{kp_excite},{ki_excite},{kd_excite},{max_inhib},{max_excite},{fixFlagInhib},{fixFlagExcite},{expo_k_inhib},{expo_k_excite},\n"
        send_command(cmd)
        update_current_info()
    except Exception as e:
        log_message("Error reading parameters: " + str(e))

# -----------------------
# Button to send Photometry Settings
# -----------------------
def set_photometry_settings():
    try:
        baselineSampleDuration = float(entry_baselineSample.get())
        # Map the normalization string to an integer code:
        mapping = {"RAW": 0, "ZSCORE": 1, "BASELINE": 2, "STD": 3}
        normalizationMethod = mapping[norm_var.get()]
        deadband = float(entry_deadband.get())

        # Build a command: P<baselineSampleDuration>,<normalizationMethod>,<deadband>\n
        cmd = "P" + f"{baselineSampleDuration},{normalizationMethod},{deadband},\n"
        send_command(cmd)
        log_message("Photometry settings updated.")
        photo_info_label.config(text=(
            "           Photometry Settings\n"
            f"  Low pass filter: 48 Hz\n"
            f"  Baseline sample:   {baselineSampleDuration} ms\n"
            f"  Normalization:     {norm_var.get()}\n"
            f"  Deadband:         {deadband}"
        ))
    except Exception as e:
        log_message("Error updating photometry settings: " + str(e))

# -----------------------
# 60s timer to reset baseline window of PID
# -----------------------
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
            root.after(0, progress_label.config, {"text": f"Reset window finished"})
    threading.Thread(target=reset_sequence, args=(reset_timer_event,), daemon=True).start()

# -----------------------
# Continuously read clamp status from Arduino
# -----------------------
def read_clamp_status():
    global lastFlushTime
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("BASELINE_STATS:"):
                try: # don’t fall through to CLAMP or DEBUG handlers
                    # e.g. "BASELINE_STATS:0.123456,0.007890"
                    payload = line.split(":",1)[1]
                    median_str, std_str = payload.split(",",1)
                    baseline_median = float(median_str)
                    baseline_std = float(std_str)
                    # overwrite the Reset‐finished label
                    log_message(f"Reset window finished (median: {baseline_median:.1f}, std: {baseline_std:.1f})")
                except Exception as e:
                    log_message(f"Error parsing baseline stats: {e}")
                continue

            if debugModeGUI:
                # In debug mode, every 5 seconds flush the serial buffer
                if time.time() - lastFlushTime > 5:
                    ser.reset_input_buffer()
                    lastFlushTime = time.time()
                # Proceed to read whatever is available
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:  # Only log nonempty lines
                        log_message("DEBUG: " + line)
            else:
                # In non-debug mode, we simply flush the buffer (to avoid accumulation)
                ser.reset_input_buffer()
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    # Normal operation: update the status only if the line contains clamp info.
                    if line.startswith("CLAMP:"):
                        clamp_val = clamp_val = line.split(":")[1].strip()
                        if clamp_val == "0":
                            root.after(0, pid_status_label.config, {"text": "PID status: OFF", "bg": "red", "fg": "white"})
                        else:
                            root.after(0, pid_status_label.config, {"text": "PID status: ON", "bg": "green", "fg": "white"})
        except Exception as e:
            log_message("Error reading clamp status: " + str(e))
        time.sleep(0.01)

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
                            kp_excite_range, ki_excite_range, kd_excite_range,
                            logscale, fixFlagInhib, fixFlagExcite):
    study = optuna.create_study(direction='minimize')
    
    def objective(trial):
        log_message(f"\n========= Starting trial {trial.number} =========")
        
        if fixFlagInhib and not fixFlagExcite:
            kp_inhib = 0.0
            ki_inhib = 0.0
            kd_inhib = 0.0
            kp_excite = trial.suggest_float("Kp_excite", kp_excite_range[0], kp_excite_range[1], log=logscale)
            ki_excite = trial.suggest_float("Ki_excite", ki_excite_range[0], ki_excite_range[1], log=logscale)
            kd_excite = trial.suggest_float("Kd_excite", kd_excite_range[0], kd_excite_range[1], log=logscale)
        elif fixFlagExcite and not fixFlagInhib:
            kp_excite = 0.0
            ki_excite = 0.0
            kd_excite = 0.0
            kp_inhib = trial.suggest_float("Kp_inhib", kp_inhib_range[0], kp_inhib_range[1], log=logscale)
            ki_inhib = trial.suggest_float("Ki_inhib", ki_inhib_range[0], ki_inhib_range[1], log=logscale)
            kd_inhib = trial.suggest_float("Kd_inhib", kd_inhib_range[0], kd_inhib_range[1], log=logscale)
        elif not fixFlagExcite and not fixFlagInhib:
            kp_inhib = trial.suggest_float("Kp_inhib", kp_inhib_range[0], kp_inhib_range[1], log=logscale)
            ki_inhib = trial.suggest_float("Ki_inhib", ki_inhib_range[0], ki_inhib_range[1], log=logscale)
            kd_inhib = trial.suggest_float("Kd_inhib", kd_inhib_range[0], kd_inhib_range[1], log=logscale)
            kp_excite = trial.suggest_float("Kp_excite", kp_excite_range[0], kp_excite_range[1], log=logscale)
            ki_excite = trial.suggest_float("Ki_excite", ki_excite_range[0], ki_excite_range[1], log=logscale)
            kd_excite = trial.suggest_float("Kd_excite", kd_excite_range[0], kd_excite_range[1], log=logscale)

        max_inhib = float(entry_max_inhib.get())
        max_excite = float(entry_max_excite.get())
        expo_k_inhib = float(entry_expo_k_inhib.get())
        expo_k_excite = float(entry_expo_k_excite.get())

        log_message(f"Testing parameters: Inhib -> Kp:{kp_inhib:.2f}, Ki:{ki_inhib:.2f}, Kd:{kd_inhib:.2f}, max: {max_inhib}; " +
                    f"Excite -> Kp:{kp_excite:.2f}, Ki:{ki_excite:.2f}, Kd:{kd_excite:.2f}, max: {max_excite}, " +
                    f"FixInhib:{fixFlagInhib}, FixExcite:{fixFlagExcite}, " +
                    f"Expo_k_inhib:{expo_k_inhib}, Expo_k_excite:{expo_k_excite}")
        cmd = "T" + f"{kp_inhib},{ki_inhib},{kd_inhib},{kp_excite},{ki_excite},{kd_excite},{max_inhib},{max_excite},{fixFlagInhib},{fixFlagExcite},{expo_k_inhib},{expo_k_excite},\n"
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
        avg_error = (error_sum / count)
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
# Calibration Settings Pop-up
# -----------------------
def open_calibration_popup():
    popup = tk.Toplevel(root)
    popup.title("Laser Calibration")

    # Define a helper function to adjust slider value
    def adjust_slider(slider, delta):
        slider.set(slider.get() + delta)
    
    # Row 0: Inhibition Slider and adjustment buttons
    tk.Label(popup, text="Inhibition PWM Frequency (Hz):").grid(row=0, column=0, padx=5, pady=5, sticky="e")
    inhib_slider = tk.Scale(popup, from_=0, to=255, orient=tk.HORIZONTAL)
    inhib_slider.set(70)  # Default value
    inhib_slider.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
    
    # Buttons for inhibition slider adjustments
    minus_inhib = tk.Button(popup, text="-", command=lambda: adjust_slider(inhib_slider, -1))
    minus_inhib.grid(row=0, column=2, padx=5, pady=5)
    plus_inhib = tk.Button(popup, text="+", command=lambda: adjust_slider(inhib_slider, 1))
    plus_inhib.grid(row=0, column=3, padx=5, pady=5)
    
    # Row 1: Excitation Slider and adjustment buttons
    tk.Label(popup, text="Excitation PWM Frequency (Hz):").grid(row=1, column=0, padx=5, pady=5, sticky="e")
    excite_slider = tk.Scale(popup, from_=0, to=255, orient=tk.HORIZONTAL)
    excite_slider.set(45)  # Default value
    excite_slider.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
    
    # Buttons for excitation slider adjustments
    minus_excite = tk.Button(popup, text="-", command=lambda: adjust_slider(excite_slider, -1))
    minus_excite.grid(row=1, column=2, padx=5, pady=5)
    plus_excite = tk.Button(popup, text="+", command=lambda: adjust_slider(excite_slider, 1))
    plus_excite.grid(row=1, column=3, padx=5, pady=5)
    
    def on_test():
        pwm_inhib = inhib_slider.get()
        pwm_excite = excite_slider.get()
        # Calibration command starts with 'C' followed by frequencies separated by comma
        cmd = "C" + f"{pwm_inhib},{pwm_excite}\n"
        send_command(cmd)

    def on_save():
        pwm_inhib = inhib_slider.get()
        pwm_excite = excite_slider.get()
        # Update the main window Max Power entries with the chosen values.
        entry_max_inhib.delete(0, tk.END)
        entry_max_inhib.insert(0, str(pwm_inhib))
        entry_max_excite.delete(0, tk.END)
        entry_max_excite.insert(0, str(pwm_excite))
        set_parameters()
        send_command("9\n")  # "9" turns PID off
        log_message("Command sent: PID OFF")
        pid_button.config(text="Turn PID On", bg="green", fg="white")
        popup.destroy()
    
    def on_cancel():
        cmd = "C" + f"{0},{0}\n"
        send_command(cmd)
        popup.destroy()
    
    button_frame = tk.Frame(popup)
    button_frame.grid(row=2, column=0, columnspan=3, pady=10)
    test_button = tk.Button(button_frame, text="Test", command=on_test)
    test_button.pack(side="left", expand=True, fill="x", padx=5)
    save_button = tk.Button(button_frame, text="Save", command=on_save)
    save_button.pack(side="left", expand=True, fill="x", padx=5)
    cancel_button = tk.Button(button_frame, text="Cancel", command=on_cancel)
    cancel_button.pack(side="left", expand=True, fill="x", padx=5)
    
    popup.grab_set()
    root.wait_window(popup)

# Modify the optimization toggle to simply open the popup
def toggle_optimization():
    open_optimization_popup()

# -----------------------
# Optimization Settings Pop-up
# -----------------------
def open_optimization_popup():
    popup = tk.Toplevel(root)
    popup.title("Optimization Settings")
    
    # Configure popup grid columns for equal weight
    for col in range(8):
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
    
    # Row 1: Fix PID and Log Scale checkboxes
    logscale_var = tk.BooleanVar(value=True)
    tk.Label(popup, text="Log Scale:").grid(row=1, column=0, sticky="e", padx=5, pady=5)
    logscale_cb = tk.Checkbutton(popup, variable=logscale_var)
    logscale_cb.grid(row=1, column=1, sticky="w", padx=5, pady=5)

    fix_inhib_opt_var = tk.BooleanVar(value=False)
    tk.Label(popup, text="Fix Inhib:").grid(row=1, column=2, sticky="e", padx=5, pady=5)
    fix_inhib_opt_cb = tk.Checkbutton(popup, variable=fix_inhib_opt_var)
    fix_inhib_opt_cb.grid(row=1, column=3, sticky="w", padx=5, pady=5)
    
    fix_excite_opt_var = tk.BooleanVar(value=False)
    tk.Label(popup, text="Fix Excite:").grid(row=1, column=4, sticky="e", padx=5, pady=5)
    fix_excite_opt_cb = tk.Checkbutton(popup, variable=fix_excite_opt_var)
    fix_excite_opt_cb.grid(row=1, column=5, sticky="w", padx=5, pady=5)
    
    # Row 2: Container for Inhibition and Excitation Parameter Frames
    container = tk.Frame(popup)
    container.grid(row=2, column=0, columnspan=7, sticky="nsew", padx=5, pady=5)
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
    
    # Row 3: OK and Cancel buttons side-by-side
    buttons_frame = tk.Frame(popup)
    buttons_frame.grid(row=3, column=0, columnspan=6, pady=10)
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
            logscale = logscale_var.get()
            fixFlagInhib = 1 if fix_inhib_opt_var.get() else 0
            fixFlagExcite = 1 if fix_excite_opt_var.get() else 0
        except Exception as e:
            log_message("Error reading optimization settings: " + str(e))
            return
        popup.destroy()

        # Pass the new fixed flags to run_optimization_custom:
        threading.Thread(target=run_optimization_custom, args=(trials, duration, 
                                                                kp_inhib_range, ki_inhib_range, kd_inhib_range, 
                                                                kp_excite_range, ki_excite_range, kd_excite_range,
                                                                logscale, fixFlagInhib, fixFlagExcite),
                         daemon=True).start()
    
    def on_cancel():
        popup.destroy()
    
    ok_button = tk.Button(buttons_frame, text="OK", command=on_ok)
    ok_button.pack(side="left", expand=True, fill="x", padx=5)
    cancel_button = tk.Button(buttons_frame, text="Cancel", command=on_cancel)
    cancel_button.pack(side="left", expand=True, fill="x", padx=5)
    
    popup.grab_set()
    root.wait_window(popup)

def toggle_optimization():
    open_optimization_popup()

# -----------------------
# Update Display Functions
# -----------------------
def update_current_info():
    info_text = (f"                             PID Parameters\n"
                 f"  Inhibit PID: Kp: {current_pid.get('Kp_inhib')}, Ki: {current_pid.get('Ki_inhib')}, Kd: {current_pid.get('Kd_inhib')}, Max: {current_pid.get('Max_inhib')}, expo: {entry_expo_k_inhib.get()}\n"
                 f"  Excite  PID: Kp: {current_pid.get('Kp_excite')}, Ki: {current_pid.get('Ki_excite')}, Kd: {current_pid.get('Kd_excite')}, Max: {current_pid.get('Max_excite')}, expo: {entry_expo_k_excite.get()}")
    info_label.config(text=info_text)

# -----------------------
# Build the Main GUI
# -----------------------
root = tk.Tk()
root.title("PID Controller GUI")

root.grid_rowconfigure(10, weight=1)
n_col = 6
for col in range(n_col):
    root.grid_columnconfigure(col, weight=1)

toggle_frame = tk.Frame(root)
toggle_frame.grid(row=0, column=0, columnspan=n_col, pady=5)
for col in range(n_col):
    toggle_frame.grid_columnconfigure(col, weight=1)

pid_button = tk.Button(toggle_frame, text="Turn PID On", command=toggle_pid, bg="green", fg="white")
pid_button.grid(row=0, column=0, padx=5, pady=5)
reset_button = tk.Button(toggle_frame, text="Reset Baseline Window", command=start_reset_timer)
reset_button.grid(row=0, column=1, padx=5, pady=5)
opt_button = tk.Button(toggle_frame, text="Online Optimization", command=toggle_optimization)
opt_button.grid(row=0, column=2, padx=5, pady=5)
calib_button = tk.Button(toggle_frame, text="Calibrate Laser", command=open_calibration_popup)
calib_button.grid(row=0, column=3, padx=5, pady=5)
debug_var = tk.BooleanVar(value=False)
debug_checkbox = tk.Checkbutton(toggle_frame, text="Debug mode", variable=debug_var,
                                command=toggle_debug)
debug_checkbox.grid(row=0, column=4, padx=5, pady=5)

pid_status_label = tk.Label(root, text="PID status: Detecting...", bg="gray", fg="white", font=("Helvetica", 12, "bold"))
pid_status_label.grid(row=1, column=0, columnspan=n_col, padx=5, pady=5)

progress_label = tk.Label(root, text="", font=("Helvetica", 10))
progress_label.grid(row=2, column=0, columnspan=n_col, padx=5, pady=5)

# PID inhibit params
# Fix settings for inhibition
fix_inhib_var = tk.BooleanVar(value=False)
fix_inhib_checkbox = tk.Checkbutton(root, text="Fix inhibition", variable=fix_inhib_var,
                                    command=lambda: toggle_fix("inhib"))
fix_inhib_checkbox.grid(row=3, column=1, padx=5, pady=5)
tk.Label(root, text="Kp_inhib:").grid(row=4, column=0, padx=5, pady=5)
entry_kp_inhib = tk.Entry(root)
entry_kp_inhib.grid(row=4, column=1, padx=5, pady=5)
tk.Label(root, text="Ki_inhib:").grid(row=5, column=0, padx=5, pady=5)
entry_ki_inhib = tk.Entry(root)
entry_ki_inhib.grid(row=5, column=1, padx=5, pady=5)
tk.Label(root, text="Kd_inhib:").grid(row=6, column=0, padx=5, pady=5)
entry_kd_inhib = tk.Entry(root)
entry_kd_inhib.grid(row=6, column=1, padx=5, pady=5)
tk.Label(root, text="Max power (inhib):").grid(row=7, column=0, padx=5, pady=5)
entry_max_inhib = tk.Entry(root)
entry_max_inhib.grid(row=7, column=1, padx=5, pady=5)
tk.Label(root, text="Expo k (inhib):").grid(row=8, column=0, padx=5, pady=5)
entry_expo_k_inhib = tk.Entry(root)
entry_expo_k_inhib.insert(0, "5.0")
entry_expo_k_inhib.grid(row=8, column=1, padx=5, pady=5)

# PID excite params
# Fix setting for excitation
fix_excite_var = tk.BooleanVar(value=False)
fix_excite_checkbox = tk.Checkbutton(root, text="Fix excitation", variable=fix_excite_var,
                                    command=lambda: toggle_fix("excite"))
fix_excite_checkbox.grid(row=3, column=3, padx=5, pady=5)
tk.Label(root, text="Kp_excite:").grid(row=4, column=2, padx=5, pady=5)
entry_kp_excite = tk.Entry(root)
entry_kp_excite.grid(row=4, column=3, padx=5, pady=5)
tk.Label(root, text="Ki_excite:").grid(row=5, column=2, padx=5, pady=5)
entry_ki_excite = tk.Entry(root)
entry_ki_excite.grid(row=5, column=3, padx=5, pady=5)
tk.Label(root, text="Kd_excite:").grid(row=6, column=2, padx=5, pady=5)
entry_kd_excite = tk.Entry(root)
entry_kd_excite.grid(row=6, column=3, padx=5, pady=5)
tk.Label(root, text="Max power (excite):").grid(row=7, column=2, padx=5, pady=5)
entry_max_excite = tk.Entry(root)
entry_max_excite.grid(row=7, column=3, padx=5, pady=5)
tk.Label(root, text="Expo k (excite):").grid(row=8, column=2, padx=5, pady=5)
entry_expo_k_excite = tk.Entry(root)
entry_expo_k_excite.insert(0, "2.0")
entry_expo_k_excite.grid(row=8, column=3, padx=5, pady=5)

# Photometry processing params
tk.Label(root, text="Low pass filter (Hz):").grid(row=4, column=4, padx=5, pady=5)
entry_photometryWindow = tk.Entry(root)
entry_photometryWindow.insert(0, "48")
entry_photometryWindow.config(state="disabled")
entry_photometryWindow.grid(row=4, column=5, padx=5, pady=5)
tk.Label(root, text="Baseline sample duration (ms):").grid(row=5, column=4, padx=5, pady=5)
entry_baselineSample = tk.Entry(root)
entry_baselineSample.insert(0, "3000")  # default 3000 ms
entry_baselineSample.grid(row=5, column=5, padx=5, pady=5)
tk.Label(root, text="Normalization method:").grid(row=6, column=4, padx=5, pady=5)
norm_options = ["RAW", "ZSCORE", "BASELINE", "STD"]
norm_var = tk.StringVar(value=norm_options[0])  # default "RAW"
norm_menu = tk.OptionMenu(root, norm_var, *norm_options)
norm_menu.config(width=8)
norm_menu.grid(row=6, column=5, padx=5, pady=5)
tk.Label(root, text="Deadband:").grid(row=7, column=4, padx=5, pady=5)
entry_deadband = tk.Entry(root)
entry_deadband.insert(0, "3") 
entry_deadband.grid(row=7, column=5, padx=5, pady=5)

set_param_button = tk.Button(root, text="Set PID Parameters", command=set_parameters)
set_param_button.grid(row=9, column=0, columnspan=4, padx=5, pady=5)
info_label = tk.Label(root, text="PID Parameters: Not Set", justify=tk.LEFT)
info_label.grid(row=10, column=0, columnspan=4, padx=5, pady=5)

set_photo_button = tk.Button(root, text="Set Photometry Settings", command=set_photometry_settings)
set_photo_button.grid(row=8, column=4, columnspan=n_col, padx=5, pady=5)
photo_info_label_text =(
    "           Photometry Settings\n"
    f"  Low pass filter: 48 Hz\n"
    f"  Baseline sample:   {float(entry_baselineSample.get())} ms\n"
    f"  Normalization:     {norm_var.get()}\n"
    f"  Deadband:          {float(entry_deadband.get())}"
)
photo_info_label = tk.Label(root, text=photo_info_label_text, justify=tk.LEFT)
photo_info_label.grid(row=10, column=4, columnspan=n_col, padx=5, pady=5)

opt_text_box = tk.Text(root, height=10, width=170)
opt_text_box.grid(row=11, column=0, columnspan=n_col, padx=5, pady=5, sticky='nsew')

entry_kp_inhib.insert(0, "10.0")
entry_ki_inhib.insert(0, "10.0")
entry_kd_inhib.insert(0, "60.0")
entry_kp_excite.insert(0, "10.0")
entry_ki_excite.insert(0, "10.0")
entry_kd_excite.insert(0, "40.0")
entry_max_inhib.insert(0, "255")
entry_max_excite.insert(0, "255")
update_current_info()

# Set PID to be OFF at startup.
pid_on = False
send_command("9\n")
pid_button.config(text="Turn PID On", bg="green", fg="white")

# Start background thread to read clamp status from Arduino.
threading.Thread(target=read_clamp_status, daemon=True).start()

root.mainloop()
ser.close()
