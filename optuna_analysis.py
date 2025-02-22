import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os

# Set up display style
# sns.set(style="whitegrid")

# ---------------------------
# 1. Connect to the Database
# ---------------------------
db_path = "pid_optimize.db"  # change if needed
conn = sqlite3.connect(db_path)

# Read studies table
studies_df = pd.read_sql_query("SELECT * FROM studies", conn)
print("Studies in DB:")
print(studies_df)

# Ask user to select a study by entering study_id
study_id = int(input("Enter study_id to analyze: "))
study_name = studies_df[studies_df['study_id'] == study_id]['study_name'].iloc[0]
print(f"\nSelected study: {study_name}")

results_folder = f"Results/{study_name}"
os.makedirs(results_folder, exist_ok=True)

# Ask user to select a study based on animal name
# animal_name = input("Select study animal: ")
# Give user a indexed list of studies to choose from based on the animal name
# print(f"Available studies for {animal_name}:")
# for i, study_name in enumerate(studies_df[studies_df['animal_name'] == animal_name]['study_name']):
#     print(f"{i + 1}. {study_name}")
# study_idx = int(input("Enter the index of the study you want to analyze: ")) - 1
# study_name = studies_df[studies_df['animal_name'] == animal_name]['study_name'].iloc[study_idx]
# study_id = studies_df[studies_df['study_name'] == study_name]['study_id'].iloc[0]
# print(f"\nSelected study: {study_name}")

# ---------------------------
# 2. Extract Data from Trials and Trial Params
# ---------------------------
# Read trials table for the study of interest
trials_df = pd.read_sql_query(f"SELECT * FROM trials WHERE study_id = {study_id}", conn)
print("\nTrials (first few rows):")
print(trials_df.head())

# Read trial_params table (contains one row per parameter per trial)
params_df = pd.read_sql_query("SELECT * FROM trial_params", conn)
print("\nTrial Parameters (first few rows):")
print(params_df.head())

# Read the trial values (loss) if available
try:
    values_df = pd.read_sql_query("SELECT * FROM trial_values", conn)
    print("\nTrial Values (first few rows):")
    print(values_df.head())
    # Merge the values with the trials dataframe
    trials_df = pd.merge(trials_df, values_df, on="trial_id", how="left")
except pd.io.sql.DatabaseError:
    print("\nNo trial values found in the database.")

# Close the connection once data is read
conn.close()

# ---------------------------
# 3. Process and Merge the Data
# ---------------------------

# Pivot trial_params so that each trial_id becomes a row with parameters as columns
pivot_params = params_df.pivot(index="trial_id", columns="param_name", values="param_value").reset_index()

# Merge the pivoted parameters with the trials dataframe
merged_df = pd.merge(trials_df, pivot_params, on="trial_id", how="left")

# Sort by trial number if available, otherwise by trial_id
if 'number' in merged_df.columns:
    merged_df.sort_values(by="number", inplace=True)
    x_col = "number"
else:
    merged_df.sort_values(by="trial_id", inplace=True)
    x_col = "trial_id"

merged_df.reset_index(drop=True, inplace=True)
print("\nMerged Trial Data (first few rows):")
print(merged_df.head())

# Save the merged data to a CSV file
merged_csv_path = os.path.join(results_folder, "merged_df.csv")
merged_df.to_csv(merged_csv_path, index=False)


# ---------------------------
# 4. Visualize Loss and Parameter Changes
# ---------------------------

pid_params = ["Kp_inhib", "Ki_inhib", "Kd_inhib", "Kp_excite", "Ki_excite", "Kd_excite"]
n_params = len(pid_params)

# Get the best params from the merged_df
best_params = merged_df.loc[merged_df["value"].idxmin(), pid_params]
best_idx = merged_df["number"].loc[merged_df["value"].idxmin()]

# Plot param evolution for each PID parameter
fig, axes = plt.subplots(n_params, 1, figsize=(10, 2 * n_params), sharex=True)
for i, param in enumerate(pid_params):
    if param in merged_df.columns:
        axes[i].plot(merged_df[x_col], merged_df[param], marker="o", linestyle="-")
        # Plot the loss on the right axes
        ax2 = axes[i].twinx()
        ax2.plot(merged_df[x_col], merged_df["value"], color="orange", linestyle="-")
        ax2.set_ylabel("Loss (MSE)", color="orange")
        ax2.tick_params(axis="y", labelcolor="orange")
        axes[i].set_ylabel(param)
        axes[i].set_title(f"Evolution of {param}")
        # Highlight the best value
        best_value = best_params[param]
        axes[i].plot(best_idx, best_value, marker="*", markersize=10, color="red")
    else:
        axes[i].text(0.5, 0.5, f"No data for {param}", horizontalalignment="center", verticalalignment="center")
axes[-1].set_xlabel("Trial Number")
plt.tight_layout()
plt.show(block=False)
# Save the figure with study name in results folder
fig.savefig(os.path.join(results_folder, "optuna_param_evolution.png"))

# Plot param vs loss for each PID parameter
fig, axes = plt.subplots(int(n_params/2), 2, figsize=(10, 2 * n_params))
for i, param in enumerate(pid_params):
    if param in merged_df.columns:
        sns.scatterplot(data=merged_df, x=param, y="value", ax=axes[i//2, i%2])
        axes[i//2, i%2].set_title(f"{param} vs Loss")
        axes[i//2, i%2].set_xlabel(param)
        axes[i//2, i%2].set_ylabel("Loss (MSE)")
        # Highlight the best value
        best_value = best_params[param]
        axes[i//2, i%2].plot(best_value, merged_df["value"].min(), marker="*", markersize=10, color="red")
    else:
        axes[i//2, i%2].text(0.5, 0.5, f"No data for {param}", horizontalalignment="center", verticalalignment="center")
plt.tight_layout()
plt.show(block=False)
# Save the figure with study name in results folder
fig.savefig(os.path.join(results_folder, "optuna_param_vs_loss.png"))

print(f"Finished: optimization summary saved in Results folder.")