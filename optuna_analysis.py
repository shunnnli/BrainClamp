import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

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

# ---------------------------
# 4. Visualize Loss and Parameter Changes
# ---------------------------

pid_params = ["Kp_inhib", "Ki_inhib", "Kd_inhib", "Kp_excite", "Ki_excite", "Kd_excite"]
n_params = len(pid_params)
fig, axes = plt.subplots(n_params, 1, figsize=(10, 2 * n_params), sharex=True)
for i, param in enumerate(pid_params):
    if param in merged_df.columns:
        axes[i].plot(merged_df[x_col], merged_df[param], marker="o", linestyle="-")
        # Plot the loss on the right axes
        if "value" in merged_df.columns:
            ax2 = axes[i].twinx()
            ax2.plot(merged_df[x_col], merged_df["value"], color="red", linestyle="--")
            ax2.set_ylabel("Loss (MSE)", color="red")
            ax2.tick_params(axis="y", labelcolor="red")
        axes[i].set_ylabel(param)
        axes[i].set_title(f"Evolution of {param}")
    else:
        axes[i].text(0.5, 0.5, f"No data for {param}", horizontalalignment="center", verticalalignment="center")
axes[-1].set_xlabel("Trial Number")
plt.tight_layout()
plt.show()

# Save the figure with study name in results folder
fig.savefig(f"Results/{study_name}_optuna.png")
print(f"Finished: optimization summary saved in Results folder.")