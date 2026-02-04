import pandas as pd
import allantools
import matplotlib.pyplot as plt
import numpy as np

# Define the sampling frequency (adjust if necessary)
fs = 40.0 

# Load data from CSV
df = pd.read_csv('allan_log.csv')

accel_cols = ['icm_ax', 'icm_ay', 'icm_az', 'asm_ax', 'asm_ay', 'asm_az']
gyro_cols = ['icm_gx', 'icm_gy', 'icm_gz', 'asm_gx', 'asm_gy', 'asm_gz']

# Store results for printing
allan_dev_values = {}

# --- Calculation and Plotting ---
plt.figure(figsize=(12, 8))

# Iterate over all relevant columns
for col in accel_cols + gyro_cols:
    data = df[col].values
    
    # Compute Overlapping Allan Deviation (OADEV)
    (t2, ad, ade, adn) = allantools.oadev(data, rate=fs, data_type="freq", taus='all')
    
    allan_dev_values[col] = ad[-1]
    
    plt.loglog(t2, ad, label=f'{col} Allan Deviation')

# --- Finalize Plot ---
plt.xlabel('Tau (s)')
plt.ylabel('Allan Deviation (units vary by sensor)')
plt.title('Allan Deviation Plot for All IMU Axes (ICM and ASM)')
plt.grid(True, which="both", ls="-", alpha=0.5)
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.tight_layout()
plt.show()

# --- Print Results ---
print("Calculated Allan Deviation (approximate final values):")
for col, value in allan_dev_values.items():
    print(f"{col}: {value:.6f}")
