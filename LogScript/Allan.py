import pandas as pd
import allantools
import matplotlib.pyplot as plt
import numpy as np

# Define the sampling frequency (adjust if necessary)
fs = 40.0

# Load data from CSV
df = pd.read_csv('allan_log.csv')

accel_cols = ['icm_ax', 'icm_ay', 'icm_az', 'asm_ax', 'asm_ay', 'asm_az']
gyro_cols  = ['icm_gx', 'icm_gy', 'icm_gz', 'asm_gx', 'asm_gy', 'asm_gz']

# Store results for printing
bias_instability = {}
white_noise = {}
random_walk = {}

def extract_white_and_rw(tau, adev):

    tau = np.asarray(tau)
    adev = np.asarray(adev)

    # Protect against any zeros (log issues)
    mask = (tau > 0) & (adev > 0)
    tau = tau[mask]
    adev = adev[mask]

    log_tau = np.log10(tau)
    log_ad  = np.log10(adev)

    # Local slope d(log adev)/d(log tau)
    slope = np.diff(log_ad) / np.diff(log_tau)

    # Midpoints for reporting and coefficient evaluation
    tau_mid = np.sqrt(tau[:-1] * tau[1:])     
    ad_mid  = np.sqrt(adev[:-1] * adev[1:])

    # Find where slope is closest to -0.5 (white) and +0.5 (random walk)
    i_white = np.argmin(np.abs(slope + 0.5))
    i_rw    = np.argmin(np.abs(slope - 0.5))

    tau_w = tau_mid[i_white]
    ad_w  = ad_mid[i_white]
    tau_r = tau_mid[i_rw]
    ad_r  = ad_mid[i_rw]

    white_coeff = ad_w * np.sqrt(tau_w)          
    rw_coeff    = ad_r * np.sqrt(3.0 / tau_r)     

    return {
        "tau_white": tau_w,
        "white_coeff": white_coeff,
        "slope_white": slope[i_white],
        "tau_rw": tau_r,
        "rw_coeff": rw_coeff,
        "slope_rw": slope[i_rw],
    }

plt.figure(figsize=(12, 8))

# Iterate over all relevant columns
for col in accel_cols + gyro_cols:
    # Limit to first 350k rows (as you had)
    data = df[col].values
    print(col + " Data initialized")

    # Compute Overlapping Allan Deviation (OADEV)
    (t2, ad, ade, adn) = allantools.oadev(data, rate=fs, data_type="freq", taus='all')
    print(col + " OADEV Calculated")

    # Bias instability (minimum Allan deviation)
    bias_instability[col] = float(np.min(ad))
    print(col + " Bias instability found")

    # White noise + random walk (slope-based automatic pick)
    metrics = extract_white_and_rw(t2, ad)
    white_noise[col] = float(metrics["white_coeff"])
    random_walk[col] = float(metrics["rw_coeff"])

    print(
        f"{col} White region: tau={metrics['tau_white']:.3g}s, "
        f"slope={metrics['slope_white']:.2f}, N={metrics['white_coeff']:.6e}"
    )
    print(
        f"{col} RW region:    tau={metrics['tau_rw']:.3g}s, "
        f"slope={metrics['slope_rw']:.2f}, K={metrics['rw_coeff']:.6e}"
    )

    # Plot
    plt.loglog(t2, ad, label=f'{col} Allan Deviation')

#Finalize plot
plt.xlabel('Tau (s)')
plt.ylabel('Allan Deviation (units vary by sensor)')
plt.title('Allan Deviation Plot for All IMU Axes (ICM and ASM)')
plt.grid(True, which="both", ls="-", alpha=0.5)
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.tight_layout()
plt.show()

#Print results
print("\nCalculated Bias Instability (min Allan deviation):")
for col, value in bias_instability.items():
    print(f"{col}: {value:.6e}")

print("\nEstimated White Noise Coefficient (N ≈ σ(τ)*sqrt(τ)):")
for col, value in white_noise.items():
    print(f"{col}: {value:.6e}")

print("\nEstimated Random Walk Coefficient (K ≈ σ(τ)*sqrt(3/τ)):")
for col, value in random_walk.items():
    print(f"{col}: {value:.6e}")
