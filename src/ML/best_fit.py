import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import pandas as pd

# ---- Step 1: Load dataset ----
data = pd.read_csv("lab_dataset.csv")  # must have columns: L, a, b

a_values = data["a"].values
b_values = data["b"].values

# ---- Step 2: Define nonlinear function ----
def nonlinear_func(a, p1, p2, p3):
    return p1 * a**2 + p2 * a + p3

# ---- Step 3: Perform curve fitting ----
params, _ = curve_fit(nonlinear_func, a_values, b_values)
p1, p2, p3 = params
print(f"Fitted equation: b = ({p1:.4f} * a ** 2) + ({p2:.4f} * a) + {p3:.4f}")

# ---- Step 4: Generate fitted curve points ----
a_fit = np.linspace(min(a_values), max(a_values), 300)
b_fit = nonlinear_func(a_fit, *params)

# ---- Step 5: Define threshold band (e.g. ±10 in 'b' axis) ----
threshold = 6
b_upper = b_fit + threshold
b_lower = b_fit - threshold

# ---- Step 6: Plot results ----
plt.figure(figsize=(8, 6))
plt.scatter(a_values, b_values, color='blue', s=25, alpha=0.6, label='Data points')
plt.plot(a_fit, b_fit, color='red', linewidth=2, label='Fitted curve')

# Add shaded threshold area
plt.fill_between(a_fit, b_lower, b_upper, color='orange', alpha=0.2, label=f'±{threshold} range')

plt.xlabel('a')
plt.ylabel('b')
plt.title('Nonlinear Curve Fit with Threshold Boundary')
plt.legend()
plt.grid(True)
plt.show()

# ---- Optional: Compute coverage ----
# Find what % of points are within the threshold band
b_pred = nonlinear_func(a_values, *params)
within_band = np.abs(b_values - b_pred) < threshold
coverage = np.sum(within_band) / len(b_values) * 100
print(f"Coverage within ±{threshold}: {coverage:.2f}% of points")