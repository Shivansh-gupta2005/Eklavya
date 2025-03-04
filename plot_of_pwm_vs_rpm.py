import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from numpy.polynomial.polynomial import Polynomial

# Load CSV data
CSV_FILE = "motor_data.csv"
data = pd.read_csv(CSV_FILE)

# Extract PWM and RPM values
pwm_values = data["PWM"].values
rpm_values = data["RPM"].values

# Fit a curve (Choose polynomial degree: 1 = linear, 2 = quadratic, 3 = cubic)
degree = 2  # Adjust this based on how well the data fits
coeffs = np.polyfit(pwm_values, rpm_values, degree)
rpm_function = np.poly1d(coeffs)  # Creates function RPM(PWM)

# Generate smooth PWM values for plotting curve
pwm_smooth = np.linspace(min(pwm_values), max(pwm_values), 100)
rpm_smooth = rpm_function(pwm_smooth)

# Plot data points and fitted curve
plt.scatter(pwm_values, rpm_values, color="red", label="Measured Data")
plt.plot(pwm_smooth, rpm_smooth, color="blue", label=f"Fitted Curve (Degree {degree})")

# Labels and legend
plt.xlabel("PWM")
plt.ylabel("RPM")
plt.title("PWM vs RPM Curve Fitting")
plt.legend()
plt.grid()

# Show equation on the plot
equation = "RPM = " + " + ".join([f"{c:.2f}*PWM^{i}" for i, c in enumerate(coeffs[::-1])])
plt.annotate(equation, (min(pwm_values), max(rpm_values)), fontsize=10, color="blue")

plt.show()

# Print the function equation
print(f"Fitted Equation: {rpm_function}")
