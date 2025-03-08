import numpy as np
import matplotlib.pyplot as plt

# Provided data (PWM, RPM)
data = [
    (75.0, 35.1260), (80.0, 37.6471), (85.0, 40.3361), (90.0, 42.6891),
    (95.0, 45.2101), (100.0, 47.8992), (105.0, 50.4202), (110.0, 53.1092),
    (25.0, 10.4202), (30.0, 12.6050), (35.0, 15.1260), (40.0, 17.8151),
    (50.0, 22.6891), (30.0, 12.7731), (35.0, 15.1260), (40.0, 17.6471),
    (45.0, 19.8319), (50.0, 22.3529), (55.0, 25.2101), (60.0, 27.5630),
    (65.0, 30.0840), (70.0, 32.2689), (75.0, 35.2941), (80.0, 37.8151),
    (85.0, 40.1681), (90.0, 42.8571), (95.0, 45.5462), (100.0, 47.8992),
    (105.0, 50.5882), (110.0, 53.1092), (115.0, 55.7983), (120.0, 58.1513),
    (125.0, 60.6723), (130.0, 63.3613), (135.0, 65.7143), (140.0, 68.4034),
    (145.0, 70.9244), (150.0, 73.4454), (155.0, 76.1345), (160.0, 78.4874),
    (165.0, 81.1765), (170.0, 83.8655), (175.0, 86.3866), (180.0, 89.0756),
    (185.0, 91.5966), (190.0, 94.2857), (195.0, 96.8067), (200.0, 99.1597),
    (205.0, 101.8487), (210.0, 104.5378), (215.0, 106.8908), (220.0, 109.5798),
    (225.0, 112.2689), (230.0, 114.7899), (235.0, 117.3109), (240.0, 120.0000),
    (245.0, 122.5210), (250.0, 125.0420), (255.0, 128.0672), (260.0, 128.5714)
]

# Convert to numpy arrays
pwm, rpm = np.array(data).T

# Find where RPM stops increasing
increasing_indices = np.where(np.diff(rpm) > 0)[0]
last_increasing_index = increasing_indices[-1] + 1 if len(increasing_indices) > 0 else len(rpm)

# Filter data to use only increasing region
pwm_valid = pwm[:last_increasing_index]
rpm_valid = rpm[:last_increasing_index]

# Linear fit
m, c = np.polyfit(pwm_valid, rpm_valid, 1)
print(f"Linear Fit Equation: RPM = {m:.4f} * PWM + {c:.4f}")

# Generate fitted RPM values
rpm_fitted = m * pwm_valid + c

# Plot data
plt.figure(figsize=(8, 5))
plt.scatter(pwm, rpm, color='blue', label="Actual Data")
plt.scatter(pwm_valid, rpm_valid, color='green', label="Increasing Region")
plt.plot(pwm_valid, rpm_fitted, color='red', linestyle='--', label=f"Fit: RPM = {m:.2f} * PWM + {c:.2f}")
plt.xlabel("PWM")
plt.ylabel("RPM")
plt.title("RPM vs PWM with Linear Fit (Only Increasing Data)")
plt.legend()
plt.grid(True)
plt.show()
