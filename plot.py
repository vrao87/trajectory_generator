import pandas as pd
import matplotlib.pyplot as plt

# Load trajectory
df = pd.read_csv("trajectory.csv")

# Plot
plt.figure(figsize=(8,4))
plt.plot(df["x"], df["y"], label="Quintic Trajectory", linewidth=2)
plt.scatter([0, 40], [0, 3], color="red", marker="x", s=100, label="Constraints")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Quintic Polynomial Trajectory")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()
