import pandas as pd
import matplotlib.pyplot as plt

# Load trajectories
df_quintic = pd.read_csv("trajectory.csv")
df_bezier  = pd.read_csv("trajectory_bezier.csv")

# Plot
plt.figure(figsize=(8,4))
plt.plot(df_quintic["x"], df_quintic["y"], label="Quintic Trajectory", linewidth=2)

# Mark start and end
plt.scatter([df_quintic["x"].iloc[0], df_quintic["x"].iloc[-1]],
            [df_quintic["y"].iloc[0], df_quintic["y"].iloc[-1]],
            color="red", marker="x", s=100, label="Constraints")

plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Trajectory Comparison")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()
