import pandas as pd
import matplotlib.pyplot as plt

# Load trajectories
df_quintic = pd.read_csv("trajectory.csv")
df_bezier  = pd.read_csv("trajectory_bezier.csv")

# Obstacle parameters (match your C++ obstacle)
obs_x = 10.0
obs_y = 0.0
obs_width = 4.2
safety_margin = 0.5

# Plot
plt.figure(figsize=(10,5))

# Trajectories
plt.plot(df_quintic["x"], df_quintic["y"], label="Quintic Trajectory", linewidth=2)
plt.plot(df_bezier["x"], df_bezier["y"], label="Bezier Trajectory", linewidth=2, linestyle='--')

# Start & End points
plt.scatter([df_quintic["x"].iloc[0], df_quintic["x"].iloc[-1]],
            [df_quintic["y"].iloc[0], df_quintic["y"].iloc[-1]],
            color="red", marker="x", s=100, label="Constraints")

# Obstacle rectangle (with safety margin)
plt.gca().add_patch(plt.Rectangle(
    (obs_x - obs_width/2 - safety_margin, obs_y - 0.2),
    obs_width + 2*safety_margin, 0.4,
    color='red', alpha=0.5, label='Obstacle'
))

plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Trajectory Comparison with Obstacle")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()
