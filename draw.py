from time import sleep
import krpc
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import math

# Create figure and 3D axis
fig = plt.figure(figsize=(5, 4))
ax = fig.add_subplot(111, projection='3d')

# Set axis limits with some margin
axis_limit = 1.5
ax.set_xlim([-axis_limit, axis_limit])
ax.set_ylim([-axis_limit, axis_limit])
ax.set_zlim([-axis_limit, axis_limit])

# Add axis labels with larger font
ax.set_xlabel('X Axis', fontsize=12, labelpad=15)
ax.set_ylabel('Y Axis', fontsize=12, labelpad=15)
ax.set_zlabel('Z Axis', fontsize=12, labelpad=15)

# Add a title
plt.suptitle('Vessel Direction Vector Visualization', fontsize=14, y=0.95)

# Draw coordinate axes
ax.quiver(0, 0, 0, axis_limit, 0, 0, color='gray', arrow_length_ratio=0.1, linestyle='--', alpha=0.5)
ax.quiver(0, 0, 0, 0, axis_limit, 0, color='gray', arrow_length_ratio=0.1, linestyle='--', alpha=0.5)
ax.quiver(0, 0, 0, 0, 0, axis_limit, color='gray', arrow_length_ratio=0.1, linestyle='--', alpha=0.5)

# Add labels for the axes
ax.text(axis_limit, 0, 0, 'X', color='gray', fontsize=12)
ax.text(0, axis_limit, 0, 'Y', color='gray', fontsize=12)
ax.text(0, 0, axis_limit, 'Z', color='gray', fontsize=12)

# Draw a point at the origin
ax.scatter([0], [0], [0], color='black', s=50, label='Origin')

# Initial vector (unit vector)
vector = np.array([1, 0, 0])

# Create initial arrow object with better visibility
arrow = ax.quiver(0, 0, 0, vector[0], vector[1], vector[2], 
                 color='r', arrow_length_ratio=0.15, linewidth=3, 
                 label='Vessel Direction')

# Add legend
ax.legend(loc='upper right', fontsize=10)

# Connect to KRPC
conn = krpc.connect(name='Vector Visualization')
vessel = conn.space_center.active_vessel
def update(frame):
    global vector, arrow
    
    # Get vessel direction and convert to numpy array
    d = vessel.direction(vessel.orbital_reference_frame)
    flight = vessel.flight(vessel.orbit.body.reference_frame)
    print("altitude:%", flight.surface_altitude)

    # print("heading:", heading)
    vector = np.array([d[0], d[1], d[2]])  # Reorder if needed for your coordinate system
    
    # Clear previous arrow
    arrow.remove()
    
    # Create new arrow with updated direction
    arrow = ax.quiver(0, 0, 0, vector[0], vector[1], vector[2],
                     color='r', arrow_length_ratio=0.15, linewidth=3)
    
    # Update viewing angle for better perspective
    # 
    
    # Update title with vector components
    # plt.title(f'Direction Vector: ({vector[0]:.2f}, {vector[1]:.2f}, {vector[2]:.2f})', 
            #   fontsize=12, pad=20)
    
    return arrow,

# Create animation with smoother updates
ani = FuncAnimation(fig, update, frames=200, interval=500, blit=False)

def main():
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
