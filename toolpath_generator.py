import numpy as np
from shapely.geometry import Polygon, LineString
import matplotlib.pyplot as plt

# 1. Assume you have a 2D boundary from your previous trimesh slicing
# For example, this is a simple square, but your truck slices will have many points
boundary_points = [(0, 0), (100, 0), (100, 50), (0, 50)] 
surface_polygon = Polygon(boundary_points)

# 2. Define your CSAM Hatch Parameters
# The distance between parallel lines (depends on your nozzle's spray spot size)
hatch_distance = 2.0  
# The angle of the lines (0 = horizontal). Change this per layer!
raster_angle_deg = 0  

# 3. Get the bounds of your 2D surface (min X, min Y, max X, max Y)
min_x, min_y, max_x, max_y = surface_polygon.bounds

# 4. Generate the parallel infinite lines
y_coords = np.arange(min_y, max_y, hatch_distance)
raster_lines = []

for y in y_coords:
    # Create a line that stretches entirely across the bounding box
    line = LineString([(min_x - 10, y), (max_x + 10, y)])
    raster_lines.append(line)

# 5. Intersect the lines with your actual part boundary
toolpath_segments = []

for line in raster_lines:
    # This mathematical intersection cuts the infinite line so it only exists INSIDE your part
    intersection = surface_polygon.intersection(line)
    
    # Check if the line actually hit the polygon
    if not intersection.is_empty:
        # If it's a single line segment
        if intersection.geom_type == 'LineString':
            toolpath_segments.append(list(intersection.coords))
        # If the part has a hole (like a window in the truck), it splits into multiple segments
        elif intersection.geom_type == 'MultiLineString':
            for geom in intersection.geoms:
                toolpath_segments.append(list(geom.coords))

# 6. Organize into a Zig-Zag sequence
ordered_waypoints = []
for i, segment in enumerate(toolpath_segments):
    if i % 2 == 0:
        # Even lines: Go Left to Right
        ordered_waypoints.extend(segment)
    else:
        # Odd lines: Reverse the segment to go Right to Left
        ordered_waypoints.extend(segment[::-1])

print(f"Generated {len(ordered_waypoints)} KUKA waypoints for this layer.")


# --- 2. VISUALIZATION ---

print(f"Plotting {len(ordered_waypoints)} waypoints...")

# Create the plot figure
fig, ax = plt.subplots(figsize=(10, 6))

# A. Plot the Outer Boundary
x_ext, y_ext = surface_polygon.exterior.xy
ax.plot(x_ext, y_ext, color='black', linewidth=3, label='Part Boundary (STL Slice)')

# B. Extract X and Y coordinates from the toolpath
x_path = [pt[0] for pt in ordered_waypoints]
y_path = [pt[1] for pt in ordered_waypoints]

# C. Plot the actual toolpath lines and individual waypoints
ax.plot(x_path, y_path, color='blue', linewidth=1.5, label='KUKA Toolpath')
ax.scatter(x_path, y_path, color='darkorange', s=10, zorder=3, label='Waypoints')

# D. Mark the Start and End points explicitly
ax.plot(x_path[0], y_path[0], marker='o', color='green', markersize=10, zorder=4, label='Start Point')
ax.plot(x_path[-1], y_path[-1], marker='X', color='red', markersize=10, zorder=4, label='End Point')

# Formatting the plot for an engineering view
ax.set_aspect('equal') # Ensures 1mm in X looks exactly like 1mm in Y
ax.set_title("CSAM 2D Raster Toolpath Visualization", fontsize=14, fontweight='bold')
ax.set_xlabel("X Coordinate (mm)")
ax.set_ylabel("Y Coordinate (mm)")
ax.legend(loc='upper right')
ax.grid(True, linestyle='--', alpha=0.7)

# Display the window
plt.show()