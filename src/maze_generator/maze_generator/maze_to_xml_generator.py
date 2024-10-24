import math
import matplotlib.pyplot as plt
import random
def generate_wall_model(x, y, yaw):
    return f"""
    <model name="wall_{x}_{y}">
        <link name="wall_link_{x}_{y}">
            <collision name="wall_collision_{x}_{y}">
                <geometry>
                    <box>
                        <size>1 0.1 0.5</size>
                    </box>
                </geometry>
            </collision>

            <visual name="wall_visual_{x}_{y}">
                <geometry>
                    <box>
                        <size>1 0.1 0.5</size>
                    </box>
                </geometry>
                <material>
                    <script>
                        <uri>file://media/materials/scripts/gazebo.material</uri>
                        <name>Gazebo/White</name>
                    </script>
                </material>
            </visual>
            <pose>{x} {y} 0.25 0 0 {yaw}</pose>
        </link>
        <static>true</static>
    </model>
    """

initial_text="""<sdf version='1.7'>
    <world name='default'>
    <light name='sun' type='directional'>
        <cast_shadows>1</cast_shadows>
        <pose>0 0 10 0 -0 0</pose>
        <diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0.2 0.2 0.2 1</specular>
        <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
        </attenuation>
        <direction>-0.5 0.1 -0.9</direction>
        <spot>
            <inner_angle>0</inner_angle>
            <outer_angle>0</outer_angle>
            <falloff>0</falloff>
        </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <audio>
      <device>default</device>
    </audio>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>"""

middle_text="""<state world_name='default'>
        <sim_time>2846 887000000</sim_time>
        <real_time>2852 902045418</real_time>
        <wall_time>1728055208 115912590</wall_time>
        <iterations>2846887</iterations>
        <model name='ground_plane'>
            <pose>0 0 0 0 -0 0</pose>
            <scale>1 1 1</scale>
            <link name='link'>
            <pose>0 0 0 0 -0 0</pose>
            <velocity>0 0 0 0 -0 0</velocity>
            <acceleration>0 0 0 0 -0 0</acceleration>
            <wrench>0 0 0 0 -0 0</wrench>
            </link>
        </model>
"""

last_text="""        <light name='sun'>
            <pose>0 0 10 0 -0 0</pose>
        </light>
    </state>
    
    <gui fullscreen='0'>
        <camera name='user_camera'>
            <pose>0.00672 -0.006794 21.6311 2.3e-05 1.5698 1.58037</pose>
            <view_controller>orbit</view_controller>
            <projection_type>perspective</projection_type>
        </camera>
    </gui>

    </world>
</sdf>"""


def save_models_to_file(filename, model_params):
    # Open the file in write mode
    with open(filename, 'w') as file:
        # Write the root XML element
        file.write(initial_text+"\n")
        
        # Loop through the list of tuples (x, y, yaw)
        for (x, y, yaw) in model_params:
            # Generate XML for each model and write it to the file
            file.write(generate_wall_model(x, y, yaw))
        file.write(middle_text+"\n")
        

        # Write the closing root element
        file.write(last_text)

# Create model parameters
model_params = []



# Maze generation using depth-first search with recursive backtracking
def generate_maze_dfs(width, height):
    # Initialize the maze grid (False means unvisited)
    maze = [[False for _ in range(width)] for _ in range(height)]
    
    # The maze will be built using line segments (walls) between cells.
    # Walls are stored in two lists: horizontal and vertical.
    horizontal_walls = [[True for _ in range(width)] for _ in range(height + 1)]  # Horizontal walls
    vertical_walls = [[True for _ in range(width + 1)] for _ in range(height)]  # Vertical walls

    # Directions for movement: down, up, right, left (relative to cells)
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    # Depth-First Search (recursive backtracking)
    def dfs(x, y):
        maze[y][x] = True  # Mark the current cell as visited
        
        # Randomly shuffle the directions to ensure random maze generation
        random.shuffle(directions)

        for dx, dy in directions:
            nx, ny = x + dx, y + dy

            # Check if the next cell is within bounds and unvisited
            if 0 <= nx < width and 0 <= ny < height and not maze[ny][nx]:
                # Remove the wall between the current cell and the next cell
                if dx == 1:  # Moving right
                    vertical_walls[y][x + 1] = False
                elif dx == -1:  # Moving left
                    vertical_walls[y][x] = False
                elif dy == 1:  # Moving down
                    horizontal_walls[y + 1][x] = False
                elif dy == -1:  # Moving up
                    horizontal_walls[y][x] = False

                # Recursively visit the next cell
                dfs(nx, ny)

    # Start DFS from the top-left corner (1,1)
    dfs(0, 0)

    # Collect the final list of line segments
    line_segments = []

    # Process horizontal walls
    for y in range(height + 1):
        for x in range(width):
            if horizontal_walls[y][x]:
                # Horizontal line segment: center point is at (x + 0.5, y)
                line_segments.append((x + 0.5, y, 0))

    # Process vertical walls
    for y in range(height):
        for x in range(width + 1):
            if vertical_walls[y][x]:
                # Vertical line segment: center point is at (x, y + 0.5)
                line_segments.append((x, y + 0.5, math.pi / 2))

    return line_segments

# Example: create a maze of desired dimensions (width and height)
width = 10  # Number of cells horizontally
height = 10  # Number of cells vertically

line_segments = generate_maze_dfs(width, height)

# Print the list of line segments (x, y, theta)
for segment in line_segments:
    # Convert the tuple to a list so you can modify its values
    segment_list = list(segment)

    # Change the coordinate system
    segment_list[0] = segment_list[0] - 5  # Modify x-coordinate
    segment_list[1] = segment_list[1] - 5  # Modify y-coordinate

    # Convert the list back to a tuple and append it to model_params
    model_params.append(tuple(segment_list))

# Save the models to an XML file
save_models_to_file('/home/chandansinghchauhan/programming/robotics/simulator_ws/src/micro_mouse_bringup/worlds/maze.world.xml', model_params)

# Plotting x, y in Cartesian coordinates
# x_0 = [x for (x, y, yaw) in model_params if yaw == 0]
# y_0 = [y for (x, y, yaw) in model_params if yaw == 0]
# x_1_57 = [x for (x, y, yaw) in model_params if yaw == 1.5707963267948966]
# y_1_57 = [y for (x, y, yaw) in model_params if yaw == 1.5707963267948966]

# plt.scatter(x_0, y_0, color='red', label='Yaw = 0')
# plt.scatter(x_1_57, y_1_57, color='blue', label='Yaw = 1.5707963267948966')

# Labeling the plot
# plt.xlabel('X coordinate')
# plt.ylabel('Y coordinate')
# plt.title('Wall Model Coordinates with Different Yaw Values')
# plt.legend()

# Show the plot
# plt.grid(True)
# plt.show()

print("Plot created and displayed.")
