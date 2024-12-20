Micro Mouse Competition Guide

Introduction
Welcome to the Micro Mouse Competition! In this guide, you will learn how to set up the ROS2 workspace for your micro mouse robot, build the necessary packages, and make the required edits to the maze_solver.py program. Please follow the instructions carefully.

Step 1: Install ROS2 (if not already installed)
If you don’t have ROS2 installed, follow these steps:

1. Install ROS2 from the official website: ROS2 Installation Guide (https://docs.ros.org/en/humble/Installation.html)
2. ```bash
   cd
   ```
3.  Create a new folder for ROS2 workspace
   ```bash
   mkdir micro_mouse_competition
   ```
   
5. Make sure you have colcon installed for building ROS2 packages:

   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

Step 2: Clone the Repository
1. Open a terminal and navigate to the directory to set up your workspace (inside the micro_mouse_competition directory).
```bash
cd micro_mouse_competition
```
3. Clone the competition Git repository (replace <repo-link> with the actual repository link):

   ```bash
   git clone https://github.com/chad9012/micro_mouse.git
   ```

4. Navigate into the workspace:

   ```bash
   cd micro_mouse  # This folder should contain a 'src' folder.
   ```

Step 3: Build the Workspace
Make sure you are in the root of the workspace directory.

1. Build the workspace using colcon:

   ```bash
   colcon build --symlink-install 
   ```

2. After building, source the workspace:

   ```bash
   source install/setup.bash
   ```

   Note: You will need to source the workspace every time you open a new terminal session. You can add this to your .bashrc to automate it:

   ```bash
   echo "source /path/to/your/workspace/install/setup.bash" >> ~/.bashrc
   ```

Step 4: Understanding the Workspace
The repository contains only one package located in the `src/` directory:

- `micro_mouse_controller`: This is the package that handles the robot's movement through the maze.

IMPORTANT: You are only allowed to modify the `maze_solver.py` script in this package.

Key File:
- `src/micro_mouse_controller/maze_solver.py`: This is where you will implement your maze-solving logic.

Step 5: Editing the maze_solver.py Script
Open the `maze_solver.py` script in your preferred code editor:

   ```bash
   nano src/micro_mouse_controller/maze_solver.py
   ```

Do not modify any other files in the repository. You are only allowed to add logic to this script to control the robot.

Inside `maze_solver.py`, you will find functions and callbacks related to:

- Ultrasonic sensor data: Callbacks like `front_sensor_callback`, `left_sensor_callback`, etc., handle sensor data.
- Movement control: The `control_loop` function is where you can add movement logic to navigate the robot through the maze.
- Area checking: The script already subscribes to the robot's area information, so you can stop or change the robot's behavior based on this data.

Step 6: Running the Micro Mouse simulation
After making your changes, build the workspace again:

   ```bash
   colcon build --symlink-install 
   ```

Source the workspace:

   ```bash
   source install/setup.bash
   ```
Running the simulation of mico_mouse
```bash
ros2 launch micro_mouse_bringup micro_mouse_gazebo.launch.xml
```
this will open the gazebo simulator and now you will be able to see the maze and the robot in the bottom left of the maze 

Run the micro mouse evaluator and maze solver using the following commands in separate terminals:

- Evaluator Node: This node evaluates the robot's performance based on its movement through the maze:

   ```bash
   ros2 run micro_mouse_evaluator micro_mouse_evaluator 
   ```

- Maze Solver Node: This node controls the robot’s movement logic that you implemented:

   ```bash
   ros2 run micro_mouse_controller maze_controller
   ```

Step 7: Testing and Submitting
Test your maze-solving logic as many times as needed. Ensure that your robot successfully navigates the maze and stops when reaching the final position (Area 3).

After you are satisfied with your results, generate the submission file by running the evaluator node. It will automatically generate the results file.

you can evaluate you algorithm on different mazes 
to generate new maze 
first put correct path value of maze.world.xml file in maze_to_xml_generator.py file 
```bash
   cd /path/to/src/
```
then go to maze_generator directory
```bash
  cd maze_generator/maze_generator/
```
edit the path of world_file 

now to generate the new maze just 
```bash
   python maze_to_xml_generator.py
```
excepted output "New Maze Generated"

Now if you run the simulator of micro_mouse you will see a new map


Step 8: Troubleshooting
- Build issues: If you encounter build errors, make sure your ROS2 environment is correctly sourced and that there are no syntax errors in your code.
- Simulation issues: Ensure that you have set up Gazebo and the required sensor topics correctly.

Good luck with the competition!

Resources:
1. https://docs.ros.org/en/humble/Tutorials.html
2. https://youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy&si=f-paI3-NMJy10J57
3. https://www.udemy.com/share/103Dzm3@hxCBvGwbOsHQopi0FpMp9CgVsPgup19FupXFfb1TopvkikICcdaKVxp84-3zHMcl5w==/
