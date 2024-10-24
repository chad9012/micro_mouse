Micro Mouse Competition Guide

Introduction
Welcome to the Micro Mouse Competition! In this guide, you will learn how to set up the ROS2 workspace for your micro mouse robot, build the necessary packages, and make the required edits to the maze_solver.py program. Please follow the instructions carefully.

Step 1: Install ROS2 (if not already installed)
If you don’t have ROS2 installed, follow these steps:

1. Install ROS2 from the official website: ROS2 Installation Guide (https://docs.ros.org/en/humble/Installation.html)
2. Make sure you have colcon installed for building ROS2 packages:

   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

Step 2: Clone the Repository
1. Open a terminal and navigate to the directory where you want to set up your workspace.
2. Clone the competition Git repository (replace <repo-link> with the actual repository link):

   ```bash
   git clone <repo-link>
   ```

3. Navigate into the workspace:

   ```bash
   cd <repo-name>  # This folder should contain a 'src' folder.
   ```

Step 3: Build the Workspace
Make sure you are in the root of the workspace directory.

1. Build the workspace using colcon:

   ```bash
   colcon build
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

Step 6: Running the Micro Mouse Program
After making your changes, build the workspace again:

   ```bash
   colcon build
   ```

Source the workspace:

   ```bash
   source install/setup.bash
   ```

Run the micro mouse evaluator and maze solver using the following commands in separate terminals:

- Evaluator Node: This node evaluates the robot's performance based on its movement through the maze:

   ```bash
   ros2 run micro_mouse_controller evaluator
   ```

- Maze Solver Node: This node controls the robot’s movement logic that you implemented:

   ```bash
   ros2 run micro_mouse_controller maze_solver
   ```

Step 7: Testing and Submitting
Test your maze-solving logic as many times as needed. Ensure that your robot successfully navigates the maze and stops when reaching the final position (Area 3).

After you are satisfied with your results, generate the submission file by running the evaluator node. It will automatically generate the results file.

Step 8: Troubleshooting
- Build issues: If you encounter build errors, make sure your ROS2 environment is correctly sourced and that there are no syntax errors in your code.
- Simulation issues: Ensure that you have set up Gazebo and the required sensor topics correctly.

Good luck with the competition!
