# COMP0129-CW3
Created by Yuzhou Chen, Zeyu Chen, Jeffrey Li of team 5 as part of COMP0129 Coursework 1.
## Prerequisites
To run this task, the following environment and packages are required:
- Ubuntu 20.04
- ROS Noetic
- Point Cloud Library (PCL)
- MoveIt

 ## Usage

Follow these steps to build the project and run the solution:

1. **Build the project using `catkin`:**

    Navigate to your catkin workspace root (where your `src` directory is located) and run:

    ```bash
    catkin build
    ```

    

2. **Source the setup file to set up your environment:**

    Once the build is complete, you need to source the setup file to make ROS aware of the new packages:

    ```bash
    source devel/setup.bash
    ```

    Note: You should run this command from the root of your catkin workspace. If you open a new terminal, you'll need to source this file again, or you can add it to your `.bashrc` or `.zshrc` to source it automatically.

3. **Launch the solution:**

    Finally, to run the solution, use the `roslaunch` command with your package name and launch file:

    ```bash
    roslaunch cw1_team_x run_solution.launch
    ```

    This command will start the ROS nodes and bring up any necessary components as defined in your `run_solution.launch` file.

4. **Launch the solution:**
Then open another terminal to run:
```bash
rosservice call /task 1
```

```bash
rosservice call /task 2
```

```bash
rosservice call /task 3
```
The task will working.
## Description
### Task 1
![alt text](COMP0129-CW3/blob/main/task1.png?raw=true)
The goal of task 1 is to use a robotic arm to pick up nought/cross and place it into a brown basket. The coordinates are randomly generated and are given in the /task 1 service request.
### Task 2
![alt text](COMP0129-CW3/blob/main/task2.png?raw=true)
The goal of Task 2 is to determine which reference shape the mystery shape matches. The coordinates of the shapes are randomly generated during and are given in the /task 2 service request.
### Task 3
![alt text](COMP0129-CW3/blob/main/task3?raw=true)
The goal of Task 3 is to count the total number of shapes, excluding the black obstacles and determine which shape is more common, and pick up and place the most common shape while avoiding obstacles.

## Troubleshooting

When running this package, you might encounter instances where the process dies unexpectedly or where the robotic arm stops in the middle of executing a task, especially for task 3. In such cases, it's recommended to gracefully stop the running process using `ctrl+C` and then restart the application by following these steps in the terminal:

```bash
catkin build
```
```bash
source devel/setup.bash
```
```bash
roslaunch cw1_team_x run_solution.launch
```
And in another terminal again run:
```bash
rosservice call /task 1
```

```bash
rosservice call /task 2
```

```bash
rosservice call /task 3
```

## Contribution
### Task 1(1 hour)
- Zeyu Chen (33%)
- Jeffrey Li (33%)
- Yuzhou Chen (33%)

### Task 2(20 hour)
- Zeyu Chen (33%)
- Jeffrey Li (33%)
- Yuzhou Chen (33%)

### Task 3(30 hour)
- Zeyu Chen (33%)
- Jeffrey Li (33%)
- Yuzhou Chen (33%)
