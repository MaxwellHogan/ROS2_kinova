# ROS 2 Kinova Gen3 — Robotics Laboratory

This repository provides a Docker-based environment for working with the **Kinova Gen3 6-DOF robotic arm and Robotiq 2F-85 gripper** using ROS 2 Humble, MoveIt 2 and RViz.

The environment uses simulated (fake) hardware, allowing students to develop and test robot control algorithms without access to a physical robotic arm.

## 1. Prerequisites

Before starting, make sure you have installed the following software on your computer. You will have already installed these if you have been following along in the labs. 

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) — to run the ROS 2 environment.
- [Git](https://git-scm.com/downloads) — to clone the repository.
- [Visual Studio Code](https://code.visualstudio.com/) — recommended for editing Python files and laboratory exercises.

Ensure Docker Desktop is running before proceeding.

**Note:** The container uses ROS 2 Humble and includes the required Kinova packages. You do not need to install ROS 2 directly on your computer.

## 2. Clone the repository

Open a terminal (or PowerShell on Windows), navigate to a suitable directory and run:

```bash
git clone https://github.com/MaxwellHogan/ROS2_kinova.git
```

Navigate into the repository:

```bash
cd ROS2_kinova
```

This directory will serve as your local workspace containing the Docker configuration and laboratory exercises.

## 3. Configure your workspace path

**Important: Complete this step before running the Docker container startup script.**

The Docker container requires access to the files you cloned onto your computer. This is accomplished using a Docker bind mount.

Open the provided container startup script found in the "scripts" directory in VS Code and locate the workspace volume mapping. It will contain a Docker volume argument similar to:

```bash
-v "<PATH_TO_YOUR_REPOSITORY>:/ros2_ws"
```

Replace `<PATH_TO_YOUR_REPOSITORY>` with the **absolute path** to the `ROS2_kinova/workspace` directory on your computer.

For example, on Windows:

```text
-v "C:\Users\Student\Documents\ROS2_kinova\workspace:/ros2_ws"
```

Or on MacOS:

```text
-v "/home/student/ROS2_kinova/workspace:/ros2_ws"
```

The directory on the left is the folder on your computer. The directory on the right (`/ros2_ws`) is the location inside the Docker container.

Save the script after making the change.

**Why is this necessary?**

The bind mount allows you to edit exercise files in VS Code on your computer and run them inside the ROS 2 Docker environment. Changes made to files in the mounted workspace are reflected on both sides.

Be careful not to change the `/ros2_ws` destination unless specifically instructed.

## 4. Build the Docker image

From the root of the cloned repository, run:

```bash
docker build --no-cache -t ros2_kinova -f docker/Dockerfile.kinova .
```

Docker will download and install the required ROS 2 packages, dependencies and Kinova configuration.

The initial build may take several minutes depending on your computer and internet connection.

When the build completes successfully, confirm that the image exists:

```bash
docker images
```

Look for the `ros2_kinova` image.

You do not need to rebuild the image every time you change your Python exercise files.

## 5. Start the Docker container

Run the container startup script supplied with the repository, after completing the workspace-path configuration in Section 3.

The script is responsible for creating the Docker container, mounting your workspace and making the ROS 2 environment available.

Once inside the container, verify the workspace:

```bash
pwd
```

The expected directory is:

```text
/ros2_ws
```

You can also check that ROS 2 is installed:

```bash
ros2 --help
```

## 6. Launch the Kinova robotic arm

From a terminal inside the Docker container, run:

```bash
ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config \
  robot.launch.py \
  robot_ip:=0.0.0.0 \
  use_fake_hardware:=true \
  use_sim_time:=false \
  launch_rviz:=true
```

This starts:

- The Kinova Gen3 robot description.
- The ROS 2 controller manager and simulated hardware interfaces.
- The joint trajectory and Robotiq gripper controllers.
- MoveIt 2 for robot motion planning.
- RViz for visualisation.

**This setup does not require Gazebo or a physical robot.** The fake-hardware interfaces provide joint-state updates as commands are executed, but do not simulate realistic physical dynamics.

Keep this terminal running while completing the laboratory exercises.

## 7. Verify the robot is working

When RViz opens, you should see the Kinova Gen3 robotic arm with its Robotiq gripper.

### Test the arm

In RViz:

1. Open the **MotionPlanning** panel.
2. Select the `manipulator` planning group.
3. Choose a valid goal configuration.
4. Set the start state to the robot's current state.
5. Click **Plan** to calculate a trajectory.
6. Click **Execute** to move the simulated robot.

You should see the arm move to the selected configuration.

### Test the gripper

Select the `gripper` planning group and try the available open and closed configurations.

Alternatively, open another terminal inside the container and command the gripper directly:

```bash
ros2 action send_goal \
  /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 50.0}}"
```

This opens the gripper.

To close the gripper:

```bash
ros2 action send_goal \
  /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.79, max_effort: 50.0}}"
```

You should observe the gripper fingers moving in RViz.

If you subsequently plan a gripper trajectory in RViz, update the planning start state to the current position first.

## 8. Laboratory exercises

The exercises introduce robot programming and kinematics progressively.

| Exercise | Topic |
|---|---|
| 1 | Reading robot joint positions using ROS 2 |
| 2 | Sending joint trajectory commands |
| 3 | Forward kinematics and homogeneous transformations |
| 4 | Inverse kinematics using numerical methods |
| 5 | Inverse kinematics using optimisation |
| 6 | Cartesian waypoint execution |

You will use Python and ROS 2 to observe, command and model the robot.

Follow the separate student laboratory instructions provided with the exercises.

**Important:** Some exercises publish joint commands directly to ROS 2 controllers rather than through MoveIt. These commands do not automatically receive MoveIt's collision checking. Only perform these exercises with simulated hardware unless authorised to work with a physical robot under supervision.

## 9. Troubleshooting

**Docker cannot find the workspace directory**

Check that the workspace path in your startup script points to the directory containing the cloned repository.

**RViz does not open**

Check that Docker Desktop is running and that the container startup script supports graphical applications on your operating system.

**MoveIt plans a trajectory but execution fails**

Check the active controllers:

```bash
ros2 control list_controllers
```

The `joint_trajectory_controller`, `joint_state_broadcaster` and `robotiq_gripper_controller` should be active.

Also ensure the planned starting position matches the robot's current state.

**Robot joint states are not updating**

Check the joint-state topic:

```bash
ros2 topic echo /joint_states --once
```

**Python cannot import ROS 2 packages**

Ensure you are executing Python inside the Docker container and that the ROS 2 environment has been sourced.

---

## Software

This environment uses ROS 2 Humble, MoveIt 2, `ros2_control`, the Kinova Kortex ROS 2 packages and RViz.

The Docker image includes local compatibility modifications to the installed Kinova/Robotiq Xacro configuration to support fake-hardware gripper control.

The laboratory environment is intended for teaching, simulation and development rather than direct deployment to a physical robot.