# Kinova Gen3 6-DOF — ROS 2, Gazebo and Python laboratory

**Student worksheet | ROS 2 Humble | Ubuntu 22.04 | Gazebo Fortress**

**Suggested time:** 2 hours for the MoveIt demonstration and Exercises 1–4; Exercise 5 and the waypoint extension are optional. Work **inside the course's Docker container**. You do not need a physical robot.

## What will you learn?

By the end of the laboratory you should be able to:

1. Inspect ROS 2 topics and read the six joint angles of a simulated robot arm.
2. Publish a joint trajectory from Python to move a joint by a specified angle.
3. Calculate the end-effector's **X, Y, Z position** using forward kinematics.
4. Calculate joint angles for a desired XYZ position using a **numerical Jacobian**.
5. Optionally compare Jacobian inverse kinematics with a numerical optimiser.

The main exercises are `01_read_joints.py` through `04_inverse_kinematics.py`. The optional extension files are `05_optimisation.py` and `06_xyz_waypoints.py`.

**How to complete the exercises:** Open each numbered `.py` file and search for `TODO`. Replace the unfinished `None` values / comments with your code. Test each exercise before starting the next one. The helper files `kinova_ros.py` and `kinova_model.py` are supplied **complete**; you do not need to modify them. The hints below deliberately explain the method without filling in every answer for you.

> **Simulation safety:** Direct Python trajectories do not check obstacles or self-collision. Use only the Gazebo simulation, make small movements in an empty scene and check computed moves before using `--execute`. Do not send these examples to a real robot.

---

## Part A — Start the simulation and prepare your workspace

### A1. Make sure all the files are in one folder

Your lab directory should look like this:

```text
/ros2_ws/kinova_gen3_lab/
├── README.md
├── 01_read_joints.py
├── 02_move_joints.py
├── 03_forward_kinematics.py
├── 04_inverse_kinematics.py
├── 05_optimisation.py             # optional
├── 06_xyz_waypoints.py            # optional extension
├── kinova_ros.py                  # supplied helper: ROS communication
├── kinova_model.py                # supplied helper: robot kinematics
└── LEGACY_MAPPING.md             # background for the 6-DOF adaptation
```

If your instructor has mounted the folder elsewhere, use **that location** throughout this worksheet.

**Using VS Code:** Open the **whole `kinova_gen3_lab` folder**, rather than opening an individual Python file from another folder. Use a terminal **inside the ROS 2 Docker environment** to execute the scripts; a Python interpreter on your Windows/macOS host will not have the same ROS 2 packages. If VS Code underlines `rclpy`, confirm it is using the container interpreter.

**If you moved the exercise/solution files into another directory:** Python must still be able to find the two helpers. The simplest method is to keep scripts and helpers together. Alternatively, in that terminal run:

```bash
export PYTHONPATH=/ros2_ws/kinova_gen3_lab:$PYTHONPATH
```

This environment variable applies to that terminal session only.

### A2. Terminal 1 — launch Gazebo

Open a terminal **in the running container** and leave this process running:

```bash
source /opt/ros/humble/setup.bash
ros2 launch kortex_bringup kortex_sim_control.launch.py \
  robot_type:=gen3 dof:=6 sim_ignition:=true \
  sim_gazebo:=false use_sim_time:=true launch_rviz:=false
```

You should see the Kinova robot in Gazebo. This is the **six-joint full-size Gen3**, *not* the Gen3 Lite or seven-joint model.

### A3. Terminal 2 — launch MoveIt and RViz

Leave Gazebo running. In a **second container terminal**:

```bash
source /opt/ros/humble/setup.bash
ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config sim.launch.py \
  use_sim_time:=true launch_rviz:=true
```

Your course Docker image already includes the required Kinova/Robotiq Xacro compatibility fix.

**MoveIt demonstration (10–15 minutes)**

1. In RViz, find the **MotionPlanning** panel and set the planning group to **`manipulator`**.
2. Examine the current robot pose. Try a named configuration such as **Home** or **Retract** if available.
3. Use an end-effector interactive marker to select a nearby target within the robot's workspace.
4. Press **Plan**. Observe the planned trajectory preview without moving the Gazebo arm.
5. Press **Execute** (or Plan & Execute). Observe the movement in Gazebo.
6. Discuss: how is *planning* different from *sending joint angles directly*?

The MoveIt configuration may also describe a Robotiq gripper, but **none of these Python exercises require gripper control**. If an RViz interactive marker or gripper controller is unavailable, continue with the six arm joints.

**During the Python exercises:** Do not simultaneously send movement commands from MoveIt and Python. You may leave RViz open for observation, but stop interacting with its controls while a Python trajectory is running.

### A4. Terminal 3 — verify ROS is publishing data

Open a third terminal in the **same container**:

```bash
source /opt/ros/humble/setup.bash
cd /ros2_ws/kinova_gen3_lab
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

You should see `joint_state_broadcaster` and `joint_trajectory_controller` as **active**, and a message containing the six arm joint names and positions.

**Think about this message:** The `name` and `position` arrays correspond entry by entry. They are **not guaranteed to be in joint-number order**. For example, the third position may correspond to `joint_4`, not `joint_3`.

---

## Part B — Quick Python and robotics reference

You will use these ideas in several exercises.

| Item | Meaning | Example |
|---|---|---|
| `q` | Vector of six joint angles, in radians | `q[0]` is `joint_1` |
| `p` / `xyz` | End-effector position relative to `base_link`, in metres | `[x, y, z]` |
| `J` | Position Jacobian, a **3 × 6** matrix | Each column relates one joint to XYZ change |
| `T` | Homogeneous transform, a **4 × 4** matrix | Position in `T[:3, 3]` |
| `@` | NumPy matrix multiplication | `A @ B` |
| `np.linalg.norm(v)` | Length of a vector | XYZ error distance |
| `np.eye(n)` | Identity matrix of shape n × n | `np.eye(4)` |

### Units — radians and degrees

Robot controllers usually use **radians**. To display an angle in degrees or convert a desired movement to radians:

```python
import math

angle_rad = math.radians(30)       # 30 degrees -> 0.524 radians
angle_deg = math.degrees(0.5)      # 0.5 radians -> 28.65 degrees
```

### Useful Python patterns

```python
# A dictionary associates each name with one value:
example = {"motor_A": 1.2, "motor_B": -0.3}
angle_A = example["motor_A"]

# Pair up two lists, entry by entry:
labels = ["A", "B"]
values = [10, 20]
for label, value in zip(labels, values):
    print(label, value)

# Copy a NumPy array so changes do not affect the original:
new_values = old_values.copy()

# A NumPy vector's shape tells you how many elements it has:
# e.g. q.shape == (6,), xyz.shape == (3,), J.shape == (3, 6)
```

The file `kinova_model.py` reads the installed Kinova URDF/Xacro. **Do not invent the link lengths or reuse dimensions from a seven-joint robot**. `kinova_ros.py` handles ROS subscriptions and trajectory publishing for the later exercises.

---

## Exercise 1 — Read the joint positions (about 15 minutes)

**File:** `01_read_joints.py`  
**Goal:** Print `joint_1` through `joint_6`, in **radians and degrees**, once per second.

### Understand the starting code

The script creates an ROS 2 node and subscribes to `/joint_states` using `sensor_msgs/msg/JointState`. Whenever ROS receives a new message, the `on_joint_states(self, msg)` callback runs. The message has fields such as:

```text
msg.name     -> list of joint names, e.g. ["joint_2", "joint_1", ...]
msg.position -> joint angles at those matching indices, in radians
msg.velocity -> joint angular velocities
```

The constant `JOINTS` defines the **required order** (`joint_1`, `joint_2`, ..., `joint_6`).

### Complete the TODOs

**TODO 1 — Associate each name with its position.** A dictionary is helpful because you can retrieve the value for a name even when the original arrays are out of order. Hint: `zip(msg.name, msg.position)` gives the matching pairs. Look back at the Python dictionary example.

**TODO 2 — Build an ordered list of six values.** Iterate through `JOINTS`, looking up each name in the dictionary. First check that *every required joint name* is present in the message. If any are missing, use `return` to skip the callback.

**TODO 3 — Convert each angle to degrees.** `math.degrees()` converts **one** float. Use a loop or a list comprehension to convert **all six** values.

### Run it

```bash
python3 01_read_joints.py
```

**What you should observe:** Six lines appear roughly once a second. The numbers are close to zero in the initial robot configuration, or different if you moved the robot in RViz. Press **Ctrl+C** to stop the program. Move a joint using MoveIt and run the script again to see a changed value.

**Self-check:** Are there exactly six joints? Does `joint_3` have the correct value if ROS supplies its position *fifth* in the message? Are the radians and degrees consistent (approximately 57.3 degrees per radian)?

**Questions:** Why might a sensor or simulator change the order in which joint names are published? What would go wrong if you wrote `joint_3 = msg.position[2]`?

---

## Exercise 2 — Move the joints using Python (about 20 minutes)

**File:** `02_move_joints.py`  
**Goal:** Move `joint_1` by **10 degrees** from its current position, keeping the other joints unchanged.

### Understand the starting code

`arm.wait_for_state()` reads all six current joint positions in the correct order. `target` should hold six **absolute target joint angles**, not a list of angle changes. The standard ROS 2 message types are:

- `trajectory_msgs/msg/JointTrajectory`: list of joint names and planned points.
- `trajectory_msgs/msg/JointTrajectoryPoint`: desired positions (and optionally velocities) at a specified time.
- `builtin_interfaces/msg/Duration`: time from the start of the trajectory.

The supplied code checks joint limits and rejects movements larger than **0.35 rad** per joint. It also waits for the active controller and reports whether the requested target was reached.

### Complete the TODOs

**TODO 1 — Copy the current configuration.** Use `.copy()`. Why would writing `target = current` be risky if you modify the array later?

**TODO 2 — Change only the first joint.** Convert **10 degrees to radians** using `np.deg2rad(10)` (or `math.radians(10)`). Add this change to the element for `joint_1` (`index 0`). Don't change the other five elements.

**TODO 3 — Construct a trajectory message.** Create an instance of `JointTrajectory()`. Fill its `joint_names` field using the already supplied `JOINTS` variable. The names and positions must be consistent.

**TODO 4 — Construct the trajectory point.** Create `JointTrajectoryPoint()`. Set its `positions` to a **Python list** of the six target values (`target.tolist()` is useful). Set `time_from_start` using `Duration` so the move takes **four seconds**, then append that point to the trajectory's `points` list.

### Run it

```bash
python3 02_move_joints.py
```

The arm should rotate its first joint by a small amount in Gazebo. A successful result should include `Goal reached: True` and final angles close to the requested configuration.

**Try next:** Edit the program to move `joint_2` by a smaller amount. Remember that each run starts from the *current*, not the original, robot pose. Do not keep adding movements until the arm hits a limit.

**Questions:** Why do we send all six joint positions even if only one changes? What is the difference between an absolute target angle and an angle increment? What does `time_from_start` change?

---

## Exercise 3 — Calculate XYZ from joint angles (about 25 minutes)

**File:** `03_forward_kinematics.py`  
**Goal:** Calculate the XYZ position (metres) of `end_effector_link` relative to `base_link`.

### What is forward kinematics?

Forward kinematics answers: **"Given the joint angles, where is the end effector?"**

A robot arm consists of links joined together. Each joint has a position and orientation relative to its parent link. To get from the robot base to the tip, multiply the transformations in order:

```text
base_link -- T1 --> link_1 -- T2 --> link_2 -- ... --> end_effector_link
```

Mathematically:

\[
T_{base,tip} = T_1 T_2 \cdots T_n
\]

Each `T` is a **4 × 4** matrix containing rotation and translation:

```text
[ r11 r12 r13   x ]
[ r21 r22 r23   y ]
[ r31 r32 r33   z ]
[   0   0   0   1 ]
```

The last column contains the XYZ position. NumPy uses `@` for matrix multiplication; ordinary `*` means element-wise multiplication, which is **not** what we want here.

### What has already been done for you?

`KinovaModel()` reads the installed six-axis robot description (URDF/Xacro). Its `model.chain` contains joints from `base_link` to `end_effector_link`, including **fixed joints**. The helper `joint_transform(joint, angle)` returns the correct 4 × 4 transform for one joint. `model.position(q)` provides a reference answer for checking your work.

### Complete the TODOs

**TODO 1 — Start with the identity transform.** An identity transform means "no translation and no rotation". Which NumPy function creates a 4 × 4 identity matrix?

**TODO 2 — Multiply all joint transforms.** Loop through `model.chain` in its existing order. Use `angles.get(joint.name, 0.0)` to obtain the joint angle, using zero for fixed joints. On each iteration, update the accumulated matrix with `@ joint_transform(joint, angle)`.

**TODO 3 — Extract the position.** From the completed 4 × 4 matrix, take rows 0–2 of column 3. Hint: NumPy slicing has the form `matrix[row_slice, column_index]`.

### Run it

```bash
python3 03_forward_kinematics.py
```

The script prints your XYZ, the supplied reference XYZ and the error in metres. **Aim for a numerical error close to zero**, apart from tiny floating-point differences.

To check with ROS TF, use a **separate terminal**:

```bash
ros2 run tf2_ros tf2_echo base_link end_effector_link
```

This returns the transform from the base frame to the end-effector frame. Compare its **translation**, not its orientation, with your script's output. If they disagree, make sure the robot hasn't moved between the two measurements and both are using the same reference frames.

**Questions:** Why does matrix multiplication order matter? Why must fixed joints be included? Why is the result in metres rather than radians?

---

## Exercise 4 — Move the end effector to a target XYZ (about 35–45 minutes)

**File:** `04_inverse_kinematics.py`  
**Goal:** Given a desired XYZ position, calculate six joint angles that reach it.

### Forward versus inverse kinematics

- **Forward kinematics (FK):** joint angles `q` -> position `p`.
- **Inverse kinematics (IK):** target position `p_target` -> a suitable configuration `q`.

For this lab, we care about XYZ **position only**, not tool orientation. There are three position coordinates but six joint variables, so **multiple configurations can reach the same XYZ**.

### Step 1 — Approximate a Jacobian

A Jacobian describes how small changes in joint angles change the end-effector position:

\[
\Delta p \approx J\,\Delta q
\]

For XYZ-only control of a six-joint arm, `J.shape == (3, 6)`. You will calculate it numerically: change *one joint at a time* by a tiny amount `epsilon` and see how much the forward-kinematics position changes.

\[
J_{:,i}\approx \frac{p(q+\epsilon e_i)-p(q)}{\epsilon}
\]

**TODO 1:** In the `for col in range(6)` loop, `shifted = q.copy()` has already been provided. Increase **just** `shifted[col]` by `epsilon`.

**TODO 2:** Call `model.position(shifted)` to get the new XYZ. Subtract the original `p`, divide by `epsilon` and store the resulting **three numbers** in `J[:, col]`.

**Shape check:** A single Jacobian column is a three-element vector. The full Jacobian has 3 rows and 6 columns.

### Step 2 — Calculate the position error

The direction we need to move is:

\[
e = p_{target} - p(q)
\]

**TODO 3:** Inside `solve_ik(...)`, calculate the current position with `model.position(q)`, then subtract it **from the target**. Be careful about the order of subtraction: reversing the sign makes the arm move away from the target.

The loop stops once `np.linalg.norm(error)` is less than **0.005 m (5 mm)** or it reaches its iteration limit.

### Step 3 — Calculate the joint update

A normal matrix inverse won't work for a **3 × 6 Jacobian** (it isn't square). We use a *damped least-squares pseudoinverse* instead:

\[
\Delta q = J^T(JJ^T + \lambda^2 I_3)^{-1}e
\]

Here:

- `J.T` transposes the Jacobian to **6 × 3**.
- `J @ J.T` has shape **3 × 3**.
- `np.eye(3)` gives the 3 × 3 identity matrix.
- `damping` (lambda) is already provided in the script.
- `np.linalg.solve(A, b)` solves `A * x = b` without explicitly calculating a matrix inverse.

**TODO 4:** Translate this equation into NumPy, using `@` for matrix multiplication and `np.linalg.solve()` for the middle matrix operation. The result `delta_q` must contain **six** joint updates.

The rest of the script restricts the size of each numerical step and clips angles to the robot's joint limits. These limits are taken from the six-axis robot description.

### Test without moving the robot

```bash
python3 04_inverse_kinematics.py
```

By default, the program constructs a **nearby reachable target from another six-joint configuration**. It prints the starting, requested and solved XYZ positions, final error and whether the solver converged. It should say **DRY RUN** and not move the Gazebo robot.

**Interpret the results:**

- `Converged: True` means the numerical position error fell below the threshold.
- `Error [m]` should be small: for example, `0.003` means **3 mm**.
- `Iterations` tells you how many updates the solver required.
- `Joint change [rad]` shows how far each joint would move.

Only once the calculation converges, the changes are modest and the simulated path is clear should you choose to execute it:

```bash
python3 04_inverse_kinematics.py --execute
```

This is **not** a collision-free motion planner. Do not run it on hardware.

**Further challenge:** Try a custom target **in `base_link` coordinates (metres)**:

```bash
python3 04_inverse_kinematics.py --x 0.3 --y 0.1 --z 0.45
```

The example coordinates are illustrative, **not guaranteed reachable**. Test as a dry run and do **not** append `--execute` unless the target has converged and you have checked the movement.

**Questions:** Why is Jacobian damping useful near singularities? Why might the solver produce different angles from the configuration you expected? Why can a mathematical IK solution be unsafe in a cluttered scene?

---

## Optional Exercise 5 — Inverse kinematics using optimisation

**File:** `05_optimisation.py`  
**Goal:** Use a general-purpose numerical optimiser to find a joint configuration that reaches the same target XYZ.

The SciPy function `least_squares(...)` tries different joint angles to make a **residual vector** small. Here, we combine two types of residual:

1. **Position error:** `predicted XYZ - requested XYZ`, a three-element vector.
2. **Movement penalty:** a small multiple of `q - q_start`, a six-element vector, which encourages the answer to remain near the initial joint configuration.

The combined residual contains **nine** values. The multiplier should be small, for example **0.01**, so position error remains the main objective.

**TODO 1:** Evaluate `model.position(q)` and subtract `target_xyz`.

**TODO 2:** Form the movement penalty using the difference between the candidate joint angles and `q_start`.

**TODO 3:** Call `least_squares()` with your residual function, the initial joint angles, bounds given by `model.lower` and `model.upper`, and `max_nfev=250`. Your instructor can provide any additional solver tolerance settings.

Start with a dry run:

```bash
python3 05_optimisation.py
```

Compare its `Error`, `evaluations` and joint movements with Exercise 4. `evaluations` and `iterations` are **different quantities**, so don't treat them as directly equivalent.

Run `--execute` only in a clear simulated scene after checking convergence and the proposed movement.

---

## Optional demonstration — Repeated XYZ waypoints

**File:** `06_xyz_waypoints.py`

This script extends Exercise 4 by solving **several** XYZ targets in sequence, similar to a task performed by a robot arm during a real manipulation application. It is adapted to the six-joint arm and ROS 2, rather than reusing the original seven-joint robot's dimensions.

It **requires a completed Exercise 4** because it loads `solve_ik()` from `04_inverse_kinematics.py` (or from an instructor-supplied solutions folder).

```bash
python3 06_xyz_waypoints.py             # calculate only
python3 06_xyz_waypoints.py --execute   # move in Gazebo after checking
```

The target sequence is generated from nearby configurations. It is **not** a collision-checked trajectory. If a waypoint is rejected, do not bypass the checks; discuss why with your instructor.

---

## Part C — Troubleshooting

| Problem | What to check |
|---|---|
| `ModuleNotFoundError: kinova_ros` or `kinova_model` | Are the helper files alongside your exercise script? Run from the lab folder, or set `PYTHONPATH` as shown in Part A. |
| `ModuleNotFoundError: rclpy` | Are you inside the ROS 2 Docker container? Source `/opt/ros/humble/setup.bash`. |
| `None` / `NotImplementedError` | An exercise TODO is still unfinished. Find all `TODO` entries in that script. |
| No `/joint_states` data | Is Gazebo running? Check `ros2 topic echo /joint_states --once` and `ros2 control list_controllers`. |
| Joint values are wrong / out of order | Always use the matching `msg.name` entry. Never assume `msg.position[2]` is joint 3. |
| Python trajectory is ignored | Check `joint_trajectory_controller` is **active**; do not issue competing commands from RViz. |
| `Target exceeds joint limits` | The move is too large or violates a limit. Return to a safe pose and use a smaller target. |
| FK result differs from TF | Check matrix multiplication order, fixed joints, and the `base_link` to `end_effector_link` reference frames. |
| IK does not converge | Try the default nearby target first; verify the Jacobian calculation and target-error sign. Custom XYZ positions may be unreachable. |
| RViz works but no window appears for a colleague | The container's GUI forwarding may differ by platform. The Python exercises can still be checked using terminal output. |

**Useful ROS 2 commands** (run in another terminal):

```bash
ros2 node list
ros2 topic list
ros2 topic info /joint_states
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 topic info /joint_trajectory_controller/joint_trajectory
ros2 run tf2_ros tf2_echo base_link end_effector_link
```

## Completion checklist

- [ ] I can launch the Kinova 6-DOF simulation and inspect a MoveIt planned trajectory.
- [ ] Exercise 1: I can print all six joint angles in the correct name order.
- [ ] Exercise 2: I can move one joint by a chosen amount with Python and verify the response.
- [ ] Exercise 3: My forward-kinematics XYZ agrees with the supplied model reference and ROS TF (when measured at the same pose).
- [ ] Exercise 4: I can obtain a small XYZ error from a default reachable target without executing movement first.
- [ ] I can explain why a successful IK calculation is **not necessarily** a safe trajectory.
- [ ] Optional: I can compare Jacobian IK with the numerical optimiser.

**Final reflection:** What information does the robot need for joint-space movement? What additional mathematical information is needed to request an XYZ end-effector position? How does MoveIt provide capabilities beyond the numerical IK methods in this lab?
