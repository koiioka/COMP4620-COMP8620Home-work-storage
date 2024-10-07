# COMP4620-COMP8620 Project Assignment #5
## Novelty 1: Slope
### Task a: How to Run the Adapted Algorithm on the Novel Domain
**1. Operating System:**

- The algorithm was developed and tested on `Ubuntu 22.04` with `ROS 2 Humble` installed.
- Ensure that `Gazebo` is installed for simulating the environment.

**2. Dependencies:**

- ROS 2 Humble: Ensure that `Nav2`, `Gazebo`, and `TurtleBot3` packages are installed.

- The following ROS 2 packages are required:
`turtlebot3_navigation2`
`turtlebot3_gazebo`

- Ensure that the environment variables for TurtleBot3 are correctly set:
`export TURTLEBOT3_MODEL=waffle`

**3. Step-by-Step Guide:**

- Download the files `slope.yam`, `slope.pgm`, `slope_world.world`, `slope_world.launch.py`, `new_navigation2.launch.py`, `modified_nav2_params.yaml`

- Place `slope.yaml`, `slope.pgm` into the `maps/` folder under the home directory

- Place `slope_world.world` into `turtlebot3_gazebo/worlds/` folder under the turtlebot3 workspace

- Place `slope_world.launch.py` into `turtlebot3_gazebo/launch/` folder under the turtlebot3 workspace

- Place `new_navigation2.launch.py` into `\opt\ros\humble\share\turtlebot3_navigation2\launch`

- Place `modified_nav2_params.yaml` into `\opt\ros\humble\share\turtlebot3_navigation2\param`

- Start the Gazebo simulation with the slope world:
`ros2 launch turtlebot3_gazebo slope_world.launch.py`

- Start the navigation stack with the adjusted algorithm:
`ros2 launch turtlebot3_navigation2 new_navigation2.launch.py use_sim_time:=True map:=maps/slope.yaml`

- Run the Robot:
In RViz, you can set an initial pose and goal for the robot. The robot should be able to navigate the slope using the adapted parameters.

### Task b: Modifications to the Baseline Algorithm
**1. Baseline Algorithm:**

- The baseline algorithm used the default Nav2 stack and TurtleBot3 navigation setup. It struggled with navigating novel terrains such as slopes, as the robot’s sensors would prematurely detect the slope as an obstacle and stop navigation.

**2. Modifications:**

- Sensor Range Adjustments: The most significant change was reducing the sensor range in both the local and global costmaps (`obstacle_max_range` and `raytrace_max_range`) to allow the robot to approach the slope before detecting it as an obstacle. This change was made in both the `obstacle_layer` and `voxel_layer`.

**3. Ensuring Performance on Standard Instances:**

- The modifications made to the sensor range were specifically designed for scenarios involving changes in elevation (such as the slope). For flat, standard navigation tasks, these adjustments do not interfere with regular obstacle avoidance since obstacles closer than the adjusted range are still detected in time.

### Task c: Results
The following table presents a comparison of the baseline algorithm and the adapted algorithm when navigating the slope.
|Metric|Baseline Algorithm|Adapted Algorithm|
|--|--|--|
|Success Rate|0% (Stuck before slope)|100% (Successfully climbs slope)|
|Time to Reach Goal|N/A (Fails to reach)|60s|

### Task d: Discussion on the Adapted Algorithm’s Performance
**1. Why the Adapted Algorithm Performs Better:**

- The adapted algorithm performs better in navigating the slope due to the reduction in sensor range, which allows the robot to approach the slope before detecting it as an obstacle. The original algorithm was overly cautious, stopping the robot before it could even reach the slope.

**2. Effectiveness on Different Novelties:**

- The adapted algorithm is highly effective on terrain that involves changes in elevation, as it allows the robot to approach slopes and navigate them with more flexibility.
However, this approach might be less effective in environments with obstacles that require more sensitive detection (e.g., narrow passages or tight corners), as the reduced sensor range could cause the robot to come dangerously close to obstacles before detecting them.
Thus, while the modifications improve performance on slopes, careful consideration is needed to avoid degrading performance in environments with a high density of small, low-lying obstacles.

## Novelty 2:
### Task a: How to Run the Adapted Algorithm on the Novel Domain
**1. Operating System:**

**2. Dependencies:**

**3. Step-by-Step Guide:**


### Task b: Modifications to the Baseline Algorithm
**1. Baseline Algorithm:**

**2. Modifications:**

**3. Ensuring Performance on Standard Instances:**


### Task c: Results
The following table presents a comparison of the baseline algorithm and the adapted algorithm on the novel instance.
|Metric|Baseline Algorithm|Adapted Algorithm|
|--|--|--|
|Success Rate| | |
|Time to Reach Goal| | |

### Task d: Discussion on the Adapted Algorithm’s Performance
**1. Why the Adapted Algorithm Performs Better:**

**2. Effectiveness on Different Novelties:**


## Novelty 3:
### Task a: How to Run the Adapted Algorithm on the Novel Domain
**1. Operating System:**

**2. Dependencies:**

**3. Step-by-Step Guide:**


### Task b: Modifications to the Baseline Algorithm
**1. Baseline Algorithm:**

**2. Modifications:**

**3. Ensuring Performance on Standard Instances:**


### Task c: Results
The following table presents a comparison of the baseline algorithm and the adapted algorithm on the novel instance.
|Metric|Baseline Algorithm|Adapted Algorithm|
|--|--|--|
|Success Rate| | |
|Time to Reach Goal| | |

### Task d: Discussion on the Adapted Algorithm’s Performance
**1. Why the Adapted Algorithm Performs Better:**

**2. Effectiveness on Different Novelties:**


## Novelty 4:
### Task a: How to Run the Adapted Algorithm on the Novel Domain
**1. Operating System:**

**2. Dependencies:**

**3. Step-by-Step Guide:**


### Task b: Modifications to the Baseline Algorithm
**1. Baseline Algorithm:**

**2. Modifications:**

**3. Ensuring Performance on Standard Instances:**


### Task c: Results
The following table presents a comparison of the baseline algorithm and the adapted algorithm on the novel instance.
|Metric|Baseline Algorithm|Adapted Algorithm|
|--|--|--|
|Success Rate| | |
|Time to Reach Goal| | |

### Task d: Discussion on the Adapted Algorithm’s Performance
**1. Why the Adapted Algorithm Performs Better:**

**2. Effectiveness on Different Novelties:**


## Novelty 5:
### Task a: How to Run the Adapted Algorithm on the Novel Domain
**1. Operating System:**

**2. Dependencies:**

**3. Step-by-Step Guide:**


### Task b: Modifications to the Baseline Algorithm
**1. Baseline Algorithm:**

**2. Modifications:**

**3. Ensuring Performance on Standard Instances:**


### Task c: Results
The following table presents a comparison of the baseline algorithm and the adapted algorithm on the novel instance.
|Metric|Baseline Algorithm|Adapted Algorithm|
|--|--|--|
|Success Rate| | |
|Time to Reach Goal| | |

### Task d: Discussion on the Adapted Algorithm’s Performance
**1. Why the Adapted Algorithm Performs Better:**

**2. Effectiveness on Different Novelties:**


## Contributions:
|Name|UID|Contribution|
|--|--|--|
|Hexuan Meng|u7605165|Novelty 1: Slope|
| | | |
| | | |
| | | |
| | | |

