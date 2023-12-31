# Cognitive-Robotics
Cognitive Robotics's Miniproject : RRT - planning for "Dubin Car".
Extention of the project RRT_Dubins of FelicienC on GitHub (https://github.com/FelicienC/RRT-Dubins)

## Miniproject 2: 
Goals: 
 - Explain the geomentry of the robot and how RRT is working 
 - Inspect the Code RRT_algorithm and modify it 
 - Generate comment in different situation and explain the differences between the orignal algorithm and RRT-Dubin car
 - Test the code in high and low dimensional space, with different test about the radius of curvature 
 - Extend the code to work also for a circular Robot of diameter D
 - Create a Jupeter notebook file where all is executable


## Table Of Contents

- [RRT Dubins](#RRT-Dubins)
  * [Requirements](#requirements)
  * [Setup](#setup)
  * [Dubins Path](#dubins-path)
    + [Different kind of metric](Different_kind_of_metric)
    + [Usage](#usage)
      + [Static Environment](#Static_Environment)
  * [RRT](#rrt)
    + [How it is work?](#how-it-is-work)
    + [Differences between RRT and RRT-Dubin](#Differences_between_RRT_and_RRT-Dubin)
      + [Original RRT (Rapidly-exploring Random Tree)](#Original_RRT_Rapidly-exploring_Random_Tree))
      + [RRT Dubin](#RRT_Dubin)
    + [Graph of RRT and RRT-Dubin](#Graph_of_RRT_and_RRT-Dubin)
      + [Low or High dimentional space works better?](#Low_or_High_dimentional_space_works-better?)
      + [Avarage of the path, increasing the radius and diameter of the robot](#Avarage_of_the_path,_increasing_the_radius_and_diameter_of_the_robot)
      + [Dynamic Environment](#Dynamic_Environment)
        + [Dynamic Environment static](#Dynamic_Environment_static)
        + [Dynamic Environment moving](#Dynamic_Environment_moving)
      + [Description functions](#Description_functions) 
  * [References](#References)



## Setup
To run the code, simply download the repository and execute the 'script.ipynb' file. It contains various modules from which you can individually view the output for each function.
The execution may take some time due to the high number of iterations for certain functions.

## Requirements

- [Scipy](https://www.scipy.org)
- [Numpy](https://numpy.org)
- [Matplotlib](https://matplotlib.org)
- [Shapely](https://pypi.org/project/Shapely)

## Dubins path

Dubins path refers to the shortest curve that connects two points in the two-dimensional Euclidean plane if only one direction of movement is permitted (the vehicle can only go forward). The reason the Dubins path has only six types of combinations is due to the restrictions imposed by the model. This type of path is necessarly in the form of one of 6 types, combination of right turns (**R**), left turns (**L**) and straight segments (**S**): **RLR, LRL, LSR, RSL, LSL, RSR**.



### How does it work ?

As we want to have the shortest path among the 6 potential candidates, we compute the lenght of the path we would obtain if we followed each of the trajectories. To do so, we need to separate three cases:
  1. **LSL** and **RSR** : Two turns in the same direction separated by a straight segment
  2. **LSR** and **RSL** : Two turns in opposite directions separated by a straight segment
  3. **RLR** and **LRL** : Three turns

These combinations represent the shortest paths that a vehicle with a fixed turning radius can take from an initial configuration to a target configuration. By restricting the vehicle's movements to these combinations, the Dubins path planning problem becomes simpler and more computationally efficient.

Here we plot some image of how these six configuratio type woks in way to move between one node and another:
<p align="center">
  <img src="docs/lsr.png"><img src="docs/lrl.png"> <img src="docs/lsl.png"> 
</p>

### Different kind of metric

The RRT (Rapidly-exploring Random Tree) algorithm supports the utilization of various metrics to measure distances. Initially, it employed two metrics, namely local and Euclidean. However, additional metrics such as Chebyshev, Minkowski, Hamming, and cosine distances have been incorporated to facilitate a comprehensive analysis of this algorithm. By employing these diverse metrics, it becomes possible to discern and appreciate the distinctions in the outcomes yielded by the RTT algorithm.


 - Local distance: It represents the specific cost or distance metric used within the algorithm in this case is applied using the sort algorithm of the distance.

 - Euclidean distance: The Euclidean distance is a measure of the straight-line distance between two points in a two dimensional space. It is calculated using the Pythagorean theorem and represents the length of the shortest path between the points.
 <p align="center">
   <img src="docs/euclidian_distance.png">
 </p>
 - Chebyshev distance: The Chebyshev distance, also known as the chessboard distance, calculates the maximum difference between the coordinates of two points along each dimension. It represents the distance in terms of the number of moves required by a king in a chessboard.
 <p align="center">
   <img src="docs/chebyshev_Distance.png">
 </p>
 - Minkowski distance: The Minkowski distance is a generalized distance metric that includes both the Euclidean and Manhattan distances as special cases. It is defined by a parameter "p" and can be used to measure distances in multi-dimensional spaces.
 <p align="center">
   <img src="docs/Minkiwski_Distance.png">
 </p>
 - Hamming distance: The Hamming distance is used to compare two equal-length strings or binary vectors. It measures the number of positions at which the corresponding elements are different, representing the minimum number of substitutions required to change one string into the other.

 - Cosine distance: The Cosine distance calculates the dissimilarity between two vectors based on the cosine of the angle between them. It is often used to measure the similarity or dissimilarity between documents represented as vectors, where the magnitude and direction of the vectors are taken into account.
 <p align="center">
   <img src="docs/cosine_distance.png">
 </p>
This graph illustrates the average distances for each metric. A total of 50 epochs were executed, utilizing a curvature radius of 5 and incorporating 10 objects within the environment:

<p align="center">
  <img src="docs/graph 50 ephocs.png">
</p>

It is evident from the data that there is a significant disparity between the Minkowski and Hamming metrics. This indicates that the Hamming metric requires more space to reach the goal, whereas the Minkowski metric exhibits closer distances, indicating a more efficient path.

### Usage 

The Dubins path are implemented in the Dubins class. 
In this class is possible to modify the Radius curvatures, is the radius of the turn used in all the potential trajectories, the Point-Separation, the distance between points of the trajectories. More points increase the precision of the path but also augments the computation time of the colision check, and the Robot_diameter that is the diameter of the circular robot.

```python

from dubins import Dubins

# We initialize the planner with the turn radius, the desired distance between consecutive points, and robot_diameter
local_planner = Dubins(radius=2, point_separation=.5, robot_diameter=10)

# We generate two points, x, y, psi
start = (0, 0, 0) # heading east
end = (20, 20, 3.141) # heading west

# We compute the path between them
path = local_planner.dubins_path(start, end)
```
The variable *path* now contains the sequence of points (x, y) linking the start and finish points given as input with only turns of a defined radius and straight line in the form of a (2xn) numpy array.

```python
import matplotlib.pyplot as plt

plt.plot(path[:, 0], path[:, 1])
```

### Usage

The rapidly exploring random tree is implemented in the RRT class.
In order to use it, the environment needs to be defined first. To start, two types of environments can be used.

#### Static Environment

In the static environment, the obstacles are polygonal and are stored in a binary search tree in order to increase the speed of the colision check we can modify the number of the obstacles here before to execute the RRT code.
The following code initializes an Environment:
```python
from environment import StaticEnvironment
from rrt import RRT

# We create an environment of 100x100 meters, with 100 obstacles
env = StaticEnvironment((100, 100), 100)
env.plot()
```
<p align="center">
  <img src="docs/without_nodes.png">
</p>

```python
# We initialize the tree with the environment
# The precision needed to stop the algorithm. In the form (delta_x, delta_y, delta_psi)
# the float diameter of the circular robot
# radius to curve
# point_separation is the distance between points of the trajectories. More points increases
# the precision of the path but also augments the computation time of the
# colision check.
rrt = RRT(env, (5, 5, 1), diameter, radius, point_separation)

# We select two random points in the free space as a start and final node
start = env.random_free_space()
end = env.random_free_space()

# We initialize an empty tree
rrt.set_start(start)

# We run 100 iterations of growth
rrt.run(end, nb_iteration=100)

env.plot()
rrt.plot(nodes=True)
```
<p align="center">
  <img src="docs/output.png">
</p>

## RRT

### How does it work ?

A Rapidly-exploring Random Tree (RRT) follows an incremental approach to construct a search tree that gradually enhances its resolution, aiming to densely cover the space in the long run. The tree initiates from a starting configuration and expands by utilizing random samples from the search space. When a sample is drawn, an attempt is made to establish a connection between it and the nearest state in the tree. If the connection is feasible, meeting the requirements of passing entirely through free space and adhering to all constraints, the new state is added to the tree.

By employing uniform sampling of the search space, the likelihood of expanding an existing state is directly proportional to the size of its Voronoi region. The Voronoi region represents the collection of points that are closer to this state than to any other state in the graph. Since the states located on the frontier of the search possess the largest Voronoi regions, the tree naturally extends its expansion towards extensive unexplored areas. Consequently, the tree expands rapidly, effectively exploring the search space.

In this project we also add a costrain about the robot, infact it is considered like a circular robot with a dimension of his area on the environment. That area is defined for each function specifying the diameter of the shape. 
This imply that during the RRT algorithm will not consider the path of the nodes where the shape of the robot touching the objects in the environment.  

### Differences between RRT and RRT-Dubin
The key difference between RRT (Rapidly-exploring Random Tree) and RRT Dubin lies in the type of motion that the trees consider.

#### Original RRT (Rapidly-exploring Random Tree):

 - RRT is a sampling-based algorithm commonly used for motion planning in robotics.
 - It constructs a tree structure incrementally by randomly sampling the configuration space and expanding the tree towards unexplored regions.
 - The connections between the nodes in the tree are made by attempting to connect the sampled configuration to the nearest existing node in the tree.
 - RRT is suitable for systems with continuous and non-holonomic motion, such as wheeled robots or vehicles.
 - It does not consider any specific motion model or constraints, allowing for general exploration of the configuration space.
 - RRT can handle systems with both translational and rotational motions, but it may not guarantee optimality in the generated paths.

#### RRT Dubin:

 - RRT Dubin is an extension of the RRT algorithm specifically designed for systems with Dubin's car-like motion constraints.
 - Dubin's car is a simplified model that represents a vehicle with fixed forward speed and a limited turning radius.
 - RRT Dubin takes into account these motion constraints to generate feasible paths for a Dubin's car.
 - The connections between nodes in the tree are made using Dubin's maneuvers, which are the shortest paths considering the car's motion model.
 - RRT Dubin guarantees the optimality of the generated paths with respect to the Dubin's car model.
 - This variant of RRT is commonly used in applications involving car-like robots or vehicles where the turning radius is restricted.

In summary, RRT is a general-purpose sampling-based algorithm for motion planning, while RRT Dubin specifically caters to systems with Dubin's car-like motion constraints, ensuring optimality in the generated paths within those constraints.

### Graph of RRT and RRT-Dubin 

<p align="center">
  <img src="docs/difference rrt original and dublin.png">
</p>
It is possible to observe a straightforward comparison between two algorithms: the original RRT depicted in green and the RRT-Dubins represented in red.

<p align="center">
  <img src="docs/difference rrt original and dublin2.png">
</p>
Both alorithms successfully reach the goal

<p align="center">
  <img src="docs/difference rrt original and dublin3.png">
</p>
A graph where original RRT have succed to reach the goal but RRT-Dubins no



#### Low or High dimentional space works better? 
In the code is possible to execute the 'test_rrt_loop' function, that is used to tested the RRT method for ten epochs in high and low dimensional space and plotting the results: 


<p align="center">
  <img src="docs/Graph.png">
</p>

Here is a possible visualization of the average distance that can be achieved between high and low dimensional spaces.

The graph depicts two axes: one representing the epochs executed and the other representing the average distance. As the dimensionality increases, the average distance between points tends to decreases.

#### Avarage of the path, increasing the radius and diameter of the robot 
In this experiment, we generated with 50 iteration of the avarage path for a RRT-dubins robot increasing the radius and also the diameter of the robot, so increasing 2 costraing at time, the value are [1,2,4,6,8,10,12]:

<p align="center">
  <img src="docs/avarage in different value of radius1.png">
</p>

As we increase the radius of curvature of our robot, it becomes increasingly noticeable how easy it is to observe the average improvement, which aligns perfectly with our expectations.

Here other two example, the first one, increasing only the radius but let the diameter the same (diameter = 5), in the second one the opposite (radius = 4). For that experiment it's set the environment at 30 objects: 

<p align="center">
  <img src="docs/avarage in different value of radius but same diameter =5 second test.png">
</p>

Avarage with different radius value and same diameter = 5

<p align="center">
  <img src="docs/avarage in different value diameter but same radius=4.png">
</p>
Avarage with different diameter value and same radius = 4

#### Dynamic Environment
In the dynamic environement, two options are available: the obstacles can either move or stay static. In both cases, the tree is pruned of the unreachable nodes once they are passed.

```python
from dynamic_environment import DynamicEnvironment
from rrt import RRT

env = DynamicEnvironment((100, 100), 5, moving=False)

```

```python
# We initialize the tree with the environment
diameter = 5
rrt = RRT(env, (5, 5, 1), diameter)

start = (50, 1, 1.57) # At the bottom of the environment
end = (50, 99, 1.57) # At the top of the environment

# Initialisation of the tree, to have a first edge
rrt.set_start(start)
rrt.run(end, 200, metric='local')

```

However, to display several frames at different timestamps, a small loop is required, as follows:
```python
# Initialisation of the position of the vehicle
position = start[:2]
current_edge = rrt.select_best_edge()

# We let it run for a few steps
time = 0
for i in range(500):
    time += 1
    # We check if we are on an edge or if we have to choose a new edge
    if not current_edge.path:
        time = rrt.nodes[current_edge.node_to].time
        current_edge = rrt.select_best_edge()
    # Update the position of the vehicle
    position = current_edge.path.popleft()
    # Update the environment
    #   The frontiers of the sampling and the obstacles
    env.update(position)
    #   The position of the goal
    end = (50, position[1]+90, 1.57)
    # Continue the growth of the tree, we try to add only 2 nodes
    rrt.run(end, 2, metric='local')
    # Ploting and generating an image (the most time consuming step)
    env.plot(time, display=False)
    rrt.plot(file_name='./images/moving'+str(i)+'.png', close=True, nodes=True)
```
This code executes relatively slowly due to the time needed to plot every single frame with matplotlib.
Here is the result obtained by concatenating all the produced images into one gif file.



##### Dynamic Environment static

<p align="center">
  <img src="code/static.gif">
</p>

##### Dynamic Environment moving

<p align="center">
  <img src="docs/moving.gif">
</p>

#### Description functions
These function are utils function in main script:

 - **Test_dubins** : Execute the class Dubins giving (radius, point_separation, robot_diameter)
 - **Test_environment** : Execute the class environment and more specific StaticEnvironment with the plotting
 - **test_rrt** : First function where the previus class are joint together to plotting the first test of RRT class
 - **test_distance** : This function plot the avarege of the distante generated by different value of radius. It execute for 50 iteraton in local metric and 8 objects in the environment
 - **rrt_image** : This plotting the different paths found for 3 kind of value radius each for 3 different dimentional space
 - **test_planner** : Plot the two path generated by RRT and RRT-Dubins in the same environmnent with 20 objects
 - **rrt_metrics** : This function calculate and plot graph of avarage RRT dubins in different metric. The metric used are : local, euclidian, chebyshev, minkowski, hamming, cosine
 - **test_rrt_loop** : This function plot the graph about the differences avarage path given by execute the algorithm in low or hight dimentional space in local metric and 100 objects in the environment.

## References

[Dino Živojević, Jasmin Velagić] "Path Planning for Mobile Robot using Dubins-curve based RRT Algorithm with Differential Constraints" 61 st International Symposium ELMAR-2019, 23-25 September 2019, Zadar, CroatiaUniversity,Faculty of Electrical Engineering/University of Sarajevo
Sarajevo, Bosnia and Herzegovina


[FelicientC]([http://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf](https://github.com/FelicienC/RRT-Dubins)) "RRT-Dubins" https://github.com/FelicienC/RRT-Dubins

[wolfram](https://demonstrations.wolfram.com/ShortestPathForTheDubinsCar/) Shortest Path for the Dubins Car
