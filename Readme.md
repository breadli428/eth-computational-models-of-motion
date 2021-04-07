# Assignment 3 - Boids

## Get Started

This is an overall `readme.md` file which introduces features of the program and how they are realized in a general and comprehensive fashion.

> In particular, the required answers to the questions asked in the instruction sheet are written in **quote form**.

## Demos

YouTube video demos please click [here](https://youtube.com/playlist?list=PLhqs0Oka9VRFcWf9pR34oIPB75XVvmQ10).

There are two versions of the video demo. While the shorter version shows the feature of the program, the longer version in addition provides a full description on detailed information of the program.

## Interface

The interface of the program is shown below.

![figure: interface](imgs/interface.png)

*Figure 1: Overview of interface layout.*

There are several features that you can choose:
- Initial Velocities
- Step Size
- Force Scaler
- Boids Behavior
- Update Method

## Initial Velocities

This feature denotes the initial velocities of the Boids when initializing the system. Four options are offered:
- Zero
  - zero initial velocities
- Orthogonal
  - initial velocities are orthogonal to the initial positions. This feature aims to implement the *circular motion* of Boids as required.
- Random
  - random initial velocities
- Random Positive
  - random initial velocities with only positive values

This feature is realized in the function `void initializePositions(InitTypes init_vel)`.

## Step Size

This feature denotes the value of step size in the discretized system dynamics. A recommended value of 0.02 has been set as default.

## Force Scaler

This feature denotes the value of gain in force calculation, i.e. a scaling factor.

## Boids Behavior

This feature denotes the Boids behaviors required to be implemented including:
- Free Fall
- Separation
- Alignment
- Cohesion
- Leading
- Circular
- Collision Avoidance
- Collaboration and Adversary

### Free Fall

A *constant gravity* is applied on each Boid.

> You are free to choose any initialization method as desired.

This feature is realized in the function `TVStack freefall(TVStack positions_temp)`.

### Seperation

A slidebar will pop up and you can change affecting range of the behavior.

If two Boids have a distance smaller than the range defined above, a repulsive force that is *inversely proportional to the square of the distance* will be applied. In order to generate a mild force feedback and reduce overshoot, an offset term is considered in the force formulation.

This feature is realized in the function `TVStack separation(TVStack positions_temp)`.

### Alignment

A slidebar will pop up and you can change affecting range of the behavior.

For Boid i, if another Boid j lies within the range defined above, j is collected as a neighbor of i. The average position of the neighbors of Boid i and their distance inbetween will then be calculated and an attractive force that is *proportional to the distance* will be applied on Boid i. Meanwhile, in order to align velocity with the surrounding counterparts, a Boid will be applied a force induced by the *difference of velocity directions* between the average velocity of its neighbors in the range and itself.

This feature is realized in the function `TVStack alignment(TVStack positions_temp)`.

### Cohesion

A slidebar will pop up and you can change affecting range of the behavior.

For Boid i, if another Boid j lies within the range defined above, j is collected as a neighbor of i. The average position of the neighbors of Boid i and their distance inbetween will then be calculated and an attractive force that is *proportional to the distance* will be applied on Boid i. In order to generate a mild force feedback and reduce overshoot, a PD controller is considered in the force formulation.

This feature is realized in the function `TVStack cohesion(TVStack positions_temp)`.

### Leading

A slidebar will pop up and you can change affecting range of the behavior. In addition, a checkbox of the presence of an obstacle will appear. If the obstacle is enabled, you can further change its position and radius in the slidebar showing up.

![figure: leading](imgs/leading.png)

*Figure 2: Overview of leading layout.*

In this feature, a Boid colored blue is assigned as the leader, whose trajectory can be instructed by cursor click on the screen. This is realized in like manner as above where a PD controller is implemented considering difference between the target and current state. The position of the target is read in the function `void read_cursor(TV target_pos_read)`.

As for the followers, if the distance between a following Boid and the leader is smaller than the range defined above, an attractive force that is *proportional to the distance* will be applied. Meanwhile, a following Boid will be applied a force induced by the *difference of velocities* between the leader and itself so as to head in the same direction as the leader.

In addition, if two Boids are too close, a repulsive force that is *inversely proportional to the square of the distance* will be applied so as to maintain a sufficient
distance to each other.

How a force to avoid obstacle is generated will be explained later in Collision Avoidance feature.

This feature is realized in the function `TVStack leading(TVStack positions_temp)`.

### Circular

Please select the initial velocities to be orthogonal to initial positions to visualize the implementation of this feature. The velocities are initialized in the function `void circular_vel_init()`. And please set step size to around 0.50 to observe clear behaviors between different update rules.

A *constant centripetal force* is applied on each Boid so as to encourage circular motion in the plane.

> You are free to choose any initialization method as desired. In particular, in the case when orthogonal initial velocities are chosen, circular motion of Boids can be observed. For each of the three integration schemes, experiment with different step sizes shows their difference in energy conservation.
> 
> In the case where the explict Euler method is implemented, the radii of the circular motion of the Boids increase gradually, i.e. the Boids are shifted away from the center, showing that the total energy of the system increases steadily when the explicit Euler method is applied. This behavior is escalated with the growth of step size, which indicates that the explicit Euler method overshoots for strongly attractive fixed-points and large time steps.
> 
> In comparison, the lengths of radii are well preserved when the symplectic Euler method is implemented, which indicates that it almost conserves the energy.
> 
> Eventually in the case where the explicit midpoint method is implemented, the Boids still shift outwards, but with a slower process compared with the explict Euler method. This is because the explicit midpoint method accumulates error of order $O(h^2)$, while the explict Euler method does of order $O(h)$. Therefore, while the explicit midpoint method is more computationally intensive than the other two methods, error generally decreases faster as h approaches 0.

This feature is realized in the function `TVStack circular(TVStack positions_temp)`.

### Collision Avoidance

An obstacle is enabled and a slidebar determining its position and radius will pop up.

If a Boid has a distance from the obstacle center smaller than the obstacle radius, a repulsive force that is *inversely proportional to the square of the distance* will be applied. In order to generate a mild force feedback and reduce overshoot, an offset term is considered in the force formulation. To avoid Boids bursting into the obstacle, the positions of Boids will be reset to the surface of the obstacle once that happens.

This feature is realized in the function `TVStack collision_avoidance(TVStack positions_temp)`.

### Collaboration and Adversary

The Boids are *divided into two groups* and colored with red and blue respectively. A slidebar will pop up and you can change affecting range of the behavior. Meanwhile, you can choose the control strategy to be applied on which group in the dropdown menu showing up. Please set the range to be around 0.10 to observe clear Boids behaviors.

![figure: collaboration_and_adversary](imgs/collaboration_and_adversary.png)

*Figure 2: Overview of collaboration and adversary layout.*

As described in the instruction, the evolution of the system respects the following rules:
- if two Boids from the same group are sufficiently close, a third one is created.
- if three Boids from the same group are close to a Boid from the other group, the latter one is removed from the
system.

To divide the Boids into two groups, a group assignment is generated to determine which group each Boid belongs to. In addition, to avoid infinite reproduction which slows down the computation, a reproduction limit of one is considered where *each Boid is allowed to generate only one child* if the conditon of the first rule is fulfilled. This information is termed as reproduction permission, where 0 denotes reproduction allowed and 1 denotes not. Together with group assignment, the information of each Boid is stored in a matrix called `group_label`, where each column corresponds to a Boid and *consists of two entries* - the first one represents the group assignment while the second one represents the reproduction permission. This matrix is initialized in the function `void initializeGroupLabel()`.

These rules are realized by dynamically adding or removing a Boid (a column) in or from `positions` and `velocities` when the conditions are fulfilled. It is worth noting that the newly added Boid is positioned and has a velocity at the average positions and velocities of its parents.

Besides the implementation of the Boids behavior, a force control strategy is also adaptively applied.

For Boid i, if another Boid j lies within the 5.0 * range specified by the slidebar, j is collected as a neighbor of i. The average position of the neighbors of Boid i and their distance inbetween will then be calculated and an attractive force that is *proportional to the distance* will be applied on Boid i, which encourages the Boids from the same group to form large communities. In order to generate a mild force feedback and reduce overshoot, a PD controller is considered in the force formulation. This strategy promises two aspects that motivates the increment of the group relative size. On the one hand, the reproduction allowed group members are to be gathered so as to generate child Boids. On the other hand, all Boids from the same group including the reproduction prohibited group members are collected to prevent from bursting into the adversarial group communities and being eliminated.

In addition to the force between Boids from the same group, a *repulsive* force is acted on a Boid if there are more Boids within 5.0 * range from the different group than those from the same group by at least 3. The intuition behind is simple - when there are more adversaries than companions, the Boid will drive away to prevent from being eliminated. Conversely, in the case where there are more companions than adversaries by at least 3, in order to increase the relative group size, an *attractive* force is applied to the Boid to encourage its motion along with its community towards the community from the other group so as to eliminate the counterparts.

Finally, if two Boids are too close, a repulsive force that is *inversely proportional to the square of the distance* will be applied so as to maintain a sufficient
distance to each other.

Please refer to the code annotation for a detailed documentation of realization.

> You are free to choose any initialization method and any of the update rule as desired. In particular, the symplectic Euler method with random initial positions and velocities can be used to observe the evolution of the populations as required.
> 
> When no control strategy is applied, the populations of the two groups behave well in accordance with the rules defined above. However this is not the best case to observe the evolution of the system, since the Boids will scatter away due to their random initial velocities.
> 
> When the control strategy designed above is applied to only one of the two groups, say group colored red, it can be observed that the red Boids will attempt to gather and form large communities in the first place, while avoiding bursting into blue communities with large sizes. Then, with a large member of companions, the red Boids in the communities will search for single blue counterparts within their observation range and approach them if they have an inferior size so as to eliminate them while increasing relative group size.
> 
> It gets interesting when the control strategy is applied to both groups. While each group presents the similar behaviors as it is controlled singly, namely first search for companions and form large communities while avoiding being caught in communities formed by Boids from the other group, they demonstrate an interesting catch-and-escape behavior, where a community from one group with smaller size tries to escape from the pursuit by a larger community from the other group. Moreover, the escaping community will seek chances from time to time to eliminate single Boids from its catcher which overshoot while chasing.

Please refer to the video demos for a better understanding of the group behaviors and population evolution process.

This feature is realized in the function `TVStack collaboration_adversary(TVStack positions_temp)`.

### Recommended Parameters

To observe an articulated simulation result, a set of recommended parameters are provided. In most cases, they are set to be the default value. An overview of recommended parameters is listed here:

| Initial Velocities 	| Step Size 	| Force Scaler 	| Boids Behavior              	| Range 	|
|:--------------------:	|:-----------:	|:------------:	|:----------------------------:	|:-------:	|
| -                  	| 0.02      	| 2.00         	| Free Fall                   	| -     	|
| -                  	| 0.02      	| 2.00         	| Separation                  	| 0.50  	|
| -                  	| 0.02      	| 2.00         	| Alignment                   	| 0.50  	|
| -                  	| 0.02      	| 2.00         	| Cohesion                    	| 0.50  	|
| -                  	| 0.02      	| 2.00         	| Leading                     	| 0.50  	|
| Orthogonal         	| 0.50      	| 2.00         	| Circular                    	| -     	|
| -                  	| 0.02      	| 2.00         	| Collision Avoidance         	| -     	|
| -                  	| 0.02      	| 2.00         	| Collaboration and Adversary 	| 0.10  	|

*Table 1: Recommended Parameters.*

## Update Method

This feature denotes the time integration rules:
- Explicit Euler
- Symplectic Euler
- Explicit Midpoint

For each Boids behavior, you are free to choose any of the three update rules. Here is a brief recap on how these update rules work, as indicated in the instruction:

### Explicit Euler

![figure: explicit_euler](imgs/explicit_euler.png)

This feature is realized in the function `void explicit_euler(std::function<TVStack(TVStack)> method)`.

### Symplectic Euler

![figure: symplectic_euler](imgs/symplectic_euler.png)

This feature is realized in the function `void symplectic_euler(std::function<TVStack(TVStack)> method)`.

### Explicit Midpoint

![figure: explicit_midpoint](imgs/explicit_midpoint.png)

This feature is realized in the function `void explicit_midpoint(std::function<TVStack(TVStack)> method)`.
