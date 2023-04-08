# graphSLAM
## Graph SLAM
Graph SLAM can hancdle large number of features.

[image1]: img/a1.png "img1"
[image2]: img/a2.png "img2"
[image3]: img/a3.png "img3"
[image4]: img/a4.png "img4"
[image5]: img/a5.png "img5"
[image6]: img/a6.png "img6"
[image7]: img/a7.png "img7"
[image8]: img/a8.png "img8"


[image9]: img/b1.png "img9"
[image10]: img/b2.png "img10"
[image11]: img/b3.png "img11"
[image12]: img/b4.png "img12"
[image13]: img/b5.png "img13"
[image14]: img/b6.png "img14"
[image15]: img/b7.png "img15"


### Graphs
![alt text][image1]

m1 : features

x0, x1 : poses

### Front-End vs Back-End
![alt text][image2]

`Front-End`: The front-end of GraphSLAM looks at how to construct the graph using the odometry and sensory measurements collected by the robot. This includes interpreting sensory data, creating the graph and continuing to add nodes and edges to it as the robot traverses the environment. Identifying whether features in the environment have been previously seen.


`Back-End`: The input to the back-end is the completed graph with all of the constraints. The output is the most probably configuration of robot poses and map features. Graph Optimization process that takes all the constraints and find the system configuration that produces the smallest error.

Graph optimiation, the process of minimizing the error present in all of the constraints in the graph. Application usage is `maximum likelihood estimation (MLE)` to structure and solve the optimization problem for the graph.

probability of normal distribution for this measurement: 

$P(X=x) = \frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{(z_{1}-(x_{0}+m_{1}))^2}{2\sigma^2}}$

The robot takes a measurement of a feature, $m_{1}$, and it returns a distance of 1.8 metres.

![alt text][image3]

With two measurements, the most probable location of the feature can be represented by the product of the two probabilities assume second measurement is 2.2.

$P(X=x) = \frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{(z_{1}-(x_{0}+m1_{1}))^2}{2\sigma^2}} * \frac{1}{\sigma\sqrt{2\pi}}e^{-\frac{(z_{1}-(x_{0}+m2_{1}))^2}{2\sigma^2}}$

![alt text][image4]


$J_{GraphSLAM} =\sum_{t} {\frac{(x_{t}-(x_{t-1}+u_{t}))^2}{2\sigma^2}} + \sum_{t} {\frac{(z_{t}-(x_{t}+m_{t}))^2}{2\sigma^2}}$

This previous are working with 1D graphs (forward or backwards)

## Multi-Dimensional Graphs (n-D)

The equations for the contraints:

`m-D measurement constraint`:  

$(z_t - h(x_t, m_j))^T Q_t-1 (z_t - h(x_t, m_j))$


where h() represent the `measurement function`, and $Q_{t}$ represent the `covariances of the measurement noise`.

`n-D Motion constraint`: 

$(x_t - g(u_t, x_{t-1}))^T R_{t-1}^{-1} (x_t - g(u_t, x_{t-1}))$


where g() represent the `motion function` and $R_{r}$ represent `motion noise`.

The `multidimensional formula` for the sum of all constraints: $J_{GraphSLAM} = x_0^T \ohm x_0 + ∑_{t}(x_{t} - g(u_{t}, x_{t-1}))^{T} R_{t}^{-1} (x_{t} - g(u_{t}, x_{t-1})) + ∑_{t} (z_{t} - h(x_{t}, m_{j}))^{T} Q_{t} (z_{t} - h(x_{t}, m_{j}))$

The first elements in the sum is the inital constraints. It sets the first robot pose to equal to the origin of the map. The `covariance`, $Ω_{0}$ represents complete confidence.
 $\ohm_{0}$ =

⎡∞ 0 0⎤

⎢0 ∞ 0⎥

⎣0 0 ∞⎦

## Information Matrix and Vector
![alt text][image5]

Through this information matrix and information vector have been populated, the path and map can be recovered by the following operation,

$\mu = \ohm^{-1} ξ$

where the result is a vector, $\mu$ defined over all poses and features, containng the best estimate for each. 

### Topology
In `linear graph`, if the robot moves environment without ever returning to a previously visited loaction than the topology is linear.Such a graph will produce a rather sparse matrix that, with some effort, can be reordered to move all non-zero elements to near the diagonal.

In `cyclical graph`, a more common topology is cyclical, in which a robot revisits a location that it has been to before, after some time has passed. In such a case, features in the environment will be linked to multiple poses - ones that are not consecutive, but spaced far apart. The further apart in time that these poses are - the more problematic, as such a matrix cannot be reordered to move non-zero cells closer to the diagonal. The result is a matrix that is more computationally challenging to recover. However,  a `variable elimination` algorithm can be used to simplify the matrix, allowing for the inversion and product to be computed quicker.

### Variable Elimination Algorithm
Variable elimination can be applied iteratively to remove all cyclical constraints. Just like it sounds, variable elimination entails removing a variable (ex. feature) entirely from the graph and matrix. This can be done by adjusting existing links or adding new links to accommodate for those links that will be removed.

After the previous image show above, the following step is elimation of m1. You can see that in this process the matrix will have five cells reset to zero (indicated in red), and four cells will have their values adjusted (indicated in green) to accommodate the variable elimination. Similarly, the information vector will have one cell removed and two adjusted.

![alt text][image6]

This process is repeated for all of the features, and in the end the matrix is defined over all robot poses. At this point, the same procedure as before can be applied $\mu = \ohm^{-1} ξ$


## Nonlinear Constraints
For nonlinear models in SLAM, most motion and measurement constraints are nonlinear, and must be linearized before they can be added to the information matrix and information vector. Therefore, apply the same procedure in the EKF Localization to linearize nonlinear constraints for SLAM.

### Linearizing Constraints
A linearization of the measurement and motion constraints is 

$g(u_{t}, x_{t-1}) ≃ g(u_{t}, μ_{t-1}) + G_{t}(x_{t-1} - μ_{t-1})$

$h(y_{t}) ≃ h(μ_{t}) + H_{t}^{i} (y_{t} - μ_{t})$

To linearize each constraint, $\mu_{t-1} or \mu_{t}$ to linearize. Apply only the motion constraints to create a pose estimate,[x_{0},...,x_{t}] , and use this primitive estimate in place of $\mu$ to linearize all of the constraints. Then, once all of the constraints are linearized and added to the matrix and vector, a solution can be computed as before, using $\mu = \ohm^{-1} ξ$


## RTAB-Map (Real-Time Appeaarance-Based Mapping)
Appaarance-Based SLAM means that the alogrithm uses data collected from vision sensors to localize the robot and map the environment. In this methods, a process called loop closure that is used to determine whether the robot has seen a location before.

Loop closure is the process of finding a match between the current and previously visitd locations in SLAM. There are two types of loop closure detection: local and global.

Local: Many probabilistic SLAM methods use local loop closure detection where matches are found between a new observation and a limited map region. The size and location of this limited map region is determined by the uncertainty associated with the robot's position.

Global: A new  location is compared with previously viewed locations. If no match is found the new location is added to memory.
