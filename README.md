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
