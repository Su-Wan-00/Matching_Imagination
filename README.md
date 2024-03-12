# Matching Imagination
__Put the Lid on It! Robot Imagination of Container Closure via Physical Simulations__

Container Imagination is a method which enables robot to ``imagine'' the matching of unseen open containers and lids via physical simulation. With the matching imagination, the robot is able to select the proper lid for a certain container, and find the optimal matching pose of the lid.
<p align="center">
<img src="doc/put_the_lid_on_it.gif" width=50% alt="Drawing">
</p>

If you have any questions or find any bugs, please let me know: <e1192539@e.nus.edu>
# Abstract
Matching open containers and lids is a very common and important task in human life. In this paper, we propose a novel method for robots to ``imagine'' the matching of unseen open containers and lids via physical simulation. Open container imagination is conducted initially to generate the footprint. The footprint is analyzed to determine the relative pose between the container and lid. Then the optimal matching pose is identified by carrying out matching imagination. Experiments were conducted in both simulation and real-world scenarios. Our method outperforms a well-known deep learning method when selecting the proper lid for the container in simulation. In real-world experiments, our method reaches a success rate of 97.78\% when the robot autonomously puts the lid on the container.
# Dependencies
The project has been tested on Ubuntu 22.04 with python 3.11.
The required package to run this repo is listed as follows:

* pybullet 3.25
* trimesh 4.1.5
* matplotlib 3.8.0
* scikit-learn 1.4.1
* shapely 2.0.3
* scipy 1.11.4

# Usage
