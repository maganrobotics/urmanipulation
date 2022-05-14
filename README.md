# Universal Robots UR3e Planning & Control System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This is an educational resource for SZTU students to learn about ur manipulation. This system original from “WRS” at Osaka University.


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing
purposes.



### Prerequisites

The following packages are needed to run this system.



```
panda3d>=1.10.7 # vital, visualization
numpy>=1.17.4 # vital, matrix computation
pycollada>=0.7.1 # optional, required by trimesh to load dae files
opencv-python>=4.4.0.46 # optional, required by vision
opencv-contrib-python>=4.4.0.46 # optional, required by vision
scikit-learn>=0.23.2 # vital?
Qpanda3D==0.2.8 # must be this versions 
```

### Installation

A step by step series of examples that tell you how to get a development env running. The recommended IDE(Integrated
Development Environment) is [PyCharm](https://www.jetbrains.com/pycharm/). You can get a community version for research
purpose at [PyCharm Community Version](https://www.jetbrains.com/pycharm/download/). Other platform like Visual Studio
Code might also be helpful, although I never tested them.



Clone this repository to your local disk and open the folder as a project in your PyCharm IDE, you will see all packages
in the Project View. Their names and usage are as follows.



```
basis: Basic helper functions for math computation, data structure conversion, and trimesh processing.
grasping: Grasp planners.
manipulation: Stability Analyzers; Placement planners.
planning: Trajectory-level and probabilistic motion-level planners.
robotsim: Robot classes are defined in this package.
vision: Utility functiosn for processing 2D and 3D vision data.
visualization: Graphics. Panda3D is the main graphics engine.
```
### A Quick-Start: Demo in Simulation

Besides the abovementioned packages, there are some folders that host several examples. Run the following one
to examine if your the prerequisites and key packages work correctly.

0000_example folder

```
bullet_test1.py and bullet_test2.py show how to use bullet, which is a dynamical engine
rtq85_define_grasps.py show how to use grasping function
test_handover.py show how to use handover function to generate pose
ur3e_dual_dynamics.py show a dynamical model of ur3e_dual

```

0000_book folder
```
this series of grasping_xxxx.py :show you how to use grasping planning  
this series of math_xxxx.py : show  how to use robot_math function
motion_rrtconnect.py : show how to use motion planning function 
this series of wrssystem_xxxx: show how to use collision_model and how to use jlchain

```
app folder

```
in app->demo
roller_hole_docking.py :show a whole process of ur3e_dual manipulation and how to use digital twin

```

### Running on a Real Robot (UR3e)
Creat your own code in the app folder and test in simulation. To run your code in a real UR3e dual-arm robot, please drop by my office and talk to me. 
