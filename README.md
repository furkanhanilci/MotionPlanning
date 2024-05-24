## Overview

------
    .
    └── Search-based Planning
        ├── Breadth-First Searching (BFS)
        ├── Depth-First Searching (DFS)
        ├── Best-First Searching
        ├── Dijkstra's
        ├── A*
        ├── Bidirectional A*
        ├── Anytime Repairing A*
        ├── Learning Real-time A* (LRTA*)
        ├── Real-time Adaptive A* (RTAA*)
        ├── Lifelong Planning A* (LPA*)
        ├── Dynamic A* (D*)
        ├── D* Lite
        └── Anytime D*
    └── Sampling-based Planning
        ├── RRT
        ├── RRT-Connect
        ├── Extended-RRT
        ├── Dynamic-RRT
        ├── RRT*
        ├── Informed RRT*
        ├── RRT* Smart
        ├── Anytime RRT*
        ├── Closed-Loop RRT*
        ├── Spline-RRT*
        ├── Fast Marching Trees (FMT*)
        └── Batch Informed Trees (BIT*)
    └── Papers


This repository implemented some common motion planners used on autonomous vehicles, including
* [Hybrid A* Planner](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)
* [Frenet Optimal Trajectory](https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame)
* [Hierarchical Optimization-Based Collision Avoidance (H-OBCA)](https://ieeexplore.ieee.org/document/9062306) (Incomplete)

Also, this repository provides some controllers for path tracking, including
* [Pure Pursuit + PID](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
* [Rear-Wheel Feedback + PID](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)
* [Front-Wheel Feedback / Stanley + PID](http://robots.stanford.edu/papers/thrun.stanley05.pdf)
* [LQR + PID](https://github.com/ApolloAuto/apollo/tree/master/modules/control/controller)
* [Linear MPC](https://borrelli.me.berkeley.edu/pdfpub/pub-6.pdf)

## Requirement
* Python 3.6 or above
* [SciPy](https://www.scipy.org/)
* [cvxpy](https://github.com/cvxgrp/cvxpy)
* [Reeds-Shepp Curves](https://github.com/zhm-real/ReedsSheppCurves)
* [pycubicspline](https://github.com/AtsushiSakai/pycubicspline)

## Vehicle models
This repository uses two models: simple car model and [car pulling trailers model](http://planning.cs.uiuc.edu/node661.html#77556).

## Hybrid A* Planner
<div align=right>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/HybridAstarPlanner/gif/hybrid%20Astar-1.gif" alt="1" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/HybridAstarPlanner/gif/hybrid%20Astar-2.gif" alt="2" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/HybridAstarPlanner/gif/hybrid%20Astar-t1.gif" alt="11" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/HybridAstarPlanner/gif/hybrid%20Astar-t5.gif" alt="12" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/HybridAstarPlanner/gif/hybrid%20Astar-t3.gif" alt="13" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/HybridAstarPlanner/gif/hybrid%20Astar-t2.gif" alt="14" width="400"/></a></td>
  </tr>
</table>
</div>

## State Lattice Planner
<div align=right>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/LatticePlanner/gif/Crusing.gif" alt="1" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/LatticePlanner/gif/StoppingMode.gif" alt="2" width="400"/></a></td>
  </tr>
</table>
</div>

## Controllers
<div align=right>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/Control/gif/purepursuit1.gif" alt="1" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/Control/gif/RWF1.gif" alt="2" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/Control/gif/stanley.gif" alt="1" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/Control/gif/MPC.gif" alt="2" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/Control/gif/LQR_Kinematic.gif" alt="1" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/MotionPlanning/blob/master/Control/gif/LQR_Dynamics.gif" alt="2" width="400"/></a></td>
  </tr>
</table>
</div>

## Animations - Search-Based
### Best-First & Dijkstra
<div align=right>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/BF.gif" alt="dfs" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/Dijkstra.gif" alt="dijkstra" width="400"/></a></td>
  </tr>
</table>
</div>

### A* and A* Variants
<div align=right>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/Astar.gif" alt="astar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/Bi-Astar.gif" alt="biastar" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/RepeatedA_star.gif" alt="repeatedastar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/ARA_star.gif" alt="arastar" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/LRTA_star.gif" alt="lrtastar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/RTAA_star.gif" alt="rtaastar" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/D_star.gif" alt="lpastar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/LPAstar.gif" alt="dstarlite" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/ADstar_small.gif" alt="lpastar" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/path-planning-algorithms/blob/master/Search_based_Planning/gif/ADstar_sig.gif" alt="dstarlite" width="400"/></a></td>
  </tr>
</table>
</div>

## Animation - Sampling-Based
### RRT & Variants
<div align=right>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/PathPlanning/blob/master/Sampling_based_Planning/gif/RRT_2D.gif" alt="value iteration" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/PathPlanning/blob/master/Sampling_based_Planning/gif/Goal_biasd_RRT_2D.gif" alt="value iteration" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/PathPlanning/blob/master/Sampling_based_Planning/gif/RRT_CONNECT_2D.gif" alt="value iteration" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/PathPlanning/blob/master/Sampling_based_Planning/gif/Extended_RRT_2D.gif" alt="value iteration" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/PathPlanning/blob/master/Sampling_based_Planning/gif/Dynamic_RRT_2D.gif" alt="value iteration" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/PathPlanning/blob/master/Sampling_based_Planning/gif/RRT_STAR2_2D.gif" alt="value iteration" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/PathPlanning/blob/master/Sampling_based_Planning/gif/RRT_STAR_SMART_2D.gif" alt="value iteration" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/PathPlanning/blob/master/Sampling_based_Planning/gif/FMT.gif" alt="value iteration" width="400"/></a></td>
  </tr>
</table>
<table>
  <tr>
    <td><img src="https://github.com/zhm-real/PathPlanning/blob/master/Sampling_based_Planning/gif/INFORMED_RRT_STAR_2D3.gif" alt="value iteration" width="400"/></a></td>
    <td><img src="https://github.com/zhm-real/PathPlanning/blob/master/Sampling_based_Planning/gif/BIT2.gif" alt="value iteration" width="400"/></a></td>
  </tr>
</table>
</div>


## Paper
### Planning
* [Basic Path Planning Algorithms: ](https://github.com/zhm-real/PathPlanning) PathPlanning
* [Baidu Apollo Planning module: ](https://github.com/ApolloAuto/apollo/tree/master/modules/planning) Recommended Materials
* [Survey of Planning and Control algos: ](https://arxiv.org/pdf/1604.07446.pdf) A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles
* [Hybrid A* Planner: ](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf) Practical Search Techniques in Path Planning for Autonomous Driving
* [Frenet Optimal Trajectory: ](https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame) Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame

### Control
* [Baidu Apollo Control module: ](https://github.com/ApolloAuto/apollo/tree/master/modules/control) Recommended Materials
* [Pure Pursuit: ](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf) Implementation of the Pure Pursuit Path Tracking Algorithm 
* [Rear-Wheel Feedback: ](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf) Automatic Steering Methods for Autonomous Automobile Path Tracking
* [Front-Wheel Feedback / Stanley: ](http://robots.stanford.edu/papers/thrun.stanley05.pdf) Stanley: The Robot that Won the DARPA Grand Challenge
* [LQR: ](https://github.com/ApolloAuto/apollo/tree/master/modules/control/controller) ApolloAuto/apollo: An open autonomous driving platform
* [Linear MPC: ](https://borrelli.me.berkeley.edu/pdfpub/pub-6.pdf) MPC-Based Approach to Active Steering for Autonomous Vehicle Systems

## Useful Material
* [HybridAStarTrailer (Julia): ](https://github.com/AtsushiSakai/HybridAStarTrailer) by AtsushiSakai
* [Hybrid Path Planner (C++): ](https://github.com/karlkurzer/path_planner) by KTH Research Concept Vehicle

### Search-base Planning
* [A*: ](https://ieeexplore.ieee.org/document/4082128) A Formal Basis for the heuristic Determination of Minimum Cost Paths
* [Learning Real-Time A*: ](https://arxiv.org/pdf/1110.4076.pdf) Learning in Real-Time Search: A Unifying Framework
* [Real-Time Adaptive A*: ](http://idm-lab.org/bib/abstracts/papers/aamas06.pdf) Real-Time Adaptive A*
* [Lifelong Planning A*: ](https://www.cs.cmu.edu/~maxim/files/aij04.pdf) Lifelong Planning A*
* [Anytime Repairing A*: ](https://papers.nips.cc/paper/2382-ara-anytime-a-with-provable-bounds-on-sub-optimality.pdf) ARA*: Anytime A* with Provable Bounds on Sub-Optimality
* [D*: ](http://web.mit.edu/16.412j/www/html/papers/original_dstar_icra94.pdf) Optimal and Efficient Path Planning for Partially-Known Environments
* [D* Lite: ](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf) D* Lite
* [Field D*: ](http://robots.stanford.edu/isrr-papers/draft/stentz.pdf) Field D*: An Interpolation-based Path Planner and Replanner
* [Anytime D*: ](http://www.cs.cmu.edu/~ggordon/likhachev-etal.anytime-dstar.pdf) Anytime Dynamic A*: An Anytime, Replanning Algorithm
* [Focussed D*: ](http://robotics.caltech.edu/~jwb/courses/ME132/handouts/Dstar_ijcai95.pdf) The Focussed D* Algorithm for Real-Time Replanning
* [Potential Field, ](https://journals.sagepub.com/doi/abs/10.1177/027836498600500106) [[PPT]: ](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf) Real-Time Obstacle Avoidance for Manipulators and Mobile Robots
* [Hybrid A*: ](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf) Practical Search Techniques in Path Planning for Autonomous Driving

### Sampling-based Planning
* [RRT: ](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf) Rapidly-Exploring Random Trees: A New Tool for Path Planning
* [RRT-Connect: ](http://www-cgi.cs.cmu.edu/afs/cs/academic/class/15494-s12/readings/kuffner_icra2000.pdf) RRT-Connect: An Efficient Approach to Single-Query Path Planning
* [Extended-RRT: ](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.1.7617&rep=rep1&type=pdf) Real-Time Randomized Path Planning for Robot Navigation
* [Dynamic-RRT: ](https://www.ri.cmu.edu/pub_files/pub4/ferguson_david_2006_2/ferguson_david_2006_2.pdf) Replanning with RRTs
* [RRT*: ](https://journals.sagepub.com/doi/abs/10.1177/0278364911406761) Sampling-based algorithms for optimal motion planning
* [Anytime-RRT*: ](https://dspace.mit.edu/handle/1721.1/63170) Anytime Motion Planning using the RRT*
* [Closed-loop RRT* (CL-RRT*): ](http://acl.mit.edu/papers/KuwataTCST09.pdf) Real-time Motion Planning with Applications to Autonomous Urban Driving
* [Spline-RRT*: ](https://ieeexplore.ieee.org/abstract/document/6987895?casa_token=B9GUwVDbbncAAAAA:DWscGFLIa97ptgH7NpUQUL0A2ModiiBDBGklk1z7aDjI11Kyfzo8rpuFstdYcjOofJfCjR-mNw) Optimal path planning based on spline-RRT* for fixed-wing UAVs operating in three-dimensional environments
* [LQR-RRT*: ](https://lis.csail.mit.edu/pubs/perez-icra12.pdf) Optimal Sampling-Based Motion Planning with Automatically Derived Extension Heuristics
* [RRT#: ](http://dcsl.gatech.edu/papers/icra13.pdf) Use of Relaxation Methods in Sampling-Based Algorithms for Optimal Motion Planning
* [RRT*-Smart: ](http://save.seecs.nust.edu.pk/pubs/ICMA2012.pdf) Rapid convergence implementation of RRT* towards optimal solution
* [Informed RRT*: ](https://arxiv.org/abs/1404.2334) Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal heuristic
* [Fast Marching Trees (FMT*): ](https://arxiv.org/abs/1306.3532) a Fast Marching Sampling-Based Method for Optimal Motion Planning in Many Dimensions
* [Motion Planning using Lower Bounds (MPLB): ](https://ieeexplore.ieee.org/document/7139773) Asymptotically-optimal Motion Planning using lower bounds on cost
* [Batch Informed Trees (BIT*): ](https://arxiv.org/abs/1405.5848) Sampling-based Optimal Planning via the Heuristically Guided Search of Implicit Random Geometric Graphs
* [Advanced Batch Informed Trees (ABIT*): ](https://arxiv.org/abs/2002.06589) Sampling-Based Planning with Advanced Graph-Search Techniques ((ICRA) 2020)
* [Adaptively Informed Trees (AIT*): ](https://arxiv.org/abs/2002.06599) Fast Asymptotically Optimal Path Planning through Adaptive Heuristics ((ICRA) 2020)
