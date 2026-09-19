# A New Class of Asymptotically Optimal Sampling-Based Motion Planners

A new class of **asymptotically optimal sampling-based motion planners** using **lazy bidirectional heuristic search** will be released soon.

## Overview

The framework currently includes two asymptotically optimal planners:

- **Bidirectional Lazy Informed Trees (BLIT\*)**
- **A second planner**, which will be released soon

Additional implementations, documentation, and experimental results will be added as the project develops.

## BLIT\* in This Repository

The BLIT\* implementation provided in this repository is an **optimized standalone version** of the planner.

Compared with the previous GitHub implementation and the OMPL core implementation, this version:

- uses our own optimized data structures, especially for Nearest Neighbor queries and child-detachment operations;
- improves computational efficiency and code organization;
- is intended to provide a clean reference implementation of BLIT\*.

A separate implementation designed for integration into the **OMPL core** will be released later.



## Implementation and Attribution

The C++ implementations of steering functions, including cost and time computations, for the following dynamical systems were independently developed by Yi Wang through translation, adaptation, and modification for the experimental evaluation and benchmarking in our IJRR manuscript:

- 4D double-integrator model
- 10D linearized quadrotor model
- 3D Dubins model

These implementations are compatible with the Open Motion Planning Library (OMPL) and can be used for testing and benchmarking motion planning algorithms.

### Acknowledgments

**Double-Integrator and Quadrotor Models**

The C++ implementations are based on the original MATLAB code developed by Dr. Dongliang Chang:

[Accelerating Kinodynamic RRT* Through Dimensionality Reduction](https://github.com/dongliangCH/Accelerating-Kinodynamic-RRTstar-Through-Dimensionality-Reduction)

**3D Dubins Model**

The C++ implementation is based on Dubins3D.jl from the Computational Robotics Laboratory:

[Dubins3D.jl](https://github.com/comrob/Dubins3D.jl)

We gratefully acknowledge the original authors and developers for making their work publicly available.

### Usage and Citation

If you use or modify these implementations, please acknowledge the original authors and the C++ implementation work, and cite the relevant publications.
