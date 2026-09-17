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

**Author:** Yi Wang

This repository provides C++ implementations of steering functions, including cost and time computations, for the following systems:

- 4D double-integrator model
- 10D linearized quadrotor model
- 3D Dubins model

### Acknowledgments

**Double-Integrator and Quadrotor Models**

These implementations are based on the original MATLAB code developed by Dr. Dongliang Chang:

[Accelerating Kinodynamic RRT* Through Dimensionality Reduction](https://github.com/dongliangCH/Accelerating-Kinodynamic-RRTstar-Through-Dimensionality-Reduction)

**3D Dubins Model**

This implementation is based on Dubins3D.jl from the Computational Robotics Laboratory:

[Dubins3D.jl](https://github.com/comrob/Dubins3D.jl)

The C++ translations and subsequent modifications or extensions were independently developed by Yi Wang.

### Usage and Citation

If you use or modify these implementations, please appropriately acknowledge the original authors and cite the relevant publications.
