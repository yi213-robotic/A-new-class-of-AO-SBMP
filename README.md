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

- uses our own optimized data structures;
- improves computational efficiency and code organization;
- is intended to provide a clean reference implementation of BLIT\*.

A separate implementation designed for integration into the **OMPL core** will be released later.
