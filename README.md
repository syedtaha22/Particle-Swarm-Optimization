# Particle Swarm Optimization (PSO)

This repository implements a **Particle Swarm Optimization (PSO)** algorithm to find the maximum value of the function:

\[ f(x) = \sin(x) - \sin(3x) \]

for \( x \) in the range \([-4, 4]\).

## Overview
The PSO algorithm is a population-based optimization technique inspired by the collective behavior of birds and fish schools. It is used to find optimal solutions by having a swarm of particles explore the search space while dynamically adjusting their positions based on both their personal experiences and the experiences of the swarm as a whole.

### Mathematical Formulation
Each particle in the swarm has:
- **\( x \):**  The current postion in the search space, representing a potential solution.
- **\( v \)**: The velocity of the particle, determining how quickly it moves in the search space.
- **\( p \)**: The best position found by the particle so far.
- **\( g \)**: The best position found by the entire swarm.

The velocity of a particle is updated using the formula:

\[
v(t+1) = w v(t) + c_1 r_1 (p - x) + c_2 r_2 (g - x)
\]

where:
- \( w \) is the inertia weight, controlling how much the particle retains its previous velocity.
- \( c_1 \) is the cognitive coefficient, representing the influence of the particle’s own experience.
- \( c_2 \) is the social coefficient, representing the influence of the global best solution.
- \( r_1, r_2 \) are random numbers in the range \( [0,1] \) to introduce stochasticity.

The new position of the particle is then updated as:
\[
x(t+1) = x(t) + v(t+1)
\]

This iterative process continues until a stopping criterion is met, such as a maximum number of iterations or a sufficiently small improvement in the best solution.

## Features
- Swarm-based optimization approach
- Adjustable parameters:
  - Inertia weight \( w \)
  - Cognitive weight \( c_1 \)
  - Social weight \( c_2 \)
- Graphical visualization of the swarm movement
- Uses an external **Physics Engine** for simulation

## Dependencies
This repository depends on the **[Physics Engine](https://github.com/syedtaha22/Physics-Engine)** for vector operations, random number generation, and graphical rendering. Ensure that the engine is available in the `Engine/` directory.

## Experimenting with Parameters
Modify the following values in `main.cpp` to observe different behaviors:
- `w`: Inertia weight (default: `0.99`)
- `c1`: Cognitive weight (default: `1`)
- `c2`: Social weight (default: `0`)

Try different settings, such as:
- `w = 1.5, c1 = 1, c2 = 1` → Overshooting behavior
- `w = 0.5, c1 = 0, c2 = 1` → Particles move towards the global best
- `w = 0.5, c1 = 1, c2 = 0` → Particles move based on their own best position

## Simulations
Sample simulation recordings can be found in the [`sims/`](sims/) directory.

**Recording Naming Convention:** Each recording is named in the format `w-c1-c2.mp4`, where `w`, `c1`, and `c2` represent the respective parameter values used in the simulation.


