# RigidBodyDynamics.jl

[![Build Status](https://travis-ci.org/tkoolen/RigidBodyDynamics.jl.svg?branch=master)](https://travis-ci.org/tkoolen/RigidBodyDynamics.jl)
[![codecov.io](https://codecov.io/github/tkoolen/RigidBodyDynamics.jl/coverage.svg?branch=master)](https://codecov.io/github/tkoolen/RigidBodyDynamics.jl?branch=master)

RigidBodyDynamics.jl is a small rigid body dynamics library for Julia. It was inspired by the [IHMCRoboticsToolkit](https://bitbucket.org/ihmcrobotics/ihmc-open-robotics-software) from the Institute of Human and Machine Cognition, and by [Drake](http://drake.mit.edu).

## Key features
* easy creation of general rigid body mechanisms
* extensive checks that verify that coordinate systems match before computation: the goal is to make reference frame mistakes impossible
* support for automatic differentiation using e.g. [ForwardDiff.jl](https://github.com/JuliaDiff/ForwardDiff.jl)
* flexible caching of intermediate results to prevent doing double work
* fairly small codebase and few dependencies

## Current functionality
* kinematics/transforming points and free vectors from one coordinate system to another
* transforming wrenches, momenta (spatial force vectors) and twists and their derivatives (spatial motion vectors) from one coordinate system to another
* relative twists/spatial accelerations between bodies
* kinetic/potential energy
* center of mass
* geometric/basic/spatial Jacobians
* momentum matrix
* mass matrix (composite rigid body algorithm)
* inverse dynamics (recursive Newton-Euler)
* dynamics

The (forward) dynamics algorithm is currently rudimentary; it just computes the mass matrix and the bias term (using the inverse dynamics algorithm), and then solves for joint accelerations without exploiting sparsity.

Closed loop systems and contact are not yet supported.

## Related packages
* [RigidBodyTreeInspector.jl](https://github.com/rdeits/RigidBodyTreeInspector.jl) - 3D visualization of RigidBodyDynamics.jl `Mechanism`s using [Director](https://github.com/RobotLocomotion/director).

## Citing this library
```
@misc{rigidbodydynamicsjl,
 author = "Twan Koolen and contributors",
 title = "RigidBodyDynamics.jl",
 year = 2016,
 url = "https://github.com/tkoolen/RigidBodyDynamics.jl"
}
```
