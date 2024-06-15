# search-in-p-collision-resolution
An implementation of a prototypal collision resolution method. It is inspired by Casey Muratori's search-in-p collision resolution method that he used in the Video Game "The Witness".
See his blog post here: https://caseymuratori.com/blog_0032. I recommend you watch his explanation first before dealing with this code.

# Setting
The code deals with a very specific collision resolution problem.
The method implemented here tries to resolve collisions of a cylinder against a triangle mesh.
The cylinder is given an additional _leg_height_ and _head_height_ parameters that are typically
used in video games to provide stair clipping.

The implementation only attempts to find a new position where to place the cylinder.
If no such position exists that is at most the cylinder's radius away from the target (in x,y), then
no new position will be found.

# Goal

The goal of my implementation was to achieve 100µs/call for 16 triangles on a 3GHz CPU using AVX-2.
The benchmark run in the usage example show running times of ~7µs/call.

# DISCLAIMER

This implementation is _not_ meant to be _complete_ and _bug-free_. Instead, it is meant to highlight the
performance potentials of search-in-p collision resolution for a very specific setting. Both the method and
the implementation have known issues. Even though it is mostly workable, I do not recommend use in
a commercial game, but only for experimentation.

This implementation is _not_ a re-implementation of Casey Muratori's method, but inspired by it.

# Usage

Compilation will (probably) only work on GCC, as I did not bother making it portable. The code
makes heavy use of AVX-2. Compile from the project's directory using

```
g++-12 -mavx2 -s -flto -O3 -ftabstop=4 -Wno-maybe-uninitialized -Wno-uninitialized -o usage_example.out usage_example.cpp
```

The example runs the collision resolution method for an example and prints the new cylinder position, then
runs a benchmark.
