# Idei implementare

Python, with uv for deps management.

Pathfinding algorithm: A*
Config module, with frozen read only dataclasses

Required packages:
- image: generated the field & processes to a numpy array

Dependencies:
- numpy
- pillow

## Part 1

Generate the image.

## Part 2

Process the image, extract red channel and normalize.

## Part 3

Convert to meters

## Part 4
https://www.geeksforgeeks.org/dsa/a-search-algorithm/
Basic A*

- use priority queue ordered by f = g + h
- g = traveled distance in meters, h = heuristic estimate to goal
- check is unblocked by using slope
- pop node with lowest f, skip if visited
- if node == goal → reconstruct path via parent map
- for each neighbor: check inside grid + slope ≤ max
- step cost = 0.1m or sqrt(2)m depending on move
- update g, f, parent if better path found
- return path and blocked cells list