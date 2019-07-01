# Boustrophedon Planner
Boustrophedon Planner is a coverage path planner that implements the [Boustrophedon Cell Decomposition](https://en.wikipedia.org/wiki/Boustrophedon_cell_decomposition) algorithm.

## Overview
The path planner is an actionlib server that takes in a `geometry_msgs/PolygonStamped` and a `geometry_msgs/PoseStamped`,
and returns a `StripingPlan` message which contains a list of waypoints to stripe the passed in polygon.
