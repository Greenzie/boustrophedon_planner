# Boustrophedon Planner
Boustrophedon Planner is a coverage path planner that implements a modified cellular decomposition algorithm. The original can be found [here](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6806237/).

## Overview
The path planner is an actionlib server that takes in a `geometry_msgs/PolygonStamped` and a `geometry_msgs/PoseStamped`,
and returns a `StripingPlan` message which contains a list of waypoints to stripe the passed in polygon.

![Sample Boustrophedon Plan](half-y-turn-concave.png)

### Instructions
run the server using:
```
roslaunch boustrophedon_server boustrophedon_server.launch
```

The server waits for an actionlib service call of type `boustrophedon_msgs/PlanMowingPathGoal ` containing a `geometry_msgs/PolygonStamped`
and a `geometry_msgs/PoseStamped`. It will then return the striping path in a `StripingPlan` message as the actionlib server result.

For testing, you can use the client node by running:
```
rosrun boustrophedon_server boustrophedon_planner_client
```

The client loads a hard-coded polygon. It then waits for a message over the `/initialpose` topic (can be sent using rviz `2D Pose Estimate`).
Once sent, the client will get the result, conect it to path and publish it to `result_path`.
The polygon is published to `input_polygon` for convenience.


## Changelog

- January 23, 2020:
  - The boustrophedon planner can now handle all types of simple polygons that it is given, both convex and concave.
  - Inner boundaries are supported, allowing the planner to create paths at a certain offset from the initial given boundary.
  - "Half - Y" turns are supported, allowing for some curvature at the start / end of paths to assist robots that are not      completely holonomic.
  - Many new launch file parameters allow the user to specify more precisely the behavior that they want the plan to have

## License

This repository is subject to GNU General Public License version 3 or later due to its dependencies.

The geometric operations rely on CGAL which is restricted by GNU General Public License version 3 or later.
