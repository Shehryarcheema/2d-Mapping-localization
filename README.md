# 2D Mapping & Localization for Mobile Robots

Occupancy-grid mapping and pose estimation for a mobile robot, built in
**ROS** on a TurtleBot3 in Gazebo and RViz. A laser scanner and wheel odometry
drive the mapping loop: each scan's beam endpoints are projected into the
global frame using the robot's odometry pose, and the corresponding grid
cells are marked occupied on a 2D map published as standard ROS topics.

Coursework for the MSc AI programme — De Montfort University (P2952028).
The implementation is framed as applied robotics engineering: a real ROS
pipeline with real sensor topics, documented end to end.

## Highlights

- **Occupancy-grid map** — 5 m × 5 m grid at 10 cm resolution (50 × 50
  cells), with per-cell occupancy values in [0, 1].
- **Odometry-driven pose tracking** — yaw recovered from the odometry
  quaternion on every `/odom` message.
- **Laser endpoint mapping** — each valid scan beam (within the sensor's
  range limits) is transformed from the robot's local frame to global
  coordinates and its endpoint cell marked occupied.
- **Standard ROS interface** — publishes latched `nav_msgs/OccupancyGrid`
  on `/map` and `nav_msgs/MapMetaData` on `/map_metadata`, viewable
  directly in RViz (occupancy scaled to the ROS 0–100 convention).
- **Documented with figures** — screenshots of the Gazebo world, the
  running TurtleBot3, and the evolving RViz map in the screenshot folders;
  the mapping and localization theory is written up in the two PDF reports.

## How it works (`mapper.py`)

1. **Pose update (`odom_callback`)** — reads position and orientation
   quaternion from `/odom`; converts to yaw with
   `tf.transformations.euler_from_quaternion`.
2. **Scan processing (`scan_callback`)** — for each beam in
   `/scan.ranges`, discards readings outside `[range_min, range_max]`,
   converts the beam to local (x, y), then applies the rotation +
   translation defined by the current pose to get global coordinates.
3. **Grid update (`Map.set_cell`)** — converts global coordinates to
   grid indices (respecting the map origin and resolution) and marks the
   endpoint cell occupied.
4. **Publishing (`publish_map`)** — converts the NumPy grid to a ROS
   `OccupancyGrid` message and publishes `/map` and `/map_metadata`.

## Repository structure

```
├── mapper.py                                  # ROS mapping node (the implementation)
├── P2952028 -SHEHRYAR (2D Map).pdf           # mapping coursework report
├── P2952028- SHEHRYAR (Localization Report).pdf  # localization coursework report
├── Screenshots (2D)/                        # TurtleBot Gazebo env, RViz grid map,
│                                            # overall running system
├── Screen Shots (2D Localization)/          # localization figures: Gazebo world,
│                                            # laser scan pose arrow, RViz views
└── README.md
```

## Getting started

Requires a ROS 1 environment with `rospy`, `nav_msgs`, `sensor_msgs`,
and `tf`, plus a robot or simulator publishing `/scan` (`LaserScan`) and
`/odom` (`Odometry`) — e.g. TurtleBot3 in Gazebo.

```bash
# In a sourced ROS workspace with the topics live:
python mapper.py          # starts the 'mapper' node (see __main__)
# In RViz, add a Map display subscribed to /map
```

The node subscribes to `/scan` and `/odom`, updates the grid on every
scan, and re-publishes the map; visualize progress live in RViz.

## Author

SHEHRYAR (P2952028) — MSc AI, De Montfort University.
