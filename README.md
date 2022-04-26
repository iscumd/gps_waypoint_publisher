# gps_waypoint_publisher
From package '[gps_waypoint_publisher](https://youtu.be/oHg5SJYRHA0)'
# File
`./gps_waypoint_publisher/gwp.py`

## Summary 
 A node that converts a file of gps points into waypoints about map, then follows them using nav2.

This node requires the use of a magnometer and GPS.

In practice, this node will wait for an initalpoint to be published. When it is, it will wait for a pose message, where that pose
is ECEF coordinates, and its orientation is a rotation from north. This is designed to match the API of the Vectornav driver.
While this is happening, the robot should not move. Once these have been obtained, it will parse the gps points and convert them into local waypoints. Finally, it will
publish these poses for consumption by a node like waypoint_publisher.

## Topics

### Publishes
- `/gps/points`: The list of converted waypoints.

### Subscribes
- `/pose`: A pose containing the orientation relative to north, and x y z ECEF gps coordinates.
- `/initalpose`: Standard initalpose. Assumed to be origin of map.

## Params
- `filepath **(required)**`: Path to file of GPS points. This file should be `lat,long` pairs, seperated by newlines.

# Misc 
 To obtain non ROS dependancies, you must run pip install -r requirements.txt in this directory. 
