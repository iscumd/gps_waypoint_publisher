# gps_waypoint_publisher
From package '[gps_waypoint_publisher](https://youtu.be/oHg5SJYRHA0)'
# File
`./gps_waypoint_publisher/gwp.py`

## Summary 
 A node that converts a file of gps points into waypoints about map, then follows them using nav2.

This node requires the use of a magnometer and GPS.

In practice, this node will wait for an initalpoint to be published. When it is, it will then wait for a
new GPS and magnometer reading to be recived. While this is happening, the robot should not move. Once these
have been obtained, it will parse the gps points and convert them into local waypoints. Finally, it will begin 
to navigate through these points using nav2's waypoint_follower.

## Topics

### Subscribes
- `/gps`: The source of gps data, used to find the gps point at the initalpose.
- `/pose`: A pose containing the orientation relative to north.
- `/initalpose`: Standard initalpose. Assumed to be origin of map.

## Params
- `filepath **(required)**`: Path to file of GPS points. This file should be `lat,long` pairs, seperated by newlines.

# Misc 
 To obtain non ROS dependancies, you must run pip install -r requirements.txt in this directory. 
