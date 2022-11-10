# LEOBotV2
This repository contains the files for my thesis project. An autonomous mobile robot using vSLAM to explore an unknown area. 

1. Explores the area using a zig-zag, seed spreader algorithm. During this it records a rosbag which contains the telemetry data from the robot (vision sensor images, laser scans, position transformations)
2. Using the recorded rosbag it calculates the path the robot has done during the exploration, creates a binary occupancy map based on the laser scans, saves the positions of the AprilTags along the way.
3. After map creation is done using RRT it patrols between AprilTags.

Later it can be upgraded so that if an April Tag is missing in the patroling part it would raise an alert for the user.
