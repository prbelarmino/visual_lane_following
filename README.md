# visual_lane_following
This package executes lane detection and lane-following. Each one of these tasks is performed by a node. 
Below is a short description of each node:

- deviation_publisher: Subscribe on /ika_racer/perception/realsense/camera/color/image_raw topic and performs the lane detection. The lateral deviation and yaw angle of the center of the image relative to the center lane are calculated and published on /ika_racer/lane_visual_following/lane_deviation topic.
- lane_folloer: Subscribe on  /ika_racer/lane_visual_following/lane_deviation and /ika_racer/perception/encoder/speed_filtered topics and implement two controllers, one for the longitudinal speed and the other for the steering. The outputs of both controllers are published on /ika_racer/locomotion/drive_command, DriveCommandStamped topic.

Besides that, lane_detection_classical_v4.py file affords the functions used for lane detection. A detailed explanation of the structure of the algorithm is available in the Jupyter Notebook.
