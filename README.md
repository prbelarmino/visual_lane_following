# visual_lane_following

## Info regarding the external packages:

- perception: it affords perception topic such as image from both cameras, imu(realsense camera), laserscan and encoder.
- locomotion: it provides the topic drive_command that sends commands for the vehcile actuators: drive: driver_code:1, steering_angle: +-1.0, steering_angle_velocity: 2.0, speed: +-1.0, acceleration:+-0.69999. 
- teleoperation: receives the commands from the joystick.
- diagnostics.
