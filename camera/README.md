# D405 bringup
Uses the installed RealSense ROS 4.55.1 / librealsense 2.55.1 without reinstalling either.
Build from the repository with `colcon build --base-paths camera/ros2/d405_bringup` in an isolated workspace. Source its install/setup.bash, then run `ros2 launch d405_bringup d405.launch.py`.
Before launch confirm no existing D405 driver process. The wrapper starts only the camera, never robot bringup. RGB/depth 848x480x30, alignment and pointcloud disabled. Topics under `/d405/wrist`; depth units must be verified from the current driver settings, not inferred from 16UC1 alone.
