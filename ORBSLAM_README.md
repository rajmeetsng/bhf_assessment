Visual Odometry from ROSBAG file 
How to use
---------------------------------------------------------------------------------------------------------------------------------------------------------------------
For Docker: 

    Run Docker file using:
    
    # docker build -t my_orbslam3
    # docker run --name orb_slam3 -it my_orbslam3

    Run Command in Terminal 1: 
    # source intall/setup.bash
    # ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp

    Run Command in Terminal 2: 
    # source intall/setup.bash
    # ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=RealSense_D435i -p image_seq:=weed
--------------------------------------------------------------------------------------------------------------------------------------------------------------------
Without Docker 

   # mkdir -p colcon_ws/src
   # cd colcon_ws/src
   # git clone the repository 
   # cd colcon_ws
   # source /opt/ros/humble/setup.bash
   # colcon build --symlink-install
   
   Run Command in Terminal 1: 
    # source intall/setup.bash
    # ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp

    Run Command in Terminal 2: 
    # source intall/setup.bash
    # ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=RealSense_D435i -p image_seq:=weed
----------------------------------------------------------------------------------------------------------------------------------------------------------------------
