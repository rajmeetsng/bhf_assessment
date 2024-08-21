# Problem

We want to create a simple visual odometry (VO) system which can localize the camera motion in the enviornment. A sample ROS2 bag is provided at [this](https://drive.google.com/drive/folders/1woMbdKQgjI6zk9FuLtzs4iL2ymrjQ7AX?usp=sharing) link which has a sample scenario. To create the VO system, OpenCV can be used. 

1. Write a dockerfile, when launched can spin up the ROS Node (use ROS2 humble).
2. Create a ROS Node (in C++) which takes in the image frames and publishes the pose of the camera and the essential matrix estimated from visual odometry (VO). Create a custom message for this. This custom message should be published at 15 fps.
3. Make sure all the packages and libraries are installed and included in the Dockerfile and the ROS2 package.
4. Account for edge cases, for example if Images are not recieved or is late.

Fork the repository and create a pull request when you are ready to submit.

This purpose of this assessment is to see your coding process, so be sure to commit and push often, good luck!
