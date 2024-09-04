#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>


class CameraPosePublisher : public rclcpp::Node
{
public:
    CameraPosePublisher() : Node("camera_pose_publisher")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10, std::bind(&CameraPosePublisher::imageCallback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<std_msgs::msg::String>("/camera/pose", 10);
        essential_matrix_pub_ = this->create_publisher<std_msgs::msg::String>("/camera/essential_matrix", 10);
        
        

        //timer_ = this->create_wall_timer(
            //std::chrono::milliseconds(67), // 15 FPS -> 1000 ms / 15 = 67 ms
            //std::bind(&CameraPosePublisher::imageCallback, this));
        
        
        
        RCLCPP_INFO(this->get_logger(), "Camera Pose Publisher Initialized.");
      
     
        
    }
    
  
    
   
    

private:

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat current_frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if (previous_frame_.empty())
        {
            previous_frame_ = current_frame.clone();
            return;
        }

        // Feature detection using ORB
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        orb->detectAndCompute(previous_frame_, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(current_frame, cv::noArray(), keypoints2, descriptors2);

        // Matching features using BFMatcher
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Filtering good matches
        std::sort(matches.begin(), matches.end());
        const int num_good_matches = matches.size() * 0.1; // Keep the top 10% matches
        matches.erase(matches.begin() + num_good_matches, matches.end());

        // Extracting point coordinates from good matches
        std::vector<cv::Point2f> points1, points2;
        for (const auto &match : matches)
        {
            points1.push_back(keypoints1[match.queryIdx].pt);
            points2.push_back(keypoints2[match.trainIdx].pt);
        }

        if (points1.size() >= 8) // Minimum points required for fundamental matrix estimation
        {
            cv::Mat essential_matrix = cv::findEssentialMat(points1, points2, focal_length_, principal_point_, cv::RANSAC, 0.999, 1.0, inliers_);

            // Recover camera pose (rotation and translation) from the essential matrix
            cv::Mat R, t;
            cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length_, principal_point_, inliers_);

            // Convert rotation matrix and translation vector to pose string
            std::ostringstream pose_stream;
            pose_stream << "R: " << R << "\n"
                        << "t: " << t;

           
            // Publish the pose
            auto pose_msg = std_msgs::msg::String();
            pose_msg.data = pose_stream.str();
            pose_pub_->publish(pose_msg);

            // Publish the essential matrix
            auto essential_matrix_msg = std_msgs::msg::String();
            essential_matrix_msg.data = cv::format("%s", essential_matrix);
            essential_matrix_pub_->publish(essential_matrix_msg);
            
            
            
            

            
         
            
          
        }

        previous_frame_ = current_frame.clone();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr essential_matrix_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::Mat previous_frame_;
    double focal_length_ = 718.8560; // random focal length
    cv::Point2d principal_point_ = cv::Point2d(607.1928, 185.2157); // random principal point
    cv::Mat inliers_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPosePublisher>());
    rclcpp::shutdown();
    return 0;
}



































































































































