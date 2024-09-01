/**
 * @file tbot_get_goals.h
 * @brief  Header file for the TurtleGoals class.
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

using namespace std::chrono_literals;

namespace turtle{
/**
 * @brief TurtleGoals class for managing turtle goals based on Aruco markers.
*/
class TurtleGoals: public rclcpp::Node{
    public:
    /**
     * @brief Constructor for TurtleGoals class.
     */
    TurtleGoals(std::string node_name):Node(node_name){

        //<! Declaring parameters
        this->declare_parameter("aruco_0.wp1.type","battery");
        this->declare_parameter("aruco_0.wp1.color","green");
        this->declare_parameter("aruco_0.wp2.type","battery");
        this->declare_parameter("aruco_0.wp2.color","red");
        this->declare_parameter("aruco_0.wp3.type","battery");
        this->declare_parameter("aruco_0.wp3.color","orange");
        this->declare_parameter("aruco_0.wp4.type","battery");
        this->declare_parameter("aruco_0.wp4.color","purple");
        this->declare_parameter("aruco_0.wp5.type","battery");
        this->declare_parameter("aruco_0.wp5.color","blue"); 

        this->declare_parameter("aruco_1.wp1.type","battery");
        this->declare_parameter("aruco_1.wp1.color","blue");
        this->declare_parameter("aruco_1.wp2.type","battery");
        this->declare_parameter("aruco_1.wp2.color","green");
        this->declare_parameter("aruco_1.wp3.type","battery");
        this->declare_parameter("aruco_1.wp3.color","orange");
        this->declare_parameter("aruco_1.wp4.type","battery");
        this->declare_parameter("aruco_1.wp4.color","red");
        this->declare_parameter("aruco_1.wp5.type","battery");
        this->declare_parameter("aruco_1.wp5.color","purple");   

        //<! Geting the parameters and storing it in attributes
        aruco_zero_wp1_color_=this->get_parameter("aruco_0.wp1.color").as_string();
        aruco_zero_wp2_color_=this->get_parameter("aruco_0.wp2.color").as_string();
        aruco_zero_wp3_color_=this->get_parameter("aruco_0.wp3.color").as_string();
        aruco_zero_wp4_color_=this->get_parameter("aruco_0.wp4.color").as_string();
        aruco_zero_wp5_color_=this->get_parameter("aruco_0.wp5.color").as_string();

        aruco_one_wp1_color_=this->get_parameter("aruco_1.wp1.color").as_string();
        aruco_one_wp2_color_=this->get_parameter("aruco_1.wp2.color").as_string();
        aruco_one_wp3_color_=this->get_parameter("aruco_1.wp3.color").as_string();
        aruco_one_wp4_color_=this->get_parameter("aruco_1.wp4.color").as_string();
        aruco_one_wp5_color_=this->get_parameter("aruco_1.wp5.color").as_string();


        // Subsciption to Aurco marker
        aruco_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers",rclcpp::SensorDataQoS(),
        std::bind(&TurtleGoals::aruco_sub_cb , this , std::placeholders::_1));
        
        // Creating a unique pointer for the tf2_ros::Buffer associated with the blue battery
        blue_battery_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // Creating a shared pointer for the tf2_ros::TransformListener using the blue_battery_tf_buffer
        blue_battery_transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*blue_battery_tf_buffer_);

        // Creating a unique pointer for the tf2_ros::Buffer associated with the red battery
        red_battery_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // Creating a shared pointer for the tf2_ros::TransformListener using the red_battery_tf_buffer
        red_battery_transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*red_battery_tf_buffer_);

        // Creating a unique pointer for the tf2_ros::Buffer associated with the green battery
        green_battery_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // Creating a shared pointer for the tf2_ros::TransformListener using the green_battery_tf_buffer
        green_battery_transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*green_battery_tf_buffer_);

        // Creating a unique pointer for the tf2_ros::Buffer associated with the orange battery
        orange_battery_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // Creating a shared pointer for the tf2_ros::TransformListener using the orange_battery_tf_buffer
        orange_battery_transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*orange_battery_tf_buffer_);

        // Creating a unique pointer for the tf2_ros::Buffer associated with the purple battery
        purple_battery_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // Creating a shared pointer for the tf2_ros::TransformListener using the purple_battery_tf_buffer
        purple_battery_transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*purple_battery_tf_buffer_);

        // Publisher for the Goals topic with a queue size of 10
        goals_publisher_=this->create_publisher<geometry_msgs::msg::PoseArray>("Goals",10);

        //Timer Callback
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&TurtleGoals::timer_cb, this));
    }



    private:

    // ==================== methods ====================
    /**
     * @brief Callback function for processing ArucoMarkers messages.
     * @param msg The received ArucoMarkers message.
    */
    void aruco_sub_cb(ros2_aruco_interfaces::msg::ArucoMarkers msg);

    /**
     * @brief Listens for the transform of the blue battery marker.
     * @param source_frame The source frame of the transform.
     * @param target_frame The target frame of the transform.
    */
    void blue_battery_listen_transform(const std::string &source_frame, const std::string &target_frame);
    /**
     * @brief Listens for the transform of the red battery marker.
     * @param source_frame The source frame of the transform.
     * @param target_frame The target frame of the transform.
    */
    void red_battery_listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief Listens for the transform of the green battery marker.
     * @param source_frame The source frame of the transform.
     * @param target_frame The target frame of the transform.
     */
    void green_battery_listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief Listens for the transform of the orange battery marker.
     * @param source_frame The source frame of the transform.
     * @param target_frame The target frame of the transform.
    */
    
    void orange_battery_listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief Listens for the transform of the purple battery marker.
     * @param source_frame The source frame of the transform.
     * @param target_frame The target frame of the transform.
     */
    void purple_battery_listen_transform(const std::string &source_frame, const std::string &target_frame);

    
    // Callback function for timer
    void timer_cb();

    // ==================== attributes ====================
    
    // Store the detected aruco marker type 
    int aruco_id_;

    
    // Variable to store waypoint color for aurco marker id  
    std::string aruco_zero_wp1_color_;
    std::string aruco_zero_wp2_color_;
    std::string aruco_zero_wp3_color_;
    std::string aruco_zero_wp4_color_;
    std::string aruco_zero_wp5_color_;

    std::string aruco_one_wp1_color_;
    std::string aruco_one_wp2_color_;
    std::string aruco_one_wp3_color_;
    std::string aruco_one_wp4_color_;
    std::string aruco_one_wp5_color_;
    
    // Subscription for ArucoMarkers messages
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscription_;

    // TF2 Buffers and TransformListeners for different battery markers
    std::unique_ptr<tf2_ros::Buffer> blue_battery_tf_buffer_;
    std::unique_ptr<tf2_ros::Buffer> red_battery_tf_buffer_;
    std::unique_ptr<tf2_ros::Buffer> green_battery_tf_buffer_;
    std::unique_ptr<tf2_ros::Buffer> orange_battery_tf_buffer_;
    std::unique_ptr<tf2_ros::Buffer> purple_battery_tf_buffer_;

    // Transform listener object for kistening to aruco marker transform
    std::shared_ptr<tf2_ros::TransformListener> blue_battery_transform_listener_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> red_battery_transform_listener_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> green_battery_transform_listener_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> orange_battery_transform_listener_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> purple_battery_transform_listener_{nullptr};
    
    // Attributes For Storing battery positions    
    float blue_battery_x_;
    float blue_battery_y_;
    float red_battery_x_;
    float red_battery_y_;
    float green_battery_x_;
    float green_battery_y_;
    float orange_battery_x_;
    float orange_battery_y_;
    float purple_battery_x_;
    float purple_battery_y_;

    
    // Storing different waypoints in the map   
    float wp1x_;
    float wp1y_;    
    float wp2x_;
    float wp2y_;
    float wp3x_;    
    float wp3y_;
    float wp4x_;
    float wp4y_;
    float wp5x_;
    float wp5y_;

    
    
    // Vector to store waypoint colors   
    std::vector<std::string> wp_colors;

    
    // Publisher for the Goals topic   
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goals_publisher_;

    
    // PoseArray to store goal poses    
    geometry_msgs::msg::PoseArray goals_pose_;

    
    // Poses of different waypoints    
    geometry_msgs::msg::Pose wp1_;
    geometry_msgs::msg::Pose wp2_;
    geometry_msgs::msg::Pose wp3_;
    geometry_msgs::msg::Pose wp4_;
    geometry_msgs::msg::Pose wp5_;
    
    
    // Timer    
    rclcpp::TimerBase::SharedPtr timer_;

}; // class TurtleGoals
}// namespace turtle
