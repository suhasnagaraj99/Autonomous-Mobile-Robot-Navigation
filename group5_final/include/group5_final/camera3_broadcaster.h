/**
 * @file camera3_broadcaster.h
 * @brief Header File for camera 3 Broadcaster 
*/
#pragma once
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>


using namespace std::chrono_literals;


namespace turtle{

    /**
     * @brief Class for camera 3 Broadcaster
    */
    class CameraThreeBroadcaster : public rclcpp::Node
    {
        public:
            /**
             * @brief Constructor For camera 3 Broadcaster 
            */
            CameraThreeBroadcaster(std::string node_name) : Node(node_name)
            {

                RCLCPP_INFO(this->get_logger(), "Broadcaster started for camera3");

                // Subscription to camera 3 image topic 
                camera3_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera3/image",rclcpp::SensorDataQoS(),
                std::bind(&CameraThreeBroadcaster::camera3_sub_cb , this , std::placeholders::_1));

                // Create a timer for broadcasting camera 3 transforms
                camerathree_broadcast_timer_=this->create_wall_timer(1s,std::bind(&CameraThreeBroadcaster::camera3_broadcast_timer_cb,this));

                // Initialize StaticTransformBroadcaster for camera3
                tf_broadcaster_camera3_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

                // Initialize Buffer for camera 3 transforms using dedicated thread
                tf_buffer_camera3_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_buffer_camera3_->setUsingDedicatedThread(true);
            }


        private:

            
            /**
             * @brief Callback function for Camera 3 Subscription 
             * @param msg The received Camera 3 message.
            */
            void camera3_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg);

            
            /**
             * @brief Timer Callback for Broadcasting Camera 3 Transformation
            */
            void camera3_broadcast_timer_cb();

            
            // Timer for camera 3 Transform
            rclcpp::TimerBase::SharedPtr camerathree_broadcast_timer_;

            
            // Subscription for camera 3 Image
            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera3_subscription_;

            
            // Buffer for camera 3 Transform
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_camera3_;

            
            // Static Transform Broadcaster for camera 3 
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_camera3_;

            
            //  X- Coordinate for camera 3 transform
            float camerathree_x_; 

            
            //  Y- Coordinate for camera 3 transform
            float camerathree_y_; 

            
            //  Z- Coordinate for camera 3 transform
            float camerathree_z_;  
        
            // X component of Quaternion of camera 3 transform
            float camerathree_q_x_;   

            // Y component of Quaternion of camera 3 transform
            float camerathree_q_y_; 

            // Z component of Quaternion of camera 3 transform
            float camerathree_q_z_;  

            // W component of Quaternion of camera 3 transform
            float camerathree_q_w_; 

            // Variable to store Battery Color 
            std::string battery_color_;

            // Variable to store the Color ID 
            int color_id_;


    };
}