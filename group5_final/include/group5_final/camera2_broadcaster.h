/**
 * @file camera2_broadcaster.h
 * @brief Header File for Camera 2 Broadcaster 
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
     * @brief Class for Camera 2 Broadcaster
    */
    class CameraTwoBroadcaster : public rclcpp::Node
    {
        public:
            /**
             * @brief Constructor For Camera 2 Broadcaster 
            */
            CameraTwoBroadcaster(std::string node_name) : Node(node_name)
            {

                RCLCPP_INFO(this->get_logger(), "Broadcaster started for camera2");

                // Subscription to Camera 2 image topic 
                camera2_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera2/image",rclcpp::SensorDataQoS(),
                std::bind(&CameraTwoBroadcaster::camera2_sub_cb , this , std::placeholders::_1));

                // Create a timer for broadcasting Camera 2 transforms
                cameratwo_broadcast_timer_=this->create_wall_timer(1s,std::bind(&CameraTwoBroadcaster::camera2_broadcast_timer_cb,this));

                // Initialize StaticTransformBroadcaster for camera2
                tf_broadcaster_camera2_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

                // Initialize Buffer for Camera 2 transforms using dedicated thread
                tf_buffer_camera2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_buffer_camera2_->setUsingDedicatedThread(true);
            }


        private:

            
            /**
             * @brief Callback function for Camera 2 Subscription 
             * @param msg The received Camera 2 message.
            */
            void camera2_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg);

            
            /**
             * @brief Timer Callback for Broadcasting Camera 2 Transformation
            */
            void camera2_broadcast_timer_cb();

            
            // Timer for Camera 2 Transform
            rclcpp::TimerBase::SharedPtr cameratwo_broadcast_timer_;
  
            // Subscription for Camera 2 Image
            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera2_subscription_;
          
            // Buffer for Camera 2 Transform
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_camera2_;

            
            // Static Transform Broadcaster for Camera 2 
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_camera2_;

            
            //  X- Coordinate for Camera 2 transform
            float cameratwo_x_; 

            
            //  Y- Coordinate for Camera 2 transform
            float cameratwo_y_; 

            
            //  Z- Coordinate for Camera 2 transform
            float cameratwo_z_;  
        
            // X component of Quaternion of Camera 2 transform
            float cameratwo_q_x_;   

            // Y component of Quaternion of Camera 2 transform
            float cameratwo_q_y_; 

            // Z component of Quaternion of Camera 2 transform
            float cameratwo_q_z_;  

            // W component of Quaternion of Camera 2 transform
            float cameratwo_q_w_; 

            // Variable to store Battery Color 
            std::string battery_color_;

            // Variable to store the Color ID 
            int color_id_;


    };
}