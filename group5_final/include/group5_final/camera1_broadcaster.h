/**
 * @file camera1_broadcaster.h
 * @brief Header File for Camera 1 Broadcaster 
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
     * @brief Class for Camera 1 Broadcaster
    */
    class CameraOneBroadcaster : public rclcpp::Node
    {
        public:
            /**
             * @brief Constructor For Camera 1 Broadcaster 
            */
            CameraOneBroadcaster(std::string node_name) : Node(node_name)
            {

                RCLCPP_INFO(this->get_logger(), "Broadcaster started for Camera1");

                // Subscription to Camera 1 image topic 
                camera1_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/camera1/image",rclcpp::SensorDataQoS(),
                std::bind(&CameraOneBroadcaster::camera1_sub_cb , this , std::placeholders::_1));

                // Create a timer for broadcasting Camera 1 transforms
                cameraone_broadcast_timer_=this->create_wall_timer(1s,std::bind(&CameraOneBroadcaster::camera1_broadcast_timer_cb,this));

                // Initialize StaticTransformBroadcaster for Camera1
                tf_broadcaster_camera1_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

                // Initialize Buffer for Camera 1 transforms using dedicated thread
                tf_buffer_camera1_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_buffer_camera1_->setUsingDedicatedThread(true);
            }


        private:

            /**
             * @brief Callback function for Camera 1 Subscription 
             * @param msg The received Camera 1 message.
            */
            void camera1_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg);
       
            /**
             * @brief Timer Callback for Broadcasting Camera 1 Transformation
            */
            void camera1_broadcast_timer_cb();

            
            // Timer for Camera 1 Transform
            rclcpp::TimerBase::SharedPtr cameraone_broadcast_timer_;

            
            // Subscription for Camera 1 Image
            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera1_subscription_;

            
            // Buffer for Camera 1 Transform           
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_camera1_;

            
            // Static Transform Broadcaster for Camera 1            
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_camera1_;
           
            //  X- Coordinate for Camera 1 transform
            float cameraone_x_; 
           
            //  Y- Coordinate for Camera 1 transform
            float cameraone_y_; 
         
            //  Z- Coordinate for Camera 1 transform
            float cameraone_z_;  
        
            // X component of Quaternion of Camera 1 transform
            float cameraone_q_x_;   

            // Y component of Quaternion of Camera 1 transform
            float cameraone_q_y_; 

            // Z component of Quaternion of Camera 1 transform
            float cameraone_q_z_;  

            // W component of Quaternion of Camera 1 transform
            float cameraone_q_w_; 

            // Variable to store Battery Color 
            std::string battery_color_;

            // Variable to store the Color ID 
            int color_id_;


    };
}