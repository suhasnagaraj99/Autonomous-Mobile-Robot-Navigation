#include "tbot_get_goals.h"

//============================================================================================//
//Function to determine the waypoints for navigation through maze 
void turtle::TurtleGoals::aruco_sub_cb(ros2_aruco_interfaces::msg::ArucoMarkers msg){

    aruco_id_=msg.marker_ids[0];

    blue_battery_listen_transform("map", "blue_battery_frame");
    red_battery_listen_transform("map", "red_battery_frame");
    green_battery_listen_transform("map", "green_battery_frame");
    orange_battery_listen_transform("map", "orange_battery_frame");
    purple_battery_listen_transform("map", "purple_battery_frame");

    if(aruco_id_==0){
        
        if(aruco_zero_wp1_color_== "blue"){
            wp1x_=blue_battery_x_;
            wp1y_=blue_battery_y_;
        }
        if(aruco_zero_wp1_color_== "red"){
            wp1x_=red_battery_x_;
            wp1y_=red_battery_y_;
        }
        if(aruco_zero_wp1_color_== "green"){
            wp1x_=green_battery_x_;
            wp1y_=green_battery_y_;
        }
        if(aruco_zero_wp1_color_== "orange"){
            wp1x_=orange_battery_x_;
            wp1y_=orange_battery_y_;
        }
        if(aruco_zero_wp1_color_== "purple"){
            wp1x_=purple_battery_x_;
            wp1y_=purple_battery_y_;
        }


        if(aruco_zero_wp2_color_== "blue"){
            wp2x_=blue_battery_x_;
            wp2y_=blue_battery_y_;
        }
        if(aruco_zero_wp2_color_== "red"){
            wp2x_=red_battery_x_;
            wp2y_=red_battery_y_;
        }
        if(aruco_zero_wp2_color_== "green"){
            wp2x_=green_battery_x_;
            wp2y_=green_battery_y_;
        }
        if(aruco_zero_wp2_color_== "orange"){
            wp2x_=orange_battery_x_;
            wp2y_=orange_battery_y_;
        }
        if(aruco_zero_wp2_color_== "purple"){
            wp2x_=purple_battery_x_;
            wp2y_=purple_battery_y_;
        }


        if(aruco_zero_wp3_color_== "blue"){
            wp3x_=blue_battery_x_;
            wp3y_=blue_battery_y_;
        }
        if(aruco_zero_wp3_color_== "red"){
            wp3x_=red_battery_x_;
            wp3y_=red_battery_y_;
        }
        if(aruco_zero_wp3_color_== "green"){
            wp3x_=green_battery_x_;
            wp3y_=green_battery_y_;
        }
        if(aruco_zero_wp3_color_== "orange"){
            wp3x_=orange_battery_x_;
            wp3y_=orange_battery_y_;
        }
        if(aruco_zero_wp3_color_== "purple"){
            wp3x_=purple_battery_x_;
            wp3y_=purple_battery_y_;
        }



        if(aruco_zero_wp4_color_== "blue"){
            wp4x_=blue_battery_x_;
            wp4y_=blue_battery_y_;
        }
        if(aruco_zero_wp4_color_== "red"){
            wp4x_=red_battery_x_;
            wp4y_=red_battery_y_;
        }
        if(aruco_zero_wp4_color_== "green"){
            wp4x_=green_battery_x_;
            wp4y_=green_battery_y_;
        }
        if(aruco_zero_wp4_color_== "orange"){
            wp4x_=orange_battery_x_;
            wp4y_=orange_battery_y_;
        }
        if(aruco_zero_wp4_color_== "purple"){
            wp4x_=purple_battery_x_;
            wp4y_=purple_battery_y_;
        }


        if(aruco_zero_wp5_color_== "blue"){
            wp5x_=blue_battery_x_;
            wp5y_=blue_battery_y_;
        }
        if(aruco_zero_wp5_color_== "red"){
            wp5x_=red_battery_x_;
            wp5y_=red_battery_y_;
        }
        if(aruco_zero_wp5_color_== "green"){
            wp5x_=green_battery_x_;
            wp5y_=green_battery_y_;
        }
        if(aruco_zero_wp5_color_== "orange"){
            wp5x_=orange_battery_x_;
            wp5y_=orange_battery_y_;
        }
        if(aruco_zero_wp5_color_== "purple"){
            wp5x_=purple_battery_x_;
            wp5y_=purple_battery_y_;
        }
    }

    if(aruco_id_==1){
        
        if(aruco_one_wp1_color_== "blue"){
            wp1x_=blue_battery_x_;
            wp1y_=blue_battery_y_;
        }
        if(aruco_one_wp1_color_== "red"){
            wp1x_=red_battery_x_;
            wp1y_=red_battery_y_;
        }
        if(aruco_one_wp1_color_== "green"){
            wp1x_=green_battery_x_;
            wp1y_=green_battery_y_;
        }
        if(aruco_one_wp1_color_== "orange"){
            wp1x_=orange_battery_x_;
            wp1y_=orange_battery_y_;
        }
        if(aruco_one_wp1_color_== "purple"){
            wp1x_=purple_battery_x_;
            wp1y_=purple_battery_y_;
        }


        if(aruco_one_wp2_color_== "blue"){
            wp2x_=blue_battery_x_;
            wp2y_=blue_battery_y_;
        }
        if(aruco_one_wp2_color_== "red"){
            wp2x_=red_battery_x_;
            wp2y_=red_battery_y_;
        }
        if(aruco_one_wp2_color_== "green"){
            wp2x_=green_battery_x_;
            wp2y_=green_battery_y_;
        }
        if(aruco_one_wp2_color_== "orange"){
            wp2x_=orange_battery_x_;
            wp2y_=orange_battery_y_;
        }
        if(aruco_one_wp2_color_== "purple"){
            wp2x_=purple_battery_x_;
            wp2y_=purple_battery_y_;
        }


        if(aruco_one_wp3_color_== "blue"){
            wp3x_=blue_battery_x_;
            wp3y_=blue_battery_y_;
        }
        if(aruco_one_wp3_color_== "red"){
            wp3x_=red_battery_x_;
            wp3y_=red_battery_y_;
        }
        if(aruco_one_wp3_color_== "green"){
            wp3x_=green_battery_x_;
            wp3y_=green_battery_y_;
        }
        if(aruco_one_wp3_color_== "orange"){
            wp3x_=orange_battery_x_;
            wp3y_=orange_battery_y_;
        }
        if(aruco_one_wp3_color_== "purple"){
            wp3x_=purple_battery_x_;
            wp3y_=purple_battery_y_;
        }



        if(aruco_one_wp4_color_== "blue"){
            wp4x_=blue_battery_x_;
            wp4y_=blue_battery_y_;
        }
        if(aruco_one_wp4_color_== "red"){
            wp4x_=red_battery_x_;
            wp4y_=red_battery_y_;
        }
        if(aruco_one_wp4_color_== "green"){
            wp4x_=green_battery_x_;
            wp4y_=green_battery_y_;
        }
        if(aruco_one_wp4_color_== "orange"){
            wp4x_=orange_battery_x_;
            wp4y_=orange_battery_y_;
        }
        if(aruco_one_wp4_color_== "purple"){
            wp4x_=purple_battery_x_;
            wp4y_=purple_battery_y_;
        }


        if(aruco_one_wp5_color_== "blue"){
            wp5x_=blue_battery_x_;
            wp5y_=blue_battery_y_;
        }
        if(aruco_one_wp5_color_== "red"){
            wp5x_=red_battery_x_;
            wp5y_=red_battery_y_;
        }
        if(aruco_one_wp5_color_== "green"){
            wp5x_=green_battery_x_;
            wp5y_=green_battery_y_;
        }
        if(aruco_one_wp5_color_== "orange"){
            wp5x_=orange_battery_x_;
            wp5y_=orange_battery_y_;
        }
        if(aruco_one_wp5_color_== "purple"){
            wp5x_=purple_battery_x_;
            wp5y_=purple_battery_y_;
        }

    }
    wp1_.position.x=wp1x_;
    wp1_.position.y=wp1y_;
    wp1_.position.z=0;
    wp2_.position.x=wp2x_;
    wp2_.position.y=wp2y_;
    wp2_.position.z=0;
    wp3_.position.x=wp3x_;
    wp3_.position.y=wp3y_;
    wp3_.position.z=0;
    wp4_.position.x=wp4x_;
    wp4_.position.y=wp4y_;
    wp4_.position.z=0;
    wp5_.position.x=wp5x_;
    wp5_.position.y=wp5y_;   
    wp5_.position.z=0;

    goals_pose_.poses.push_back(wp1_);
    goals_pose_.poses.push_back(wp2_);
    goals_pose_.poses.push_back(wp3_);
    goals_pose_.poses.push_back(wp4_);
    goals_pose_.poses.push_back(wp5_);

    aruco_subscription_.reset();
}

//============================================================================================//
// Method to listen for the transform between map frame and blue battery frame
void turtle::TurtleGoals::blue_battery_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped blue_battery_t_stamped;

    try
    {
        // Lookup the transform between source_frame and target_frame
        blue_battery_t_stamped = blue_battery_tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 5s);
    }
    catch (const tf2::TransformException &ex)
    {
        // Handle the exception if the transform cannot be obtained
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    // Extract position values
    blue_battery_x_ = blue_battery_t_stamped.transform.translation.x;
    blue_battery_y_= blue_battery_t_stamped.transform.translation.y;
}


//============================================================================================//
// Method to listen for the transform between map frame and red battery frame
void turtle::TurtleGoals::red_battery_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped red_battery_t_stamped;

    try
    {
        red_battery_t_stamped = red_battery_tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 5s);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    // Extract position values
    red_battery_x_ = red_battery_t_stamped.transform.translation.x;
    red_battery_y_= red_battery_t_stamped.transform.translation.y;
}

//============================================================================================//
// Method to listen for the transform between map frame and green battery frame
void turtle::TurtleGoals::green_battery_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped green_battery_t_stamped;

    try
    {
        green_battery_t_stamped = green_battery_tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 5s);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    // Extract position values
    green_battery_x_ = green_battery_t_stamped.transform.translation.x;
    green_battery_y_= green_battery_t_stamped.transform.translation.y;
}

//============================================================================================//
// Method to listen for the transform between map frame and orange battery frame
void turtle::TurtleGoals::orange_battery_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped orange_battery_t_stamped;

    try
    {
        orange_battery_t_stamped = orange_battery_tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 5s);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    // Extract position values
    orange_battery_x_ = orange_battery_t_stamped.transform.translation.x;
    orange_battery_y_= orange_battery_t_stamped.transform.translation.y;
}

//============================================================================================//
// Method to listen for the transform between map frame and purple battery frame
void turtle::TurtleGoals::purple_battery_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped purple_battery_t_stamped;

    try
    {
        purple_battery_t_stamped = purple_battery_tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 5s);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    // Extract position values
    purple_battery_x_ = purple_battery_t_stamped.transform.translation.x;
    purple_battery_y_= purple_battery_t_stamped.transform.translation.y;
}

//============================================================================================//
// Timer callback to publish battery poses
void turtle::TurtleGoals::timer_cb(){
    if(!goals_pose_.poses.empty()){
        goals_publisher_->publish(goals_pose_);
    }
}

//============================================================================================//
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtle::TurtleGoals>("turtlegoals");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
