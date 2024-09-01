#include "tbot_through_poses.h"

//============================================================================================//
//Function to set initial pose 
void turtle::TurtleThroughPoses::set_initial_pose() {
  auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
  message.header.frame_id = "map";
  message.pose.pose.position.x = 1.0000;
  message.pose.pose.position.y = -1.590402;
  message.pose.pose.orientation.x = 0.0004452;
  message.pose.pose.orientation.y = 0.0051003;
  message.pose.pose.orientation.z = -0.7047676;
  message.pose.pose.orientation.w = 0.7094197;
  tbot_initial_pose_pub_->publish(message);
}

//============================================================================================//
// Function to get waypoints from the battery poses
void turtle::TurtleThroughPoses::goals_cb(geometry_msgs::msg::PoseArray msg){
    wp1_=msg.poses[0];
    wp2_=msg.poses[1];
    wp3_=msg.poses[2];
    wp4_=msg.poses[3];
    wp5_=msg.poses[4];

    geometry_msgs::msg::PoseStamped wp1_msg;
    wp1_msg.header.frame_id = "map";
    wp1_msg.pose.position.x = wp1_.position.x;
    wp1_msg.pose.position.y = wp1_.position.y;
    wp1_msg.pose.position.z = 0.0;
    wp1_msg.pose.orientation.x = 0.0;
    wp1_msg.pose.orientation.y = 0.0;
    wp1_msg.pose.orientation.z = 0.0;
    wp1_msg.pose.orientation.w = 1;

    wp_.push_back(wp1_msg);


    geometry_msgs::msg::PoseStamped wp2_msg;
    wp2_msg.header.frame_id = "map";
    wp2_msg.pose.position.x = wp2_.position.x;
    wp2_msg.pose.position.y = wp2_.position.y;
    wp2_msg.pose.position.z = 0.0;
    wp2_msg.pose.orientation.x = 0.0;
    wp2_msg.pose.orientation.y = 0.0;
    wp2_msg.pose.orientation.z = 0.0;
    wp2_msg.pose.orientation.w = 1;

    wp_.push_back(wp2_msg);

    geometry_msgs::msg::PoseStamped wp3_msg;
    wp3_msg.header.frame_id = "map";
    wp3_msg.pose.position.x = wp3_.position.x;
    wp3_msg.pose.position.y = wp3_.position.y;
    wp3_msg.pose.position.z = 0.0;
    wp3_msg.pose.orientation.x = 0.0;
    wp3_msg.pose.orientation.y = 0.0;
    wp3_msg.pose.orientation.z = 0.0;
    wp3_msg.pose.orientation.w = 1;

    wp_.push_back(wp3_msg);

    geometry_msgs::msg::PoseStamped wp4_msg;
    wp4_msg.header.frame_id = "map";
    wp4_msg.pose.position.x = wp4_.position.x;
    wp4_msg.pose.position.y = wp4_.position.y;
    wp4_msg.pose.position.z = 0.0;
    wp4_msg.pose.orientation.x = 0.0;
    wp4_msg.pose.orientation.y = 0.0;
    wp4_msg.pose.orientation.z = 0.0;
    wp4_msg.pose.orientation.w = 1;

    wp_.push_back(wp4_msg);

    geometry_msgs::msg::PoseStamped wp5_msg;
    wp5_msg.header.frame_id = "map";
    wp5_msg.pose.position.x = wp5_.position.x;
    wp5_msg.pose.position.y = wp5_.position.y;
    wp5_msg.pose.position.z = 0.0;
    wp5_msg.pose.orientation.x = 0.0;
    wp5_msg.pose.orientation.y = 0.0;
    wp5_msg.pose.orientation.z = 0.0;
    wp5_msg.pose.orientation.w = 1;

    wp_.push_back(wp5_msg);

    std::this_thread::sleep_for(std::chrono::seconds(10));
    send_goals();
    goals_subscription_.reset();
}

//============================================================================================//
//Function to send goals to action client
void turtle::TurtleThroughPoses::send_goals() {
  using namespace std::placeholders;
   RCLCPP_INFO(this->get_logger(),
                 "check 0 send goals");
  if (!this->tbot_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown();
  }
   RCLCPP_INFO(this->get_logger(),
                 "check 1 send goals");
    auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();
    goal_msg.poses.push_back(wp_[0]);
    goal_msg.poses.push_back(wp_[1]);
    goal_msg.poses.push_back(wp_[2]);
    goal_msg.poses.push_back(wp_[3]);
    goal_msg.poses.push_back(wp_[4]);

  RCLCPP_INFO(this->get_logger(),
                 "check 2 send goals");
  RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&TurtleThroughPoses::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&TurtleThroughPoses::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&TurtleThroughPoses::result_callback, this, _1);


    tbot_client_->async_send_goal(goal_msg,send_goal_options);
}



void turtle::TurtleThroughPoses::goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

//============================================================================================//
// Callback function for receiving feedback during goal execution
void turtle::TurtleThroughPoses::feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback) {
  RCLCPP_INFO(this->get_logger(), "tbot is driving towards the goal");
}

//============================================================================================//
// Callback function for receiving the result of the goal
void turtle::TurtleThroughPoses::result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  rclcpp::shutdown();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<turtle::TurtleThroughPoses>("tbot_nav");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
