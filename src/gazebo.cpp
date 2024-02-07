#include <chrono>
#include <tuple>
#include <vector>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class GazeboService : public rclcpp::Node
{
  public:
    GazeboService()
    : Node("gazebo_service")
    {
      goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("goal_pose", 10);
      
      task_success_service_ = this->create_service<std_srvs::srv::Empty>("task_success", std::bind(&GazeboService::task_success_callback, this, _1, _2));
      task_fail_service_ = this->create_service<std_srvs::srv::Empty>("task_fail", std::bind(&GazeboService::task_fail_callback, this, _1, _2));
      
      entity_state_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/demo/set_entity_state");
      
      while (!entity_state_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }
      
      timer_ = this->create_wall_timer(
        60s, std::bind(&GazeboService::timer_callback, this));
    }
    
  private:
    void timer_callback()
    {
      set_agent_state();
      set_goal_state();
    }
    
    void set_goal_state()
    {
      int i = rand() % 4;
      auto goal_pose = goal_pose_list_[i];
      
      auto state = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
      state->state.name = "unit_sphere";
      state->state.pose.position.x = std::get<0>(goal_pose);
      state->state.pose.position.y = std::get<1>(goal_pose);
      state->state.pose.position.z = std::get<2>(goal_pose);
      
      entity_state_client_->async_send_request(state);
      
      auto pose = geometry_msgs::msg::Pose();
      pose.position.x = std::get<0>(goal_pose);
      pose.position.y = std::get<1>(goal_pose);
      
      goal_pose_publisher_->publish(pose);
    }
    
    void set_agent_state()
    {
      auto state = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
      state->state.name = "turtlebot3_burger";
      state->state.pose.position.x = 0.0;
      state->state.pose.position.y = 0.0;
      state->state.pose.position.z = 0.0;
      
      entity_state_client_->async_send_request(state);
    }
    
    void task_success_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, 
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      set_agent_state();
      set_goal_state();
      timer_.reset();
    }
    
    void task_fail_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, 
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      set_agent_state();
      set_goal_state();
      timer_.reset();
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_pose_publisher_;
    
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr task_success_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr task_fail_service_;
    
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr entity_state_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<std::tuple<float, float, float>> goal_pose_list_ = {
      {-2.0, -4.0, 0.0},
      {-2.0, 4.0, 0.0},
      {3.0, -4.0, 0.0},
      {2.0, 4.0, 0.0},
    };
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboService>());
  rclcpp::shutdown();
  return 0;
}
