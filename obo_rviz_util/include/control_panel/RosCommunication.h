#ifndef ROS_COMMUNICATION_H
#define ROS_COMMUNICATION_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <QObject>
#include <memory> // Include for std::enable_shared_from_this
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

// ROS2 Custom Datatypes
#include "obo_nav_msgs/msg/mission_item_list.hpp" 
//from --->#include "sr1_msgs/msg/mission_item_list.hpp"

#include "obo_nav_msgs/msg/worker_mission_report_list.hpp"
//from -->#include "sr1_msgs/msg/worker_mission_report_list.hpp"

#include "obo_sensor_msgs/msg/motor_state.hpp"
//from -->#include "sr1b_base/msg/motor_state.hpp"

#include "obo_sensor_msgs/msg/charging_dock_status.hpp"
//from -->#include "sr1_charging_dock/msg/charging_dock_status.hpp"


// ROS2 Custom Services
#include "obo_nav_msgs/srv/set_state.hpp"
//from -->##include "sr1_state_controller/srv/set_state.hpp"


#include "std_srvs/srv/trigger.hpp"

#include "obo_nav_msgs/srv/inject_mission.hpp"
#include "obo_nav_msgs/srv/inject_mission_list.hpp"

//from -->#include "sr1_msgs/srv/inject_mission.hpp"

#include "topic_tools_interfaces/srv/mux_select.hpp"
#include "obo_sensor_msgs/msg/flashlight_status.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
namespace obo_rviz_util {
class RosCommunication : public QObject, public rclcpp::Node {
    Q_OBJECT
public:
    RosCommunication();
    bool inject_mission(double x, double y, double z);
    void logInfo(const std::string& message);
    void logError(const std::string& message);
    bool SetState(const std::string& state);
    bool gohome_mission();

signals:
    void stateUpdated(const QString &state);
    void clicked_point_signal(double x, double y, double z); // Declare your custom signal here
    void mux_topic_signal(const QString &topic);
    void user_mission_signal(std::vector<QString>);
    void worker_mission_signal(std::vector<QString>);


private:
    //callback
    void state_callback(const std_msgs::msg::String::SharedPtr msg);
    void goal_clicked_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void cmd_mux_monitor_callback(const std_msgs::msg::String::SharedPtr msg);
    void user_mission_callback(const obo_nav_msgs::msg::MissionItemList::SharedPtr msg);
    void worker_mission_callback(const obo_nav_msgs::msg::WorkerMissionReportList::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_mux_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clicked_sub;
    rclcpp::Subscription<obo_nav_msgs::msg::MissionItemList>::SharedPtr user_mission_sub;
    rclcpp::Subscription<obo_nav_msgs::msg::WorkerMissionReportList>::SharedPtr worker_mission_sub;


    rclcpp::Client<obo_nav_msgs::srv::SetState>::SharedPtr set_state_client_;
    rclcpp::Client<obo_nav_msgs::srv::InjectMission>::SharedPtr inject_mission_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr inject_user_mission_client_;
    rclcpp::Client<topic_tools_interfaces::srv::MuxSelect>::SharedPtr mux_cmd_vel_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr return_home_srv_client_;   
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};
}// namespace obo_rviz_util
#endif // ROS_COMMUNICATION_H
