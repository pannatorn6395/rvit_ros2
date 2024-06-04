#include "control_panel/RosCommunication.h"
#include "rclcpp/rclcpp.hpp" 
#include <QDebug>             
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace obo_rviz_util {

RosCommunication::RosCommunication() : QObject(), Node("ros_communication_node") {
    state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/sr1_state_machine/current_state",
        10,
        std::bind(&RosCommunication::state_callback, this, std::placeholders::_1));
    clicked_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 
        10, // Queue depth
        std::bind(&RosCommunication::goal_clicked_callback, this, std::placeholders::_1)
    );
    safety_mux_sub = this->create_subscription<std_msgs::msg::String>(
        "/mux_cmd_vel/selected", 
        10, 
        std::bind(&RosCommunication::cmd_mux_monitor_callback, this, std::placeholders::_1)
    );

    user_mission_sub = this->create_subscription<obo_nav_msgs::msg::MissionItemList>(
        "/sr1_mission_manager/user_mission_item", 
        10, 
        std::bind(&RosCommunication::user_mission_callback, this, std::placeholders::_1)
    );
    worker_mission_sub = this->create_subscription<obo_nav_msgs::msg::WorkerMissionReportList>(
        "/sr1_mission_manager/worker_mission_item", 
        10, 
        std::bind(&RosCommunication::worker_mission_callback, this, std::placeholders::_1)
    );


    //create service client
    set_state_client_ = this->create_client<obo_nav_msgs::srv::SetState>(
        "/robot_state_manager/set_state"
    );

    return_home_srv_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/sr1_mission_executor/return_home"
    );

    inject_mission_client_ = this->create_client<obo_nav_msgs::srv::InjectMission>(
        "/sr1_mission_manager/inject_mission"
    );

    inject_user_mission_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/user_mission_manager/inject_user_mission"
    );

    mux_cmd_vel_client_ = this->create_client<topic_tools_interfaces::srv::MuxSelect>(
        "/mux_cmd_vel/select"
    );
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // Start a new thread to spin the node
    std::thread([this]() {
        rclcpp::spin(shared_from_this());
    }).detach();
}

void RosCommunication::worker_mission_callback(const obo_nav_msgs::msg::WorkerMissionReportList::SharedPtr msg)
{
    std::vector<QString> result;
    //change int to size_t
    for(size_t i = 0 ; i < msg->worker_mission_list.size() ; i++){
        std::ostringstream stringStream;
        stringStream << (i+1) << ". "
        << msg->worker_mission_list[i].mission_type.substr(0,5)
        << "@" << msg->worker_mission_list[i].map_id
        << "([" << msg->worker_mission_list[i].start_node
        << "]->[" << msg->worker_mission_list[i].end_node << "])" ;
        std::string item_str = stringStream.str();

        result.push_back(QString(item_str.c_str()));
    }
    emit worker_mission_signal(result);
}
void RosCommunication::user_mission_callback(const obo_nav_msgs::msg::MissionItemList::SharedPtr msg)
{
    std::vector<QString> result;
    //change int to size_t
    for(size_t i = 0 ; i < msg->mission_list.size() ; i++){
        std::ostringstream stringStream;
        stringStream << (i+1) << ". "
        << msg->mission_list[i].x_pos
        << ","
        << msg->mission_list[i].y_pos
        << ","
        << msg->mission_list[i].z_pos;

        std::string item_str = stringStream.str();

        result.push_back(QString(item_str.c_str()));
    }
    emit user_mission_signal(result);
}
void RosCommunication::cmd_mux_monitor_callback(const std_msgs::msg::String::SharedPtr msg)
{
    QString topic = msg->data.c_str();
    emit mux_topic_signal(topic);
}

void RosCommunication::goal_clicked_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Create a PoseStamped message for the input
    geometry_msgs::msg::PoseStamped input_pose;
    input_pose.header.frame_id = msg->header.frame_id;
    input_pose.header.stamp = this->get_clock()->now();
    input_pose.pose.position.x = msg->pose.position.x;
    input_pose.pose.position.y = msg->pose.position.y;
    input_pose.pose.position.z = msg->pose.position.z;
    input_pose.pose.orientation.x = 0.0;
    input_pose.pose.orientation.y = 0.0;
    input_pose.pose.orientation.z = 0.0;
    input_pose.pose.orientation.w = 1.0; // Identity quaternion

    // Transform the pose to the "semantic" frame
    try {
        auto transformed_pose = tf_buffer_->transform(input_pose, "semantic");

        // Emit the signal with the transformed coordinates
        Q_EMIT clicked_point_signal(
            transformed_pose.pose.position.x,
            transformed_pose.pose.position.y,
            transformed_pose.pose.position.z
        );
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transform pose: %s", ex.what());
    }
}
void RosCommunication::logInfo(const std::string& message) {
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
}

void RosCommunication::logError(const std::string& message) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
}


bool RosCommunication::inject_mission(double x, double y, double z) {
    // Prepare the first service request
    auto inject_request = std::make_shared<obo_nav_msgs::srv::InjectMission::Request>();
    inject_request->item.x_pos = x;
    inject_request->item.y_pos = y;
    inject_request->item.z_pos = z;

    // Send the first request asynchronously
    auto inject_result = inject_mission_client_->async_send_request(inject_request);

    // Wait for the first response asynchronously
    auto inject_status = std::async(std::launch::async, [&inject_result]() {
        return inject_result.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
    });

    if (inject_status.get() && inject_result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Mission Inject SUCCESS");

        auto user_mission_request = std::make_shared<std_srvs::srv::Trigger::Request>();

        auto user_mission_result = inject_user_mission_client_->async_send_request(user_mission_request);

        auto user_mission_status = std::async(std::launch::async, [&user_mission_result]() {
            return user_mission_result.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
        });

        if (user_mission_status.get() && user_mission_result.get()->success) {
            RCLCPP_INFO(this->get_logger(), "User Mission Inject SUCCESS");
        } else {
            RCLCPP_WARN(this->get_logger(), "User Mission Inject Failed or timed out");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Mission Inject Failed or timed out");
        // Handle first service failure or timeout
    }

    return true; // Indicate function completion
}
bool RosCommunication::gohome_mission()
{
    RCLCPP_INFO(this->get_logger(), "GO HOME BUTTON PRESSED !");

    auto go_home_request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto go_home_result = return_home_srv_client_->async_send_request(go_home_request);

    auto go_home_status = std::async(std::launch::async, [&go_home_result]() {
        return go_home_result.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
    });

    if (go_home_status.get() && go_home_result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Go Home Mission  SUCCESS");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Go Home Mission  Failed or timed out");
    }
    return true; // Indicate function completion

}
bool RosCommunication::SetState(const std::string& state)
{
    RCLCPP_INFO(this->get_logger(), "Setting state to %s", state.c_str());

    auto state_request = std::make_shared<obo_nav_msgs::srv::SetState::Request>();
    state_request->state = state; // Set the state from the input parameter

    auto state_result = set_state_client_->async_send_request(state_request);

    auto state_status = std::async(std::launch::async, [&state_result]() {
        return state_result.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
    });

    if (state_status.get() && state_result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "State set to %s SUCCESS", state.c_str());
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Setting state to %s Failed or timed out", state.c_str());
        return false;
    }
}

void RosCommunication::state_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "State received: %s", msg->data.c_str());
    QString state = QString::fromStdString(msg->data);
    emit stateUpdated(state); 
}

} // namespace obo_rviz_util

#include "RosCommunication.moc"
