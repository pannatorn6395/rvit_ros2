#ifndef CUSTOM_NAV_TOOL_H
#define CUSTOM_NAV_TOOL_H

#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <rclcpp/rclcpp.hpp>
namespace obo_rviz_util {

class CustomNavTool : public rviz_default_plugins::tools::PoseTool
{
public:
    CustomNavTool();
    virtual ~CustomNavTool() = default;

protected:
    void onInitialize() override;
    void onPoseSet(double x, double y, double theta) override;

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};
}
#endif // CUSTOM_NAV_TOOL_H
