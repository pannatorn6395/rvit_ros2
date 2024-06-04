#include <rviz_common/panel.hpp>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QRadioButton>
#include <QButtonGroup>
#include <QFrame>
#include <control_panel/RosCommunication.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
namespace obo_rviz_util {

class ControlPanel : public rviz_common::Panel {
    Q_OBJECT

public:
    explicit ControlPanel(QWidget *parent = nullptr);

protected:
    void onInitialize() override;

private:
    QLabel *stateDisplay;
    QLabel *userMissionDisplay;
    QLabel *workerMissionDisplay;
    QPushButton *injectButton;
    QPushButton *homeButton;
    QPushButton *releaseButton;
    QPushButton *startButton;
    QPushButton *stopButton;


    std::shared_ptr<RosCommunication> ros_comm_; // ROS communication object
    
    void setupStateDisplay(QVBoxLayout *layout);
    void setupButtons(QVBoxLayout *layout);
    void setupMissionTab(QVBoxLayout *layout); // แก้ไขจาก QTabWidget เป็น QVBoxLayout
    void setupDriveSection(QVBoxLayout *layout);
    void setupUserMission(QVBoxLayout *layout);
    void setupWorkerMission(QVBoxLayout *layout); 
    void setupCoordinates(QVBoxLayout *layout); 
    void setupMissionButtons(QVBoxLayout *layout); 
    void setupCoordinateInput(QLineEdit *lineEdit); 
    QLineEdit* x_pos_textbox_;
    QLineEdit* y_pos_textbox_;
    QLineEdit* z_pos_textbox_;
    std::shared_ptr<rclcpp::Node> node_; // Node instance for ROS 2 operations
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

private slots:
    void updateStateDisplay(const QString &state); // Slot to update the state display
    void handleClickedPointSignal(double x, double y, double z );
    void onInjectButtonClicked();
    void onGoHomeClicked();
    void handleStateClicked(const QString &state);
    void handleUserMissionSignal(const std::vector<QString>& missions);
    void handleWorkerMissionSignal(const std::vector<QString>& missions);
    void activateCustomNavTool();

};


} // namespace obo_rviz_util
