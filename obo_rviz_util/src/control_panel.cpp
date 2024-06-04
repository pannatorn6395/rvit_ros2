#include "control_panel/control_panel.h"
#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/visualization_manager.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "control_panel/CustomNavTool.h"

#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QRadioButton>
#include <QPushButton>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QButtonGroup>
#include "rclcpp/rclcpp.hpp" 
#include <QDebug>          
#include <QComboBox>

Q_DECLARE_METATYPE(std::vector<QString>)






namespace obo_rviz_util {

// ฟังก์ชัน setupStateDisplay
void ControlPanel::setupStateDisplay(QVBoxLayout *layout) {
    // Create horizontal layout for state information
    auto stateLayout = new QHBoxLayout();
    
    // Create STATE label
    auto stateLabel = new QLabel("STATE :");
    
    // Initialize state display label with centered text, a white background, a border, and increased width
    stateDisplay = new QLabel("CHARGING"); // Use the member variable here
    stateDisplay->setStyleSheet("QLabel { background-color: white; border: 1px solid black; padding: 5px; }");
    stateDisplay->setAlignment(Qt::AlignCenter);
    stateDisplay->setMinimumWidth(300);
    
    // Add both the label and the display to the horizontal layout
    stateLayout->addWidget(stateLabel);
    stateLayout->addWidget(stateDisplay); // Add the member variable to the layout
    
    // Add the horizontal layout to the vertical layout
    layout->addLayout(stateLayout);
}

void ControlPanel::handleClickedPointSignal(double x, double y, double z) {
    // Convert the coordinates to QString
    QString x_val = QString::number(x);
    QString y_val = QString::number(y);
    QString z_val = QString::number(z);

    // Set the converted coordinates to the text boxes
    x_pos_textbox_->setText(x_val);
    y_pos_textbox_->setText(y_val);
    z_pos_textbox_->setText(z_val);
}


// ฟังก์ชัน setupButtons
void ControlPanel::setupButtons(QVBoxLayout *layout) {
    // Create the buttons
    releaseButton = new QPushButton("Release Manual");
    connect(releaseButton, &QPushButton::clicked, [this]() {
    this->handleStateClicked("release_manual");
});
    homeButton = new QPushButton("Go Home!",this);
    connect(homeButton, &QPushButton::clicked, this, &ControlPanel::onGoHomeClicked);

    // Create horizontal layout for the buttons
    auto buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(releaseButton);
    buttonLayout->addWidget(homeButton);

    // Add the button layout to the vertical layout
    layout->addLayout(buttonLayout);

    // Create and add a horizontal line below the buttons
    auto hLine = new QFrame();
    hLine->setFrameShape(QFrame::HLine);
    hLine->setFrameShadow(QFrame::Sunken);
    layout->addWidget(hLine); // This adds the line to the layout, below the buttons
}
void ControlPanel::handleStateClicked(const QString &state) 
{
    bool result = ros_comm_->SetState(state.toStdString());
    if (result) {
        qDebug() << "State set to" << state << "successfully.";
    } else {
        qDebug() << "Failed to set state to" << state << ".";
    }

}
void ControlPanel::onGoHomeClicked() {
    bool result = ros_comm_->gohome_mission();
    if (result) {
        qDebug() << "Connected gohome_mission signals to slots.";

    } else {
        qDebug() << "Can not Connected gohome_mission signals to slots.";

    }

}
void ControlPanel::onInjectButtonClicked() {
    bool ok;
    double x = x_pos_textbox_->text().toDouble(&ok);
    double y = y_pos_textbox_->text().toDouble(&ok);
    double z = z_pos_textbox_->text().toDouble(&ok);

    QString coordinateStr = QString("1. x:%1, y:%2, z:%3")
                            .arg(QString::number(x, 'f', 2))
                            .arg(QString::number(y, 'f', 2))
                            .arg(QString::number(z, 'f', 2));
    //userMissionDisplay->setText(coordinateStr);

    //userMissionDisplay->setText(coordinateStr);

    bool result = ros_comm_->inject_mission(x, y, z);
    if (result) {
        qDebug() << "Connected signals to slots.";

        //ros_comm_->logInfo("Mission injection successful: x=" + std::to_string(x) + ", y=" + std::to_string(y) + ", z=" + std::to_string(z));
    } else {
        qDebug() << "Connected signals to slots.";

        //ros_comm_->logError("Mission injection failed: x=" + std::to_string(x) + ", y=" + std::to_string(y) + ", z=" + std::to_string(z));
    }
}

void ControlPanel::setupMissionTab(QVBoxLayout *layout) {
    // Create a tab widget
    auto tabWidget = new QTabWidget();

    // Create the Mission tab with its own layout
    auto missionTab = new QWidget();
    auto missionLayout = new QVBoxLayout(missionTab);

    // Create a horizontal layout for User and Worker mission boxes
    auto horizontalMissionLayout = new QHBoxLayout();

    // User Mission box with a border and title styling
    auto userMissionBox = new QGroupBox("User mission");
    userMissionBox->setStyleSheet(
        "QGroupBox { border: 1px solid black; margin-top: 1em; } "
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }");
    auto userMissionLayout = new QVBoxLayout(userMissionBox);
    userMissionBox->setLayout(userMissionLayout);
    userMissionBox->setFixedSize(200, 400); // Set a fixed size for the box
    horizontalMissionLayout->addWidget(userMissionBox);

    // สร้าง QLabel สำหรับแสดงพิกัดใน User Mission box
    userMissionDisplay = new QLabel("No User mission");
    userMissionDisplay->setAlignment(Qt::AlignTop | Qt::AlignLeft); // Aligns text to the top-left

    userMissionLayout->addWidget(userMissionDisplay);
    

    // Worker Mission box with a border and title styling
    auto workerMissionBox = new QGroupBox("Worker Mission");
    workerMissionBox->setStyleSheet(
        "QGroupBox { border: 1px solid black; margin-top: 1em; } "
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }");

        
    auto workerMissionLayout = new QVBoxLayout(workerMissionBox);
    workerMissionBox->setLayout(workerMissionLayout);
    workerMissionBox->setFixedSize(200, 400); // Set a fixed size for the box
    horizontalMissionLayout->addWidget(workerMissionBox);
    // สร้าง QLabel สำหรับแสดงพิกัดใน Worker Mission box
    workerMissionDisplay = new QLabel("No Worker mission");
    workerMissionDisplay->setAlignment(Qt::AlignTop | Qt::AlignLeft); // Aligns text to the top-left
    workerMissionLayout->addWidget(workerMissionDisplay);

    // Add the horizontal layout to the mission layout
    missionLayout->addLayout(horizontalMissionLayout);


    // Coordinates layout for x, y, z inputs
    QHBoxLayout *coordsLayout = new QHBoxLayout();
    coordsLayout->setSpacing(2);

    // x coordinate input
    auto xLabel = new QLabel("x:");
    coordsLayout->addWidget(xLabel);
    coordsLayout->addWidget(x_pos_textbox_); // Add the member QLineEdit for x

    // y coordinate input
    auto yLabel = new QLabel("y:");
    coordsLayout->addWidget(yLabel);
    coordsLayout->addWidget(y_pos_textbox_); // Add the member QLineEdit for y

    // z coordinate input
    auto zLabel = new QLabel("z:");
    coordsLayout->addWidget(zLabel);
    coordsLayout->addWidget(z_pos_textbox_); // Add the member QLineEdit for z

    missionLayout->addLayout(coordsLayout);

    // Dropdown menu for mission commands
    auto missionCommandComboBox = new QComboBox();
    missionCommandComboBox->addItem("Start Mission");
    missionCommandComboBox->addItem("Continue Mission");
    missionCommandComboBox->addItem("Pause Mission");
    missionCommandComboBox->addItem("Stop Mission");
    missionCommandComboBox->addItem("Inject Mission");
    // Add more items as needed

    // Execute button
    auto executeButton = new QPushButton("Execute");
    connect(executeButton, &QPushButton::clicked, [this, missionCommandComboBox]() {
        QString selectedCommand = missionCommandComboBox->currentText();
        if (selectedCommand == "Start Mission") {
            handleStateClicked("start_mission");
        } else if (selectedCommand == "Stop Mission") {
            handleStateClicked("stop_mission");
            
        } else if (selectedCommand == "Pause Mission") {
            handleStateClicked("pause_mission");
        } 
        else if (selectedCommand == "Continue Mission") {
            handleStateClicked("continue_mission");
        }else if (selectedCommand == "Inject Coordinates") {
            onInjectButtonClicked();
        }
        // Handle other commands as needed
    });

    // Layout for the dropdown and execute button
    auto commandLayout = new QHBoxLayout();
    commandLayout->addWidget(missionCommandComboBox);
    commandLayout->addWidget(executeButton);

    // Add the command layout to the mission layout
    missionLayout->addLayout(commandLayout);

    // Set the overall layout for the missionTab
    missionTab->setLayout(missionLayout);

    // Add the missionTab to the tab widget
    tabWidget->addTab(missionTab, "Mission");

    // Add the tab widget to the main layout
    layout->addWidget(tabWidget);
}
void ControlPanel::setupDriveSection(QVBoxLayout *layout) {
    // Add a horizontal line above the DRIVE choices
    auto topHLine = new QFrame();
    topHLine->setFrameShape(QFrame::HLine);
    topHLine->setFrameShadow(QFrame::Sunken);
    layout->addWidget(topHLine);

    // Create DRIVE label
    auto driveLabel = new QLabel("DRIVE:");

    // Create a horizontal layout for DRIVE and its choices
    auto driveLayout = new QHBoxLayout();
    driveLayout->addWidget(driveLabel);

    // Create a button group for radio buttons to ensure only one is selected at a time
    auto driveButtonGroup = new QButtonGroup();

    // Create SAFETY radio button
    auto safetyRadioButton = new QRadioButton("SAFETY");
    driveButtonGroup->addButton(safetyRadioButton);
    driveLayout->addWidget(safetyRadioButton);

    // Create STATE radio button
    auto stateRadioButton = new QRadioButton("STATE");
    driveButtonGroup->addButton(stateRadioButton);
    driveLayout->addWidget(stateRadioButton);

    // Add the drive layout to the main layout
    layout->addLayout(driveLayout);
}


void ControlPanel::handleUserMissionSignal(const std::vector<QString>& missions) {
    qDebug() << "Received missions Item:" << missions;
    QString missionsText;
    for (const QString& mission : missions) {
        missionsText += mission + "\n";  // รวมข้อความจากแต่ละ mission และเพิ่มขึ้นบรรทัดใหม่
    }

    userMissionDisplay->setText(missionsText);  // ตั้งค่าข้อความที่รวมแล้วให้กับ QLabel
}

void ControlPanel::handleWorkerMissionSignal(const std::vector<QString>& missions) {
    QString missionsText;
    for (const QString& mission : missions) {
        missionsText += mission + "\n";  // รวมข้อความจากแต่ละ mission และเพิ่มขึ้นบรรทัดใหม่
    }

    workerMissionDisplay->setText(missionsText);  // ตั้งค่าข้อความที่รวมแล้วให้กับ QLabel
}
void ControlPanel::updateStateDisplay(const QString &state) {
    qDebug() << "Updating state display to:" << state;
    stateDisplay->setText(state); // Set the text of stateDisplay to thze new state
}
void ControlPanel::onInitialize()
    {
    qDebug() << "ControlPanel initialized.";
    }
// Constructor ของ ControlPanel
void ControlPanel::activateCustomNavTool() {
    // Get the ToolManager from the DisplayContext
    auto* tool_manager = getDisplayContext()->getToolManager();
    
    // Attempt to retrieve your custom tool
    CustomNavTool* nav_tool = dynamic_cast<CustomNavTool*>(tool_manager->getTool("obo_rviz_util::CustomNavTool"));
    
    if (nav_tool) {
        // If the tool is found, set it as the current tool
        tool_manager->setCurrentTool(nav_tool);
    }
}


ControlPanel::ControlPanel(QWidget *parent)
    : rviz_common::Panel(parent),
      stateDisplay(new QLabel("CHARGING")),
      ros_comm_(std::make_shared<RosCommunication>()),
      x_pos_textbox_(new QLineEdit("0.00")),  // Initialize with default text
      y_pos_textbox_(new QLineEdit("0.00")),  // Initialize with default text
      z_pos_textbox_(new QLineEdit("0.00"))  // Initialize with default text
    { // Initialize ROS 2 node

    QVBoxLayout *layout = new QVBoxLayout(this);
    setupStateDisplay(layout);
    setupButtons(layout);
    setupMissionTab(layout); // This function needs to integrate the QLineEdit widgets
    setupDriveSection(layout);

     // Add your new button here
    auto activateNavToolButton = new QPushButton("Activate Custom Nav Tool", this);
    layout->addWidget(activateNavToolButton);
    connect(activateNavToolButton, &QPushButton::clicked, this, &ControlPanel::activateCustomNavTool);

    qRegisterMetaType<std::vector<QString>>("std::vector<QString>");

    // Connect signals to slots
    connect(ros_comm_.get(), &RosCommunication::stateUpdated,
            this, &ControlPanel::updateStateDisplay);
    connect(ros_comm_.get(), &RosCommunication::clicked_point_signal, 
            this, &ControlPanel::handleClickedPointSignal);
    //connect(injectButton, &QPushButton::clicked, this, &ControlPanel::onInjectButtonClicked);
    connect(ros_comm_.get(), &RosCommunication::worker_mission_signal, 
            this, &ControlPanel::handleWorkerMissionSignal);
    connect(ros_comm_.get(), &RosCommunication::user_mission_signal, 
            this, &ControlPanel::handleUserMissionSignal);

    qDebug() << "Connected signals to slots.";
    }
} // namespace obo_rviz_util

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(obo_rviz_util::ControlPanel, rviz_common::Panel)
