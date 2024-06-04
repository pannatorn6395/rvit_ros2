#ifndef MISSIONDISPLAYWIDGET_H
#define MISSIONDISPLAYWIDGET_H

#include <rclcpp/rclcpp.hpp>
#include <QWidget>
#include <QLabel>
#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QListWidget>

class MissionDisplayWidget : public QWidget {
    Q_OBJECT
public:
    explicit MissionDisplayWidget(QWidget *parent = nullptr, std::shared_ptr<rclcpp::Node> node = nullptr);
    virtual ~MissionDisplayWidget();

public slots:
    void update_user_mission_item(const std::vector<QString>& msg);
    void update_worker_mission_item(const std::vector<QString>& msg);

private:
    void create_ui_();
    void create_connect_();
    QListWidget* user_mission_list_widget;
    QListWidget* worker_mission_list_widget;
    std::shared_ptr<rclcpp::Node> node_;
};

#endif // MISSIONDISPLAYWIDGET_H
