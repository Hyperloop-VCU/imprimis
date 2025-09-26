#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


#define STAMPED

// I will change this to use some other method
// of switching between the two message types
// if necessary. This does require rebuilding
// the package every time, but it doesn't take that long.
#ifdef STAMPED
    using msgType = geometry_msgs::msg::TwistStamped;
#else
    using msgType = geometry_msgs::msg::Twist;
#endif

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void print(QString s);
    void set_node_handle(rclcpp::Node::SharedPtr nh);

private slots:

    void on_topicSet_clicked();

    void on_zeroButton_clicked();

    void timer_callback();

private:
    Ui::MainWindow *ui;
    QString topic;
    rclcpp::Node::SharedPtr node_handle; 
    QTimer* publish_timer;
    rclcpp::Publisher<msgType>::SharedPtr publisher;
    float min_linear_velocity, max_linear_velocity;
    float min_angular_velocity, max_angular_veloity;
    int publish_rate_ms;

    void publish_cmd(float lin_x, float ang_z);
};
#endif // MAINWINDOW_H
