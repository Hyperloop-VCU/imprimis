#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QTimer>
#include <QDebug>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , publish_timer(new QTimer())
    , publish_rate_ms(100)
{
    ui->setupUi(this);
    topic = "cmd_vel";
    setWindowTitle("cmd_vel publisher GUI");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::set_node_handle(rclcpp::Node::SharedPtr nh)
{
    node_handle = nh;
    publisher = nh->create_publisher<msgType>(topic.toStdString(), 10);
    connect(publish_timer, &QTimer::timeout, this, &MainWindow::timer_callback);
    publish_timer->start(publish_rate_ms);
    this->print("Topic set to: " + topic + '\n');
    
    #ifdef STAMPED
        this->print("Message type: TwistStamped\n");
    #else
        this->print("Message type: Twist\n");
    #endif
}

void MainWindow::on_topicSet_clicked()
{

    this->topic = ui->topicInput->text();
    // ui->topicInput->clear();
    this->print("Topic set to: " + topic + '\n');

    // re-create publisher with new topic
    publish_timer->stop();
    publisher.reset();
    publisher = node_handle->create_publisher<msgType>(topic.toStdString(), 10);
    publish_timer->start(publish_rate_ms);

}


void MainWindow::print(QString s)
{
    ui->console->moveCursor(QTextCursor::End);
    ui->console->insertPlainText(s);
    ui->console->moveCursor(QTextCursor::End);   // Ensure view scrolls down
}


void MainWindow::on_zeroButton_clicked()
{   
    // publish zero immediately
    this->publish_cmd(0, 0);

    // zero slider values
    ui->linearInput->setValue(0);
    ui->angularInput->setValue(0);
    this->print("Zeroed inputs.\n");
}

void MainWindow::timer_callback()
{
    
    // read data from GUI, clamping range to [-1.0, 1.0]
    float lin_x = static_cast<float>(ui->linearInput->value());
    if (lin_x > 0) lin_x /= ui->linearInput->maximum();
    else lin_x /= -ui->linearInput->minimum();

    float ang_z = static_cast<float>(ui->angularInput->value());
    if (ang_z > 0) ang_z /= ui->angularInput->maximum();
    else ang_z /= -ui->angularInput->minimum();
    
    // publish it, adding time and frame_id if it's a twist stamped message
    this->publish_cmd(lin_x, ang_z);

    // update GUI labels
    ui->labelSpeed->setText("Speed: " + QString::number(lin_x));
    ui->labelSteering->setText("Steering: " + QString::number(ang_z));
}

void MainWindow::publish_cmd(float lin_x, float ang_z)
{
    msgType msg;
    #ifdef STAMPED
        msg.header.stamp = node_handle->get_clock()->now();
        msg.header.frame_id = "base_link";
        msg.twist.linear.x = lin_x;
        msg.twist.angular.z = ang_z;
    #else
        msg.linear.x = lin_x;
        msg.angular.z = ang_z;
    #endif
        
    publisher->publish(msg);
}