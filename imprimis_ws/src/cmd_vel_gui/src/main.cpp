#include "mainwindow.h"
#include "rclcpp/rclcpp.hpp"

#include <QApplication>

class GuiApp : public QApplication
{
public:
  rclcpp::Node::SharedPtr nh;

  explicit GuiApp(int& argc, char** argv) : QApplication(argc, argv)
  {
    rclcpp::init(argc, argv);
    nh = rclcpp::Node::make_shared("cmd_vel_gui");
  }

  ~GuiApp()
  {
    rclcpp::shutdown();
  }

  int exec()
  {
    MainWindow w;
    RCLCPP_INFO(nh->get_logger(), "Starting cmd_vel_gui");
    w.set_node_handle(nh);
    w.show();

    return QApplication::exec();
  }
};

int main(int argc, char** argv)
{
  GuiApp app(argc, argv);
  return app.exec();
}
