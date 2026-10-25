#include <memory>

#include <QApplication>
#include <QCoreApplication>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

#include "allegro_hand_v6_gui/main_window.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling, true);
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps, true);
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("Wonik Robotics");
  QCoreApplication::setApplicationName("Allegro Hand V6 GUI");

  auto node = std::make_shared<rclcpp::Node>("allegro_hand_v6_gui");
  allegro_hand_v6_gui::MainWindow window(node);
  window.show();

  QTimer ros_timer;
  QObject::connect(&ros_timer, &QTimer::timeout, [&app, &node]() {
    rclcpp::spin_some(node);
    if (!rclcpp::ok()) {
      app.quit();
    }
  });
  ros_timer.start(20);

  const int result = app.exec();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
