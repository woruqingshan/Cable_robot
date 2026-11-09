// Actual RViz VisualizationFrame with its native RenderWindow screenshot API.
#include <QApplication>
#include <QTimer>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_rendering/render_window.hpp>

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: rviz_snapshot config.rviz output.png [--ros-args ...]\n";
    return 2;
  }
  const char * local = std::getenv("ROS_LOCALHOST_ONLY");
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  if (!local || std::string(local) != "1" || !domain || std::stoi(domain) < 1 ||
    std::stoi(domain) > 101)
  {
    std::cerr << "Private loopback ROS domain required\n";
    return 2;
  }
  const QString config = QString::fromLocal8Bit(argv[1]);
  const std::string output = argv[2];
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_snapshot");
  auto frame = std::make_unique<rviz_common::VisualizationFrame>(node);
  frame->setApp(&app);
  frame->setSplashPath("");
  frame->initialize(node, config);
  frame->show();
  int status = 1;
  QTimer::singleShot(12000, [&]() {
    try {
      frame->getRenderWindow()->captureScreenShot(output);
      std::cout << "Native RViz render screenshot saved: " << output << std::endl;
      status = 0;
    } catch (const std::exception & error) {
      std::cerr << error.what() << std::endl;
    }
    app.quit();
  });
  app.exec();
  frame.reset();
  node.reset();
  rclcpp::shutdown();
  return status;
}
