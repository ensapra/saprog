#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_rendering/render_window.hpp>

#include <X11/Xlib.h>

namespace
{
struct Options
{
  Window parent = 0;
  int width = 1;
  int height = 1;
  std::string config;
  std::string ready_file;
};

int ignore_x_errors(Display *, XErrorEvent *)
{
  // A Tk tab can be destroyed while this process is handling a resize tick.
  return 0;
}

Options parse_options(const std::vector<std::string> & arguments)
{
  Options options;
  if (arguments.size() != 6) {
    throw std::runtime_error(
      "expected parent XID, width, height, RViz config path, and readiness file");
  }
  try {
    options.parent = static_cast<Window>(std::stoul(arguments[1]));
    options.width = std::max(1, std::stoi(arguments[2]));
    options.height = std::max(1, std::stoi(arguments[3]));
  } catch (const std::exception &) {
    throw std::runtime_error("invalid X11 parent or viewport dimensions");
  }
  options.config = arguments[4];
  options.ready_file = arguments[5];
  if (options.parent == 0 || options.config.empty() || options.ready_file.empty()) {
    throw std::runtime_error("missing X11 parent, RViz config path, or readiness file");
  }
  return options;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    rclcpp::init(argc, argv);
    const auto positional_arguments = rclcpp::remove_ros_arguments(argc, argv);
    const auto options = parse_options(positional_arguments);
    // Qt receives only argv[0]; ROS and our launch arguments were parsed above.
    int qt_argc = 1;
    QApplication application(qt_argc, argv);
    application.setQuitOnLastWindowClosed(true);

    Display * display = XOpenDisplay(nullptr);
    if (display == nullptr) {
      throw std::runtime_error("could not connect to the X11 display");
    }
    XSetErrorHandler(ignore_x_errors);
    XWindowAttributes parent_attributes{};
    if (!XGetWindowAttributes(display, options.parent, &parent_attributes) ||
      parent_attributes.map_state != IsViewable)
    {
      XCloseDisplay(display);
      throw std::runtime_error("Tk viewport is not mapped; select the Robot tab first");
    }

    auto * panel = new rviz_common::RenderPanel();
    panel->setWindowFlags(Qt::FramelessWindowHint | Qt::Widget);
    panel->setAttribute(Qt::WA_NativeWindow, true);
    panel->resize(options.width, options.height);
    const Window child = static_cast<Window>(panel->winId());
    XReparentWindow(display, child, options.parent, 0, 0);
    XResizeWindow(display, child, static_cast<unsigned int>(options.width),
      static_cast<unsigned int>(options.height));
    XSync(display, False);

    Window query_root = 0;
    Window actual_parent = 0;
    Window * child_windows = nullptr;
    unsigned int child_count = 0;
    const bool parented = XQueryTree(
      display, child, &query_root, &actual_parent, &child_windows, &child_count) != 0;
    if (child_windows != nullptr) {
      XFree(child_windows);
    }
    if (!parented || actual_parent != options.parent) {
      delete panel;
      XCloseDisplay(display);
      throw std::runtime_error("could not attach the RViz surface to the Tk viewport");
    }

    // The native widget is already a child of Tk before Qt maps it. This prevents
    // the window manager from ever seeing an independent RViz desktop window.
    panel->show();
    panel->getRenderWindow()->initialize();

    auto ros_node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
      "saprog_embedded_rviz");
    auto manager = std::make_shared<rviz_common::VisualizationManager>(
      panel, ros_node, nullptr, ros_node->get_raw_node()->get_clock());
    panel->initialize(manager.get());
    manager->initialize();

    rviz_common::Config config;
    rviz_common::YamlConfigReader reader;
    reader.readFile(config, QString::fromStdString(options.config));
    if (reader.error()) {
      throw std::runtime_error(
        "could not read RViz configuration: " + reader.errorMessage().toStdString());
    }
    manager->load(config.mapGetChild("Visualization Manager"));
    manager->startUpdate();
    std::ofstream ready(options.ready_file, std::ios::out | std::ios::trunc);
    if (!ready) {
      throw std::runtime_error("could not write renderer readiness marker");
    }
    ready << "ready\n";
    ready.close();

    QTimer resize_timer;
    QObject::connect(&resize_timer, &QTimer::timeout, panel, [=, &application]() {
      if (!rclcpp::ok()) {
        application.quit();
        return;
      }
      XWindowAttributes attributes{};
      if (!XGetWindowAttributes(display, options.parent, &attributes)) {
        application.quit();
        return;
      }
      if (attributes.map_state == IsViewable && attributes.width > 0 && attributes.height > 0) {
        XMoveWindow(display, child, 0, 0);
        XResizeWindow(
          display, child, static_cast<unsigned int>(attributes.width),
          static_cast<unsigned int>(attributes.height));
        panel->resize(attributes.width, attributes.height);
      }
    });
    resize_timer.start(150);

    const int result = application.exec();
    resize_timer.stop();
    manager->stopUpdate();
    manager.reset();
    panel->hide();
    XReparentWindow(display, child, DefaultRootWindow(display), 0, 0);
    XSync(display, False);
    delete panel;
    ros_node.reset();
    XCloseDisplay(display);
    rclcpp::shutdown();
    return result;
  } catch (const std::exception & error) {
    std::cerr << "embedded RViz: " << error.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return EXIT_FAILURE;
  }
}
