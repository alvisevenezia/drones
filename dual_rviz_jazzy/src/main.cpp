#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include <QApplication>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QSplitter>
#include <QStatusBar>
#include <QStackedWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap_msgs/msg/info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>

namespace
{

struct DashboardSnapshot
{
  QImage rgb_image;
  QImage depth_image;

  double odom_x{0.0};
  double odom_y{0.0};
  double odom_z{0.0};
  double odom_yaw{0.0};
  double speed{0.0};

  double goal_x{0.0};
  double goal_y{0.0};
  bool has_goal{false};

  double drift{std::numeric_limits<double>::quiet_NaN()};
  double path_len{0.0};
  double drift_ratio{std::numeric_limits<double>::quiet_NaN()};

  int ref_id{0};
  int loop_closure_id{0};
  int current_goal_id{0};
  std::size_t wm_nodes{0};
  std::size_t loop_closure_count{0};
  std::string info_text;
};

static QString qs(const std::string & text)
{
  return QString::fromStdString(text);
}

static double yawFromQuaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

static std::string makeInfoText(const rtabmap_msgs::msg::Info & info)
{
  std::ostringstream oss;
  oss << "RTAB-Map info\n";
  oss << "ref_id=" << info.ref_id
      << " loop_closure_id=" << info.loop_closure_id
      << " current_goal_id=" << info.current_goal_id << '\n';
  oss << "wm_state=" << info.wm_state.size()
      << " posterior=" << info.posterior_keys.size()
      << " likelihood=" << info.likelihood_keys.size()
      << " raw_likelihood=" << info.raw_likelihood_keys.size() << '\n';
  oss << "stats=" << info.stats_keys.size() << " keys";
  return oss.str();
}

static QImage imageFromMsg(const sensor_msgs::msg::Image & msg)
{
  const int width = static_cast<int>(msg.width);
  const int height = static_cast<int>(msg.height);
  if (width <= 0 || height <= 0) {
    return QImage();
  }

  const auto * bytes = msg.data.data();
  const int step = static_cast<int>(msg.step);

  if (msg.encoding == "rgb8") {
    return QImage(bytes, width, height, step, QImage::Format_RGB888).copy();
  }
  if (msg.encoding == "bgr8") {
    return QImage(bytes, width, height, step, QImage::Format_RGB888).rgbSwapped().copy();
  }
  if (msg.encoding == "rgba8") {
    return QImage(bytes, width, height, step, QImage::Format_RGBA8888).copy();
  }
  if (msg.encoding == "bgra8") {
    return QImage(bytes, width, height, step, QImage::Format_RGBA8888).rgbSwapped().copy();
  }
  if (msg.encoding == "mono8") {
    return QImage(bytes, width, height, step, QImage::Format_Grayscale8).copy();
  }
  return QImage();
}

static QImage depthFromMsg(const sensor_msgs::msg::Image & msg)
{
  const int width = static_cast<int>(msg.width);
  const int height = static_cast<int>(msg.height);
  if (width <= 0 || height <= 0) {
    return QImage();
  }

  QImage image(width, height, QImage::Format_Grayscale8);
  image.fill(Qt::black);
  const auto clamp8 = [](double v) -> uchar {
    return static_cast<uchar>(std::clamp(v, 0.0, 255.0));
  };

  if (msg.encoding == "32FC1") {
    const auto * row0 = reinterpret_cast<const float *>(msg.data.data());
    const int stride = static_cast<int>(msg.step / sizeof(float));
    for (int y = 0; y < height; ++y) {
      auto * dst = image.scanLine(y);
      const float * src = row0 + y * stride;
      for (int x = 0; x < width; ++x) {
        const float depth = src[x];
        if (!std::isfinite(depth) || depth <= 0.0f) {
          dst[x] = 0;
          continue;
        }
        const double normalized = 1.0 - std::clamp(static_cast<double>(depth) / 8.0, 0.0, 1.0);
        dst[x] = clamp8(normalized * 255.0);
      }
    }
    return image;
  }

  if (msg.encoding == "16UC1") {
    const auto * row0 = reinterpret_cast<const std::uint16_t *>(msg.data.data());
    const int stride = static_cast<int>(msg.step / sizeof(std::uint16_t));
    for (int y = 0; y < height; ++y) {
      auto * dst = image.scanLine(y);
      const std::uint16_t * src = row0 + y * stride;
      for (int x = 0; x < width; ++x) {
        const double depth_m = static_cast<double>(src[x]) / 1000.0;
        if (depth_m <= 0.0) {
          dst[x] = 0;
          continue;
        }
        const double normalized = 1.0 - std::clamp(depth_m / 8.0, 0.0, 1.0);
        dst[x] = clamp8(normalized * 255.0);
      }
    }
    return image;
  }

  return QImage();
}

class ImageCard : public QGroupBox
{
public:
  explicit ImageCard(const QString & title, QWidget * parent = nullptr)
  : QGroupBox(title, parent)
  {
    auto * layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    image_label_ = new QLabel(this);
    image_label_->setAlignment(Qt::AlignCenter);
    image_label_->setMinimumHeight(220);
    image_label_->setStyleSheet("background: #101010; color: #b0b0b0; border: 1px solid #333;");
    image_label_->setText("Waiting for image...");
    layout->addWidget(image_label_);
  }

  void setImage(const QImage & image)
  {
    current_image_ = image;
    if (current_image_.isNull()) {
      image_label_->setText("Waiting for image...");
      image_label_->setPixmap(QPixmap());
      return;
    }
    image_label_->setText(QString());
    image_label_->setPixmap(
      QPixmap::fromImage(current_image_).scaled(
        image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  }

protected:
  void resizeEvent(QResizeEvent * event) override
  {
    QGroupBox::resizeEvent(event);
    if (!current_image_.isNull()) {
      setImage(current_image_);
    }
  }

private:
  QLabel * image_label_{nullptr};
  QImage current_image_;
};

class StatsCard : public QGroupBox
{
public:
  explicit StatsCard(QWidget * parent = nullptr)
  : QGroupBox("Live Stats", parent)
  {
    auto * layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);

    auto * form = new QFormLayout();
    form->setLabelAlignment(Qt::AlignRight);

    auto make_value = [&](const QString & initial = "-") {
      auto * label = new QLabel(initial, this);
      label->setTextInteractionFlags(Qt::TextSelectableByMouse);
      label->setStyleSheet("font-family: monospace;");
      return label;
    };

    odom_value_ = make_value();
    goal_value_ = make_value();
    speed_value_ = make_value();
    drift_value_ = make_value();
    ratio_value_ = make_value();
    path_value_ = make_value();
    rtabmap_value_ = make_value();
    loop_value_ = make_value();

    form->addRow("Odometry", odom_value_);
    form->addRow("Goal", goal_value_);
    form->addRow("Velocity", speed_value_);
    form->addRow("Drift abs.", drift_value_);
    form->addRow("Drift rel.", ratio_value_);
    form->addRow("Path length", path_value_);
    form->addRow("RTAB-Map", rtabmap_value_);
    form->addRow("Loop closing", loop_value_);
    layout->addLayout(form);

    info_text_ = new QPlainTextEdit(this);
    info_text_->setReadOnly(true);
    info_text_->setMaximumBlockCount(1000);
    info_text_->setMinimumHeight(180);
    info_text_->setStyleSheet("font-family: monospace; background: #111; color: #d0d0d0;");
    layout->addWidget(info_text_);
  }

  void updateFromSnapshot(const DashboardSnapshot & snapshot)
  {
    odom_value_->setText(
      QString("x=%1  y=%2  z=%3  yaw=%4")
        .arg(snapshot.odom_x, 0, 'f', 2)
        .arg(snapshot.odom_y, 0, 'f', 2)
        .arg(snapshot.odom_z, 0, 'f', 2)
        .arg(snapshot.odom_yaw, 0, 'f', 2));

    if (snapshot.has_goal) {
      goal_value_->setText(
        QString("x=%1  y=%2")
          .arg(snapshot.goal_x, 0, 'f', 2)
          .arg(snapshot.goal_y, 0, 'f', 2));
    } else {
      goal_value_->setText("-");
    }

    speed_value_->setText(QString("%1 m/s").arg(snapshot.speed, 0, 'f', 2));
    drift_value_->setText(std::isfinite(snapshot.drift) ? QString("%1 m").arg(snapshot.drift, 0, 'f', 3) : "-");
    ratio_value_->setText(
      std::isfinite(snapshot.drift_ratio) ? QString("%1 %%").arg(snapshot.drift_ratio, 0, 'f', 2) : "-");
    path_value_->setText(QString("%1 m").arg(snapshot.path_len, 0, 'f', 2));
    rtabmap_value_->setText(
      QString("ref=%1 loop=%2 wm=%3 goal=%4")
        .arg(snapshot.ref_id)
        .arg(snapshot.loop_closure_id)
        .arg(static_cast<int>(snapshot.wm_nodes))
        .arg(snapshot.current_goal_id));
    loop_value_->setText(QString("seen=%1").arg(static_cast<int>(snapshot.loop_closure_count)));
    info_text_->setPlainText(qs(snapshot.info_text));
  }

private:
  QLabel * odom_value_{nullptr};
  QLabel * goal_value_{nullptr};
  QLabel * speed_value_{nullptr};
  QLabel * drift_value_{nullptr};
  QLabel * ratio_value_{nullptr};
  QLabel * path_value_{nullptr};
  QLabel * rtabmap_value_{nullptr};
  QLabel * loop_value_{nullptr};
  QPlainTextEdit * info_text_{nullptr};
};

class DashboardNode : public rclcpp::Node
{
public:
  DashboardNode()
  : Node("slam_dashboard")
  {
    auto qos = rclcpp::QoS(10).best_effort();
    auto reliable_qos = rclcpp::QoS(10).reliable();

    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/rgb/image_raw", qos, [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const QImage image = imageFromMsg(*msg);
        if (!image.isNull()) {
          snapshot_.rgb_image = image;
        }
      });

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", qos, [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const QImage image = depthFromMsg(*msg);
        if (!image.isNull()) {
          snapshot_.depth_image = image;
        }
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", reliable_qos, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto & p = msg->pose.pose.position;
        const auto & q = msg->pose.pose.orientation;
        snapshot_.odom_x = p.x;
        snapshot_.odom_y = p.y;
        snapshot_.odom_z = p.z;
        snapshot_.odom_yaw = yawFromQuaternion(q.x, q.y, q.z, q.w);
        const double vx = msg->twist.twist.linear.x;
        const double vy = msg->twist.twist.linear.y;
        const double vz = msg->twist.twist.linear.z;
        snapshot_.speed = std::sqrt(vx * vx + vy * vy + vz * vz);

        if (has_prev_odom_) {
          const double dx = p.x - prev_odom_x_;
          const double dy = p.y - prev_odom_y_;
          const double dz = p.z - prev_odom_z_;
          snapshot_.path_len += std::sqrt(dx * dx + dy * dy + dz * dz);
          if (std::isfinite(snapshot_.drift) && snapshot_.path_len > 0.1) {
            snapshot_.drift_ratio = (snapshot_.drift / snapshot_.path_len) * 100.0;
          }
        }

        prev_odom_x_ = p.x;
        prev_odom_y_ = p.y;
        prev_odom_z_ = p.z;
        has_prev_odom_ = true;
      });

    drift_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/vio_drift", reliable_qos, [this](const std_msgs::msg::Float64::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.drift = msg->data;
        if (snapshot_.path_len > 0.1) {
          snapshot_.drift_ratio = (snapshot_.drift / snapshot_.path_len) * 100.0;
        }
      });

    info_sub_ = create_subscription<rtabmap_msgs::msg::Info>(
      "/rtabmap/info", reliable_qos, [this](const rtabmap_msgs::msg::Info::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.ref_id = msg->ref_id;
        snapshot_.loop_closure_id = msg->loop_closure_id;
        snapshot_.current_goal_id = msg->current_goal_id;
        snapshot_.wm_nodes = msg->wm_state.size();
        if (msg->loop_closure_id != 0 && msg->loop_closure_id != last_loop_closure_id_) {
          ++snapshot_.loop_closure_count;
          last_loop_closure_id_ = msg->loop_closure_id;
        }
        snapshot_.info_text = makeInfoText(*msg);
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/explore/goal", reliable_qos, [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.goal_x = msg->point.x;
        snapshot_.goal_y = msg->point.y;
        snapshot_.has_goal = true;
      });
  }

  DashboardSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

private:
  mutable std::mutex mutex_;
  DashboardSnapshot snapshot_;
  bool has_prev_odom_{false};
  double prev_odom_x_{0.0};
  double prev_odom_y_{0.0};
  double prev_odom_z_{0.0};
  int last_loop_closure_id_{0};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr drift_sub_;
  rclcpp::Subscription<rtabmap_msgs::msg::Info>::SharedPtr info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
};

class TelemetryPane : public QWidget
{
public:
  explicit TelemetryPane(std::shared_ptr<DashboardNode> node, QWidget * parent = nullptr)
  : QWidget(parent), node_(std::move(node))
  {
    auto * root = new QVBoxLayout(this);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(8);

    rgb_ = new ImageCard("Camera RGB", this);
    depth_ = new ImageCard("Camera Depth", this);
    stats_ = new StatsCard(this);

    root->addWidget(rgb_, 1);
    root->addWidget(depth_, 1);
    root->addWidget(stats_, 1);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &TelemetryPane::refresh);
    timer_->start(100);
  }

private:
  void refresh()
  {
    const DashboardSnapshot snapshot = node_->snapshot();
    rgb_->setImage(snapshot.rgb_image);
    depth_->setImage(snapshot.depth_image);
    stats_->updateFromSnapshot(snapshot);
  }

  std::shared_ptr<DashboardNode> node_;
  ImageCard * rgb_{nullptr};
  ImageCard * depth_{nullptr};
  StatsCard * stats_{nullptr};
  QTimer * timer_{nullptr};
};

class RvizPane : public QWidget
{
public:
  RvizPane(
    QApplication * app,
    const std::string & node_name,
    const QString & point_config_path,
    const QString & title,
    QWidget * parent = nullptr)
  : QWidget(parent)
  {
    auto * root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(6);

    point_ros_node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(node_name);
    auto * title_label = new QLabel(title, this);
    title_label->setStyleSheet("font-weight: 600; padding: 8px 8px 0 8px;");
    root->addWidget(title_label);

    point_frame_ = createFrame(app, point_ros_node_, title, point_config_path, this);
    root->addWidget(point_frame_, 1);
  }

  RvizPane(
    QApplication * app,
    const std::string & node_name,
    const QString & point_config_path,
    const QString & cube_config_path,
    const QString & title,
    const QString & toggle_label,
    QWidget * parent = nullptr)
  : QWidget(parent)
  {
    auto * root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(6);

    point_ros_node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(node_name + "_points");
    cube_ros_node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(node_name + "_cubes");

    auto * header = new QHBoxLayout();
    header->setContentsMargins(8, 8, 8, 0);

    auto * title_label = new QLabel(title, this);
    title_label->setStyleSheet("font-weight: 600;");

    toggle_button_ = new QPushButton(toggle_label, this);
    toggle_button_->setCheckable(true);
    toggle_button_->setChecked(true);
    connect(toggle_button_, &QPushButton::toggled, this, &RvizPane::setCubeMode);

    header->addWidget(title_label);
    header->addStretch(1);
    header->addWidget(toggle_button_);
    root->addLayout(header);

    stacked_ = new QStackedWidget(this);

    point_frame_ = createFrame(app, point_ros_node_, title, point_config_path, this);
    cube_frame_ = createFrame(app, cube_ros_node_, title, cube_config_path, this);

    stacked_->addWidget(point_frame_);
    stacked_->addWidget(cube_frame_);
    stacked_->setCurrentIndex(0);
    root->addWidget(stacked_, 1);
  }

private:
  rviz_common::VisualizationFrame * createFrame(
    QApplication * app,
    const std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> & ros_node,
    const QString & title,
    const QString & config_path,
    QWidget * parent)
  {
    auto * frame = new rviz_common::VisualizationFrame(ros_node, parent);
    frame->setWindowFlags(Qt::Widget);
    frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    frame->setApp(app);
    frame->setSplashPath("");
    frame->setDisplayTitleFormat(title);
    frame->initialize(ros_node, config_path);
    return frame;
  }

  void setCubeMode(bool enabled)
  {
    stacked_->setCurrentIndex(enabled ? 0 : 1);
    if (toggle_button_) {
      toggle_button_->setText(enabled ? "Mode cubes" : "Mode points");
    }
  }

  QStackedWidget * stacked_{nullptr};
  QPushButton * toggle_button_{nullptr};
  rviz_common::VisualizationFrame * point_frame_{nullptr};
  rviz_common::VisualizationFrame * cube_frame_{nullptr};
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> point_ros_node_;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> cube_ros_node_;
};

class DashboardWindow : public QMainWindow
{
public:
  DashboardWindow(
    QApplication * app,
    const std::shared_ptr<DashboardNode> & dashboard_node,
    QWidget * parent = nullptr)
  : QMainWindow(parent), dashboard_node_(dashboard_node)
  {
    auto * central = new QWidget(this);
    auto * root = new QHBoxLayout(central);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(8);

    auto * splitter = new QSplitter(Qt::Horizontal, central);
    splitter->setChildrenCollapsible(false);

    auto * telemetry = new TelemetryPane(dashboard_node_, splitter);
    telemetry->setMinimumWidth(320);

    const QString share_dir = qs(ament_index_cpp::get_package_share_directory("dual_rviz_jazzy"));
    const QString cloud_points_cfg = share_dir + "/config/dashboard_3d.rviz";
    const QString cloud_cubes_cfg = share_dir + "/config/dashboard_3d_cubes.rviz";
    const QString explore_cfg = share_dir + "/config/dashboard_explore.rviz";

    auto * cloud_view = new RvizPane(
      app,
      "slam_dashboard_rviz_cloud",
      cloud_points_cfg,
      cloud_cubes_cfg,
      "3D Reconstruction",
      "Mode cubes",
      splitter);
    cloud_view->setMinimumWidth(720);

    auto * explore_view = new RvizPane(
      app,
      "slam_dashboard_rviz_explore",
      explore_cfg,
      "Map + Goals",
      splitter);
    explore_view->setMinimumWidth(720);

    splitter->addWidget(telemetry);
    splitter->addWidget(cloud_view);
    splitter->addWidget(explore_view);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setStretchFactor(2, 1);
    splitter->setSizes({360, 900, 900});

    root->addWidget(splitter, 1);
    setCentralWidget(central);
    setWindowTitle("SLAM Dashboard — cameras, 3D reconstruction, map, goals");
    resize(2100, 1200);

    statusBar()->showMessage("Dashboard ready");
  }

private:
  std::shared_ptr<DashboardNode> dashboard_node_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);

  auto dashboard_node = std::make_shared<DashboardNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dashboard_node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  DashboardWindow window(&app, dashboard_node);
  window.show();

  const int result = app.exec();

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return result;
}
