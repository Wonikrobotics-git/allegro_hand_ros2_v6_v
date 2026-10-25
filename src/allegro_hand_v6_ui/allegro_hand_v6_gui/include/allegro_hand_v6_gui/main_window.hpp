#ifndef ALLEGRO_HAND_V6_GUI__MAIN_WINDOW_HPP_
#define ALLEGRO_HAND_V6_GUI__MAIN_WINDOW_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <QMainWindow>
#include <QStringList>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class QCloseEvent;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTimer;
class QResizeEvent;
class QSplitter;

namespace allegro_hand_v6_gui {

class MainWindow final : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(const std::shared_ptr<rclcpp::Node> &node,
                      QWidget *parent = nullptr);
  ~MainWindow() override = default;

protected:
  void closeEvent(QCloseEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;

private slots:
  void savePose();
  void moveSelectedPose();
  void refreshPoseList();
  void addSelectedPose();
  void removeSequencePose();
  void moveSequencePoseUp();
  void moveSequencePoseDown();
  void clearSequence();
  void startSequence();
  void stopSequence();
  void executeNextPose();
  void servoOn();
  void servoOff();

private:
  static constexpr std::size_t kJointCount = 20;

  void buildUi();
  void appendLog(const QString &text);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool loadPose(const QString &path, std::array<double, kJointCount> &positions,
                QString &error) const;
  bool publishPosePath(const QString &path);
  void publishServo(bool enabled);
  QString selectedPosePath() const;
  static const std::array<std::string, kJointCount> &jointNames();

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_pub_;

  std::array<double, kJointCount> latest_positions_{};
  rclcpp::Time latest_state_stamp_{0, 0, RCL_ROS_TIME};
  bool have_joint_state_{false};
  bool servo_requested_{false};
  bool sequence_running_{false};
  int sequence_index_{0};
  int completed_cycles_{0};
  int requested_cycles_{1};
  QStringList active_sequence_paths_;

  QString pose_directory_;
  QTimer *sequence_timer_{nullptr};
  QLabel *joint_state_label_{nullptr};
  QLabel *servo_status_label_{nullptr};
  QLineEdit *filename_edit_{nullptr};
  QListWidget *pose_list_{nullptr};
  QListWidget *sequence_list_{nullptr};
  QDoubleSpinBox *interval_spin_{nullptr};
  QSpinBox *repeat_spin_{nullptr};
  QPushButton *start_button_{nullptr};
  QPushButton *stop_button_{nullptr};
  QPlainTextEdit *log_edit_{nullptr};
  QSplitter *content_splitter_{nullptr};
};

} // namespace allegro_hand_v6_gui

#endif // ALLEGRO_HAND_V6_GUI__MAIN_WINDOW_HPP_
