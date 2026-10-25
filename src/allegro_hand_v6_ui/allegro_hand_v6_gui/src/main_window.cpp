#include "allegro_hand_v6_gui/main_window.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <unordered_map>

#include <QCloseEvent>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QResizeEvent>
#include <QScreen>
#include <QScrollArea>
#include <QSpinBox>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <yaml-cpp/yaml.h>

namespace allegro_hand_v6_gui {

namespace {
constexpr int kPosePathRole = Qt::UserRole + 1;

}

MainWindow::MainWindow(const std::shared_ptr<rclcpp::Node> &node,
                       QWidget *parent)
    : QMainWindow(parent), node_(node) {
  const std::string joint_state_topic = node_->declare_parameter<std::string>(
      "joint_state_topic", "/joint_states");
  const std::string position_topic = node_->declare_parameter<std::string>(
      "position_command_topic", "/allegro_hand_position_controller/commands");
  const std::string servo_topic = node_->declare_parameter<std::string>(
      "servo_command_topic", "/allegro_hand_servo_controller/commands");
  const std::string configured_pose_directory =
      node_->declare_parameter<std::string>("pose_directory", "");

  pose_directory_ = configured_pose_directory.empty()
                        ? QString::fromStdString(
                              ament_index_cpp::get_package_share_directory("allegro_hand_v6_gui") + "/poses")
                        : QString::fromStdString(configured_pose_directory);
  QDir().mkpath(pose_directory_);

  position_pub_ =
      node_->create_publisher<std_msgs::msg::Float64MultiArray>(position_topic, 10);
  servo_pub_ =
      node_->create_publisher<std_msgs::msg::Float64MultiArray>(servo_topic, 10);
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, rclcpp::SensorDataQoS(),
      std::bind(&MainWindow::jointStateCallback, this, std::placeholders::_1));

  buildUi();
  refreshPoseList();
  appendLog("Pose directory: " + pose_directory_);
}

const std::array<std::string, MainWindow::kJointCount> &
MainWindow::jointNames() {
  static const std::array<std::string, kJointCount> names = {
      "joint00", "joint01", "joint02", "joint03", "joint10",
      "joint11", "joint12", "joint13", "joint20", "joint21",
      "joint22", "joint23", "joint30", "joint31", "joint32",
      "joint33", "joint40", "joint41", "joint42", "joint43"};
  return names;
}

void MainWindow::buildUi() {
  setWindowTitle("Allegro Hand V6 Pose Control");
  setMinimumSize(480, 400);

  auto *scroll = new QScrollArea(this);
  scroll->setObjectName("poseControlScroll");
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  auto *central = new QWidget;
  auto *root_layout = new QVBoxLayout(central);

  auto *servo_group = new QGroupBox("Servo Control", central);
  auto *servo_layout = new QGridLayout(servo_group);
  auto *servo_on_button = new QPushButton("Servo ON", servo_group);
  auto *servo_off_button = new QPushButton("Servo OFF", servo_group);
  servo_on_button->setMinimumHeight(42);
  servo_off_button->setMinimumHeight(42);
  servo_status_label_ = new QLabel("Requested: OFF", servo_group);
  joint_state_label_ = new QLabel("Joint state: waiting", servo_group);
  servo_layout->addWidget(servo_on_button, 0, 0);
  servo_layout->addWidget(servo_off_button, 0, 1);
  servo_layout->addWidget(servo_status_label_, 1, 0);
  servo_layout->addWidget(joint_state_label_, 1, 1);
  root_layout->addWidget(servo_group);

  auto *splitter = new QSplitter(Qt::Horizontal, central);
  content_splitter_ = splitter;
  splitter->setObjectName("poseControlPanels");
  splitter->setChildrenCollapsible(false);

  auto *pose_group = new QGroupBox("Saved Poses", splitter);
  auto *pose_layout = new QVBoxLayout(pose_group);
  auto *save_layout = new QHBoxLayout();
  filename_edit_ = new QLineEdit(pose_group);
  filename_edit_->setPlaceholderText("pose_name");
  auto *save_button = new QPushButton("Save Current", pose_group);
  save_layout->addWidget(filename_edit_);
  save_layout->addWidget(save_button);
  pose_layout->addLayout(save_layout);

  pose_list_ = new QListWidget(pose_group);
  pose_list_->setSelectionMode(QAbstractItemView::SingleSelection);
  pose_layout->addWidget(pose_list_);

  auto *pose_button_layout = new QHBoxLayout();
  auto *refresh_button = new QPushButton("Refresh", pose_group);
  auto *move_button = new QPushButton("Move", pose_group);
  auto *add_button = new QPushButton("Add to Sequence", pose_group);
  pose_button_layout->addWidget(refresh_button);
  pose_button_layout->addWidget(move_button);
  pose_button_layout->addWidget(add_button);
  pose_layout->addLayout(pose_button_layout);

  auto *sequence_group = new QGroupBox("Pose Sequence", splitter);
  auto *sequence_layout = new QVBoxLayout(sequence_group);
  sequence_list_ = new QListWidget(sequence_group);
  sequence_layout->addWidget(sequence_list_);

  auto *edit_sequence_layout = new QHBoxLayout();
  auto *remove_button = new QPushButton("Remove", sequence_group);
  auto *up_button = new QPushButton("Up", sequence_group);
  auto *down_button = new QPushButton("Down", sequence_group);
  auto *clear_button = new QPushButton("Clear", sequence_group);
  edit_sequence_layout->addWidget(remove_button);
  edit_sequence_layout->addWidget(up_button);
  edit_sequence_layout->addWidget(down_button);
  edit_sequence_layout->addWidget(clear_button);
  sequence_layout->addLayout(edit_sequence_layout);

  auto *sequence_options = new QFormLayout();
  interval_spin_ = new QDoubleSpinBox(sequence_group);
  interval_spin_->setRange(0.1, 60.0);
  interval_spin_->setDecimals(1);
  interval_spin_->setSingleStep(0.1);
  interval_spin_->setValue(2.0);
  interval_spin_->setSuffix(" s");
  repeat_spin_ = new QSpinBox(sequence_group);
  repeat_spin_->setRange(1, 999);
  repeat_spin_->setValue(1);
  sequence_options->addRow("Pose interval", interval_spin_);
  sequence_options->addRow("Repeat count", repeat_spin_);
  sequence_layout->addLayout(sequence_options);

  auto *run_layout = new QHBoxLayout();
  start_button_ = new QPushButton("Start", sequence_group);
  stop_button_ = new QPushButton("Stop", sequence_group);
  stop_button_->setEnabled(false);
  run_layout->addWidget(start_button_);
  run_layout->addWidget(stop_button_);
  sequence_layout->addLayout(run_layout);

  splitter->addWidget(pose_group);
  splitter->addWidget(sequence_group);
  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 1);
  root_layout->addWidget(splitter, 1);

  auto *log_group = new QGroupBox("Log", central);
  auto *log_layout = new QVBoxLayout(log_group);
  log_edit_ = new QPlainTextEdit(log_group);
  log_edit_->setReadOnly(true);
  log_edit_->setMaximumBlockCount(500);
  log_edit_->setMinimumHeight(80);
  log_edit_->setMaximumHeight(160);
  auto *clear_log_button = new QPushButton("Clear Log", log_group);
  log_layout->addWidget(log_edit_);
  log_layout->addWidget(clear_log_button, 0, Qt::AlignRight);
  root_layout->addWidget(log_group);

  scroll->setWidget(central);
  setCentralWidget(scroll);
  QSize initial(940, 680);
  if (const auto *screen = QGuiApplication::primaryScreen()) {
    initial = initial.boundedTo(screen->availableGeometry().size() * 0.92);
  }
  resize(initial.expandedTo(minimumSize()));
  splitter->setOrientation(width() < 880 ? Qt::Vertical : Qt::Horizontal);

  sequence_timer_ = new QTimer(this);
  sequence_timer_->setSingleShot(true);

  connect(servo_on_button, &QPushButton::clicked, this, &MainWindow::servoOn);
  connect(servo_off_button, &QPushButton::clicked, this, &MainWindow::servoOff);
  connect(save_button, &QPushButton::clicked, this, &MainWindow::savePose);
  connect(filename_edit_, &QLineEdit::returnPressed, this, &MainWindow::savePose);
  connect(refresh_button, &QPushButton::clicked, this,
          &MainWindow::refreshPoseList);
  connect(move_button, &QPushButton::clicked, this,
          &MainWindow::moveSelectedPose);
  connect(pose_list_, &QListWidget::itemDoubleClicked, this,
          [this](QListWidgetItem *) { moveSelectedPose(); });
  connect(add_button, &QPushButton::clicked, this, &MainWindow::addSelectedPose);
  connect(remove_button, &QPushButton::clicked, this,
          &MainWindow::removeSequencePose);
  connect(up_button, &QPushButton::clicked, this,
          &MainWindow::moveSequencePoseUp);
  connect(down_button, &QPushButton::clicked, this,
          &MainWindow::moveSequencePoseDown);
  connect(clear_button, &QPushButton::clicked, this, &MainWindow::clearSequence);
  connect(start_button_, &QPushButton::clicked, this, &MainWindow::startSequence);
  connect(stop_button_, &QPushButton::clicked, this, &MainWindow::stopSequence);
  connect(sequence_timer_, &QTimer::timeout, this, &MainWindow::executeNextPose);
  connect(clear_log_button, &QPushButton::clicked, log_edit_,
          &QPlainTextEdit::clear);
}

void MainWindow::resizeEvent(QResizeEvent *event) {
  QMainWindow::resizeEvent(event);
  if (content_splitter_) {
    content_splitter_->setOrientation(width() < 880 ? Qt::Vertical : Qt::Horizontal);
  }
}

void MainWindow::appendLog(const QString &text) {
  log_edit_->appendPlainText(
      QString("[%1] %2").arg(QDateTime::currentDateTime().toString("HH:mm:ss"),
                              text));
}

void MainWindow::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.size() < kJointCount) {
    return;
  }

  if (msg->name.empty()) {
    std::copy_n(msg->position.begin(), kJointCount, latest_positions_.begin());
  } else {
    std::unordered_map<std::string, double> by_name;
    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      by_name[msg->name[i]] = msg->position[i];
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto found = by_name.find(jointNames()[i]);
      if (found == by_name.end()) {
        return;
      }
      latest_positions_[i] = found->second;
    }
  }

  if (!std::all_of(latest_positions_.begin(), latest_positions_.end(),
                   [](double value) { return std::isfinite(value); })) {
    return;
  }

  latest_state_stamp_ = node_->now();
  if (!have_joint_state_) {
    appendLog("Received first complete 20-joint state.");
  }
  have_joint_state_ = true;
  joint_state_label_->setText("Joint state: ready (20/20)");
}

void MainWindow::savePose() {
  const QString pose_name = filename_edit_->text().trimmed();
  const QRegularExpression valid_name("^[A-Za-z0-9_-]+$");
  if (!valid_name.match(pose_name).hasMatch()) {
    QMessageBox::warning(this, "Invalid pose name",
                         "Use only letters, numbers, '_' and '-'.");
    return;
  }
  if (!have_joint_state_) {
    QMessageBox::warning(this, "No joint state",
                         "A complete 20-joint state has not been received.");
    return;
  }
  if ((node_->now() - latest_state_stamp_).seconds() > 2.0) {
    QMessageBox::warning(this, "Stale joint state",
                         "The newest joint state is more than 2 seconds old.");
    return;
  }

  QDir directory(pose_directory_);
  if (!directory.exists() && !QDir().mkpath(pose_directory_)) {
    QMessageBox::critical(this, "Save failed",
                          "Could not create pose directory:\n" + pose_directory_);
    return;
  }
  const QString path = directory.filePath(pose_name + ".yaml");
  if (QFileInfo::exists(path) &&
      QMessageBox::question(this, "Replace pose",
                            pose_name + ".yaml already exists. Replace it?") !=
          QMessageBox::Yes) {
    return;
  }

  YAML::Emitter output;
  output << YAML::BeginMap;
  output << YAML::Key << "joint_names" << YAML::Value << YAML::BeginSeq;
  for (const auto &name : jointNames()) {
    output << name;
  }
  output << YAML::EndSeq;
  output << YAML::Key << "position" << YAML::Value << YAML::Flow
         << std::vector<double>(latest_positions_.begin(), latest_positions_.end());
  output << YAML::EndMap;

  std::ofstream file(path.toStdString(), std::ios::out | std::ios::trunc);
  if (!file) {
    QMessageBox::critical(this, "Save failed", "Could not write:\n" + path);
    return;
  }
  file << output.c_str() << '\n';
  file.close();

  appendLog("Saved current pose: " + pose_name);
  filename_edit_->clear();
  refreshPoseList();
}

bool MainWindow::loadPose(const QString &path,
                          std::array<double, kJointCount> &positions,
                          QString &error) const {
  try {
    const YAML::Node root = YAML::LoadFile(path.toStdString());
    if (!root["position"] || !root["position"].IsSequence() ||
        root["position"].size() != kJointCount) {
      error = "'position' must contain exactly 20 values.";
      return false;
    }

    const std::vector<double> stored =
        root["position"].as<std::vector<double>>();
    if (root["joint_names"]) {
      const std::vector<std::string> stored_names =
          root["joint_names"].as<std::vector<std::string>>();
      if (stored_names.size() != kJointCount) {
        error = "'joint_names' must contain exactly 20 names.";
        return false;
      }
      std::unordered_map<std::string, double> by_name;
      for (std::size_t i = 0; i < kJointCount; ++i) {
        by_name[stored_names[i]] = stored[i];
      }
      for (std::size_t i = 0; i < kJointCount; ++i) {
        const auto found = by_name.find(jointNames()[i]);
        if (found == by_name.end()) {
          error = QString("Missing joint '%1'.")
                      .arg(QString::fromStdString(jointNames()[i]));
          return false;
        }
        positions[i] = found->second;
      }
    } else {
      std::copy(stored.begin(), stored.end(), positions.begin());
    }

    if (!std::all_of(positions.begin(), positions.end(),
                     [](double value) { return std::isfinite(value); })) {
      error = "Pose contains NaN or infinity.";
      return false;
    }
    return true;
  } catch (const std::exception &exception) {
    error = QString::fromUtf8(exception.what());
    return false;
  }
}

bool MainWindow::publishPosePath(const QString &path) {
  std::array<double, kJointCount> positions{};
  QString error;
  if (!loadPose(path, positions, error)) {
    appendLog("Pose load failed: " + QFileInfo(path).fileName() + " — " + error);
    QMessageBox::warning(this, "Invalid pose", error);
    return false;
  }

  std_msgs::msg::Float64MultiArray command;
  command.data.assign(positions.begin(), positions.end());
  position_pub_->publish(command);
  appendLog("Commanded pose: " + QFileInfo(path).completeBaseName());
  return true;
}

QString MainWindow::selectedPosePath() const {
  const QListWidgetItem *item = pose_list_->currentItem();
  return item == nullptr ? QString() : item->data(kPosePathRole).toString();
}

void MainWindow::refreshPoseList() {
  pose_list_->clear();
  QDir directory(pose_directory_);
  const QFileInfoList files = directory.entryInfoList(
      QStringList() << "*.yaml" << "*.yml", QDir::Files, QDir::Name);
  for (const QFileInfo &file : files) {
    auto *item = new QListWidgetItem(file.completeBaseName(), pose_list_);
    item->setData(kPosePathRole, file.absoluteFilePath());
    item->setToolTip(file.absoluteFilePath());
  }
}

void MainWindow::moveSelectedPose() {
  const QString path = selectedPosePath();
  if (path.isEmpty()) {
    appendLog("Move ignored: no pose selected.");
    return;
  }
  (void)publishPosePath(path);
}

void MainWindow::addSelectedPose() {
  const QListWidgetItem *selected = pose_list_->currentItem();
  if (selected == nullptr) {
    appendLog("Add ignored: no pose selected.");
    return;
  }
  if (sequence_list_->count() > 0 &&
      sequence_list_->item(sequence_list_->count() - 1)
              ->data(kPosePathRole) == selected->data(kPosePathRole)) {
    appendLog("Consecutive duplicate pose ignored.");
    return;
  }
  auto *item = new QListWidgetItem(selected->text(), sequence_list_);
  item->setData(kPosePathRole, selected->data(kPosePathRole));
}

void MainWindow::removeSequencePose() {
  delete sequence_list_->takeItem(sequence_list_->currentRow());
}

void MainWindow::moveSequencePoseUp() {
  const int row = sequence_list_->currentRow();
  if (row <= 0) {
    return;
  }
  QListWidgetItem *item = sequence_list_->takeItem(row);
  sequence_list_->insertItem(row - 1, item);
  sequence_list_->setCurrentRow(row - 1);
}

void MainWindow::moveSequencePoseDown() {
  const int row = sequence_list_->currentRow();
  if (row < 0 || row >= sequence_list_->count() - 1) {
    return;
  }
  QListWidgetItem *item = sequence_list_->takeItem(row);
  sequence_list_->insertItem(row + 1, item);
  sequence_list_->setCurrentRow(row + 1);
}

void MainWindow::clearSequence() { sequence_list_->clear(); }

void MainWindow::startSequence() {
  if (sequence_list_->count() == 0) {
    appendLog("Sequence start ignored: sequence is empty.");
    return;
  }
  sequence_index_ = 0;
  completed_cycles_ = 0;
  requested_cycles_ = repeat_spin_->value();
  active_sequence_paths_.clear();
  for (int row = 0; row < sequence_list_->count(); ++row) {
    active_sequence_paths_.append(
        sequence_list_->item(row)->data(kPosePathRole).toString());
  }
  sequence_running_ = true;
  start_button_->setEnabled(false);
  stop_button_->setEnabled(true);
  appendLog(QString("Sequence started: %1 poses × %2")
                .arg(sequence_list_->count())
                .arg(requested_cycles_));
  executeNextPose();
}

void MainWindow::stopSequence() {
  if (!sequence_running_) {
    return;
  }
  sequence_timer_->stop();
  sequence_running_ = false;
  active_sequence_paths_.clear();
  start_button_->setEnabled(true);
  stop_button_->setEnabled(false);
  appendLog("Sequence stopped.");
}

void MainWindow::executeNextPose() {
  if (!sequence_running_) {
    return;
  }
  if (sequence_index_ >= active_sequence_paths_.size()) {
    ++completed_cycles_;
    if (completed_cycles_ >= requested_cycles_) {
      sequence_running_ = false;
      active_sequence_paths_.clear();
      start_button_->setEnabled(true);
      stop_button_->setEnabled(false);
      appendLog("Sequence completed.");
      return;
    }
    sequence_index_ = 0;
  }

  const QString path = active_sequence_paths_[sequence_index_];
  if (!publishPosePath(path)) {
    stopSequence();
    return;
  }
  ++sequence_index_;
  sequence_timer_->start(static_cast<int>(interval_spin_->value() * 1000.0));
}

void MainWindow::publishServo(bool enabled) {
  std_msgs::msg::Float64MultiArray command;
  command.data = {enabled ? 1.0 : 0.0};
  servo_pub_->publish(command);
  servo_requested_ = enabled;
  servo_status_label_->setText(enabled ? "Requested: ON" : "Requested: OFF");
  appendLog(enabled ? "Servo ON requested." : "Servo OFF requested.");
}

void MainWindow::servoOn() { publishServo(true); }

void MainWindow::servoOff() {
  stopSequence();
  publishServo(false);
}

void MainWindow::closeEvent(QCloseEvent *event) {
  sequence_timer_->stop();
  publishServo(false);
  event->accept();
}

} // namespace allegro_hand_v6_gui
