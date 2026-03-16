#include "sparo_navigation_core/door/door_plan_controller.hpp"

#include <tf2/exceptions.h>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <cctype>

DoorPlanController::DoorPlanController(const std::string& name,
                                       const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("bt_door_plan_controller");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  // Subscribe to odom for robot position
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      robot_pose_ = msg->pose.pose;
      have_robot_pose_ = true;
    });
}

BT::PortsList DoorPlanController::providedPorts()
{
  // Humble BT v3 호환: 기본값은 멤버로 관리, Ports는 존재만 선언
  return {
    BT::InputPort<std::string>("doors", "legacy: inline doors string"),
    BT::InputPort<std::string>("doors_yaml", "YAML path (preferred). If set, overrides 'doors'"),

    BT::InputPort<std::string>("plan_topic", "default: /plan"),
    BT::InputPort<std::string>("world_frame", "default: map"),
    BT::InputPort<std::string>("base_frame", "default: base_link"),

    BT::InputPort<bool>("require_feedback", "default: true"),
    BT::InputPort<double>("open_threshold", "default: 0.05"),
    BT::InputPort<double>("close_threshold", "default: 0.02"),
    BT::InputPort<double>("feedback_timeout_s", "default: 5.0"),
    BT::InputPort<double>("cooldown_s", "default: 1.0")
  };
}

void DoorPlanController::planCb(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_plan_ = msg;
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
    "[DoorPlanController] Received plan with %zu poses", msg->poses.size());
}

void DoorPlanController::doorPosCb(const std::string& ns,
                                   const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  auto& io = io_[ns];
  io.has_pos = true;

  double m = 0.0;
  for (auto v : msg->data) m = std::max(m, std::fabs(v));
  io.pos_abs_max = m;
  io.last_pos_time = node_->now();
}

static std::string trim_copy(std::string x)
{
  auto issp = [](int c){ return std::isspace(c); };
  x.erase(x.begin(), std::find_if(x.begin(), x.end(), [&](int c){ return !issp(c); }));
  x.erase(std::find_if(x.rbegin(), x.rend(), [&](int c){ return !issp(c); }).base(), x.end());
  return x;
}

std::string DoorPlanController::resolvePathMaybeInShare(const std::string& path) const
{
  if (path.empty()) return path;
  if (!path.empty() && path[0] == '/') return path;  // abs

  try
  {
    auto share = ament_index_cpp::get_package_share_directory("door_bt");
    return share + "/" + path;
  }
  catch (...)
  {
    return path; // 확실하지 않음: 설치/소스 환경 혼재 시
  }
}

bool DoorPlanController::loadDoorsFromYaml(const std::string& yaml_path,
                                           std::vector<DoorDef>& out,
                                           std::string& err)
{
  err.clear();
  out.clear();

  const std::string full = resolvePathMaybeInShare(yaml_path);

  try
  {
    YAML::Node root = YAML::LoadFile(full);
    if (!root["doors"])
    {
      err = "YAML missing key: doors";
      return false;
    }

    for (const auto& n : root["doors"])
    {
      if (!n["ns"] || !n["p1"] || !n["p2"] || !n["c"] || !n["radius"])
      {
        err = "YAML door entry missing fields (ns,p1,p2,c,radius)";
        return false;
      }

      DoorDef d;
      d.ns = n["ns"].as<std::string>();
      auto p1 = n["p1"]; auto p2 = n["p2"]; auto c = n["c"];

      d.p1x = p1[0].as<double>(); d.p1y = p1[1].as<double>();
      d.p2x = p2[0].as<double>(); d.p2y = p2[1].as<double>();
      d.cx  = c[0].as<double>();  d.cy  = c[1].as<double>();
      d.radius = n["radius"].as<double>();

      out.push_back(d);
    }

    if (out.empty())
    {
      err = "YAML doors list is empty";
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Loaded %zu doors from YAML: %s", out.size(), full.c_str());
    return true;
  }
  catch (const std::exception& e)
  {
    err = std::string("YAML exception: ") + e.what();
    return false;
  }
}

bool DoorPlanController::parseDoorsString(const std::string& s,
                                          std::vector<DoorDef>& out,
                                          std::string& err)
{
  out.clear();
  err.clear();

  std::stringstream ss(s);
  std::string item;

  while (std::getline(ss, item, ';'))
  {
    item = trim_copy(item);
    if (item.empty()) continue;

    std::vector<std::string> toks;
    std::stringstream ss2(item);
    std::string tok;
    while (std::getline(ss2, tok, ','))
      toks.push_back(trim_copy(tok));

    // ns + 7 numbers
    if (toks.size() != 8)
    {
      err = "doors format error: need 8 tokens (ns + 7 numbers)";
      return false;
    }

    DoorDef d;
    d.ns = toks[0];
    try
    {
      d.p1x = std::stod(toks[1]); d.p1y = std::stod(toks[2]);
      d.p2x = std::stod(toks[3]); d.p2y = std::stod(toks[4]);
      d.cx  = std::stod(toks[5]); d.cy  = std::stod(toks[6]);
      d.radius = std::stod(toks[7]);
    }
    catch (...)
    {
      err = "doors format error: numeric parse failed";
      return false;
    }

    out.push_back(d);
  }

  if (out.empty())
  {
    err = "doors is empty";
    return false;
  }
  return true;
}

void DoorPlanController::ensureDoorIO(const DoorDef& d)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (io_.count(d.ns)) return;

  DoorIO io;
  io.pub_cmd = node_->create_publisher<std_msgs::msg::Bool>(d.ns + "/door_open", 10);

  // ✅ door_pos 타입은 너가 확인해준 대로 Float64MultiArray “확정”
  io.sub_pos = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    d.ns + "/door_pos", rclcpp::QoS(10),
    [this, ns=d.ns](const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      this->doorPosCb(ns, msg);
    });

  io_[d.ns] = io;

  // runtime map도 미리 준비
  rt_.try_emplace(d.ns, DoorRuntime{});

  RCLCPP_INFO(node_->get_logger(),
    "Door IO ready: %s (open=%s, pos=%s)",
    d.ns.c_str(), (d.ns + "/door_open").c_str(), (d.ns + "/door_pos").c_str());
}

bool DoorPlanController::getRobotXY(double& x, double& y)
{
  using namespace std::chrono_literals;

  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!have_robot_pose_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[DoorPlanController] No odom received yet");
    return false;
  }

  x = robot_pose_.position.x;
  y = robot_pose_.position.y;
  return true;
}

bool DoorPlanController::inRadius(const DoorDef& d, double rx, double ry) const
{
  const double dx = rx - d.cx;
  const double dy = ry - d.cy;
  return (dx*dx + dy*dy) <= d.radius * d.radius;
}

int DoorPlanController::orientation(double ax, double ay, double bx, double by, double cx, double cy)
{
  double val = (by - ay) * (cx - bx) - (bx - ax) * (cy - by);
  const double eps = 1e-9;
  if (std::fabs(val) < eps) return 0;
  return (val > 0.0) ? 1 : 2;
}

bool DoorPlanController::onSegment(double ax, double ay, double bx, double by, double cx, double cy)
{
  const double eps = 1e-9;
  return (bx <= std::max(ax, cx) + eps &&
          bx + eps >= std::min(ax, cx) &&
          by <= std::max(ay, cy) + eps &&
          by + eps >= std::min(ay, cy));
}

bool DoorPlanController::segmentsIntersect(
  double p1x, double p1y, double p2x, double p2y,
  double q1x, double q1y, double q2x, double q2y)
{
  int o1 = orientation(p1x, p1y, p2x, p2y, q1x, q1y);
  int o2 = orientation(p1x, p1y, p2x, p2y, q2x, q2y);
  int o3 = orientation(q1x, q1y, q2x, q2y, p1x, p1y);
  int o4 = orientation(q1x, q1y, q2x, q2y, p2x, p2y);

  if (o1 != o2 && o3 != o4) return true;

  if (o1 == 0 && onSegment(p1x, p1y, q1x, q1y, p2x, p2y)) return true;
  if (o2 == 0 && onSegment(p1x, p1y, q2x, q2y, p2x, p2y)) return true;
  if (o3 == 0 && onSegment(q1x, q1y, p1x, p1y, q2x, q2y)) return true;
  if (o4 == 0 && onSegment(q1x, q1y, p2x, p2y, q2x, q2y)) return true;

  return false;
}

bool DoorPlanController::pathUsesDoor(const nav_msgs::msg::Path& plan, const DoorDef& d) const
{
  if (plan.poses.size() < 2) return false;
  for (size_t i = 0; i + 1 < plan.poses.size(); ++i)
  {
    const auto& a = plan.poses[i].pose.position;
    const auto& b = plan.poses[i+1].pose.position;
    if (segmentsIntersect(a.x, a.y, b.x, b.y, d.p1x, d.p1y, d.p2x, d.p2y))
      return true;
  }
  return false;
}

bool DoorPlanController::cooldownPassed(const std::string& ns) const
{
  auto it = done_time_.find(ns);
  if (it == done_time_.end()) return true;
  return (node_->now() - it->second).seconds() >= cooldown_s_;
}

void DoorPlanController::publishCmd(const std::string& ns, bool open)
{
  // 문별로 마지막 커맨드 기억해서 스팸 방지
  auto& rt = rt_[ns];
  if (rt.has_last_cmd && rt.last_cmd_open == open)
    return;

  std_msgs::msg::Bool msg;
  msg.data = open;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = io_.find(ns);
    if (it == io_.end() || !it->second.pub_cmd)
    {
      RCLCPP_ERROR(node_->get_logger(), "publisher missing ns=[%s]", ns.c_str());
      return;
    }
    it->second.pub_cmd->publish(msg);
  }

  rt.has_last_cmd = true;
  rt.last_cmd_open = open;
  rt.last_cmd_time = node_->now();

  RCLCPP_INFO(node_->get_logger(), "%s -> %s", ns.c_str(), open ? "OPEN" : "CLOSE");
}

bool DoorPlanController::doorOpened(const DoorDef& d)
{
  std::lock_guard<std::mutex> lk(mtx_);
  auto it = io_.find(d.ns);
  if (it == io_.end() || !it->second.has_pos) return false;
  return it->second.pos_abs_max >= open_threshold_;
}

bool DoorPlanController::doorClosed(const DoorDef& d)
{
  std::lock_guard<std::mutex> lk(mtx_);
  auto it = io_.find(d.ns);
  if (it == io_.end() || !it->second.has_pos) return false;
  return it->second.pos_abs_max <= close_threshold_;
}

BT::NodeStatus DoorPlanController::onStart()
{
  (void)getInput("plan_topic", plan_topic_);
  (void)getInput("world_frame", world_frame_);
  (void)getInput("base_frame", base_frame_);
  (void)getInput("require_feedback", require_feedback_);
  (void)getInput("open_threshold", open_threshold_);
  (void)getInput("close_threshold", close_threshold_);
  (void)getInput("feedback_timeout_s", feedback_timeout_s_);
  (void)getInput("cooldown_s", cooldown_s_);
  (void)getInput("doors_yaml", doors_yaml_);

  std::string err;

  if (!doors_yaml_.empty())
  {
    if (!loadDoorsFromYaml(doors_yaml_, doors_, err))
    {
      RCLCPP_ERROR(node_->get_logger(), "failed to load doors_yaml: %s", err.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    std::string doors_str;
    if (!getInput("doors", doors_str))
    {
      RCLCPP_ERROR(node_->get_logger(), "missing [doors] (or set doors_yaml)");
      return BT::NodeStatus::FAILURE;
    }
    if (!parseDoorsString(doors_str, doors_, err))
    {
      RCLCPP_ERROR(node_->get_logger(), "doors parse error: %s", err.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  sub_plan_ = node_->create_subscription<nav_msgs::msg::Path>(
    plan_topic_, rclcpp::QoS(1),
    std::bind(&DoorPlanController::planCb, this, std::placeholders::_1));

  // Subscribe to config_yaml topic for runtime YAML reload
  sub_config_yaml_ = node_->create_subscription<std_msgs::msg::String>(
    "/door_controller/config_yaml", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      std::string err;
      std::vector<DoorDef> new_doors;
      if (loadDoorsFromYaml(msg->data, new_doors, err)) {
        doors_ = new_doors;
        rt_.clear();
        for (const auto& d : doors_) ensureDoorIO(d);
        processed_in_plan_.clear();
        RCLCPP_INFO(node_->get_logger(), "[DoorPlanController] Reloaded %zu doors from %s", doors_.size(), msg->data.c_str());
      } else {
        RCLCPP_ERROR(node_->get_logger(), "[DoorPlanController] Failed to reload YAML: %s", err.c_str());
      }
    });

  for (const auto& d : doors_) ensureDoorIO(d);

  processed_in_plan_.clear();
  last_plan_stamp_ = rclcpp::Time(0,0,node_->get_clock()->get_clock_type());
  last_plan_size_ = 0;

  RCLCPP_INFO(node_->get_logger(),
    "DoorPlanController started (CONCURRENT): plan=%s world=%s base=%s doors=%zu",
    plan_topic_.c_str(), world_frame_.c_str(), base_frame_.c_str(), doors_.size());

  return BT::NodeStatus::RUNNING;
}

  using namespace std::chrono_literals;
BT::NodeStatus DoorPlanController::onRunning()
{
  exec_->spin_some(0ms);
  using namespace std::chrono_literals;

  nav_msgs::msg::Path::SharedPtr plan;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    plan = last_plan_;
  }

  double rx=0, ry=0;
  if (!getRobotXY(rx, ry)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[DoorPlanController] Waiting for robot position...");
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
    "[DoorPlanController] Robot (%.2f,%.2f) plan=%s", rx, ry, (plan?"yes":"no"));

  // plan 갱신 감지 → "이 plan에서 처리 완료" 목록만 초기화
  if (plan && plan->poses.size() >= 2)
  {
    if (plan->header.stamp != last_plan_stamp_ || plan->poses.size() != last_plan_size_)
    {
      processed_in_plan_.clear();
      last_plan_stamp_ = plan->header.stamp;
      last_plan_size_ = plan->poses.size();
    }
  }

  // ✅ 핵심: 모든 문을 매 tick 평가 (동시 open/close)
  for (const auto& d : doors_)
  {
    auto& rt = rt_[d.ns];

    const bool inside = inRadius(d, rx, ry);

    if (inside) RCLCPP_INFO(node_->get_logger(), "[Door] DETECTED %s: dist=%.2f", d.ns.c_str(), std::hypot(rx-d.cx, ry-d.cy));
    // 1) 이미 opened/opening 상태인 문이 반경 밖으로 나가면 → close 시도
    if ((rt.opened || rt.opening) && !inside)
    {
      if (!rt.closing)
      {
        // close 시작
        rt.closing = true;
        rt.opening = false;  // open 확인 중이었어도 종료
        rt.opened = true;    // close 완료 전까지는 열린 상태로 간주
        rt.has_last_cmd = false; // close publish 허용
      }

      if (!require_feedback_)
      {
        publishCmd(d.ns, false);
        rt.opened = false;
        rt.closing = false;
        processed_in_plan_.insert(d.ns);
        done_time_[d.ns] = node_->now();
        continue;
      }

      // 피드백 기반 close
      if (doorClosed(d))
      {
        rt.opened = false;
        rt.closing = false;
        processed_in_plan_.insert(d.ns);
        done_time_[d.ns] = node_->now();
        continue;
      }

      // 아직 닫힘 확인 안 됐으면 close publish (스팸은 rt.has_last_cmd로 방지)
      publishCmd(d.ns, false);

      // timeout이면 “닫힘으로 간주” (문이 실제로 닫히는지 보장은 확실하지 않음)
      if ((node_->now() - rt.last_cmd_time).seconds() > feedback_timeout_s_)
      {
        rt.opened = false;
        rt.closing = false;
        processed_in_plan_.insert(d.ns);
        done_time_[d.ns] = node_->now();
      }

      continue;
    }

    // 2) 아직 처리 안 한 문 + plan에 포함 + 반경 안이면 → open 시도
    //    (연속 2개 문이면 여기서 두 번째도 즉시 열림)
    if (inside)
    {
      if (processed_in_plan_.count(d.ns)) continue;
      if (!cooldownPassed(d.ns)) continue;

      // plan이 없거나 plan에 안 들어있으면 "열지 않음"
      if (!plan || plan->poses.size() < 2) continue;
      if (!pathUsesDoor(*plan, d)) continue;

      // open 시작/유지
      if (!require_feedback_)
      {
        if (!rt.opened)
          publishCmd(d.ns, true);
        rt.opened = true;
        rt.opening = false;
        rt.closing = false;
        continue;
      }

      // 피드백 기반 open
      if (doorOpened(d))
      {
        rt.opened = true;
        rt.opening = false;
        rt.closing = false;
        continue;
      }

      // 아직 open 확인 안 됐으면 open publish
      if (!rt.opened && !rt.opening)
      {
        rt.opening = true;
        rt.closing = false;
        rt.has_last_cmd = false; // open publish 허용
      }

      publishCmd(d.ns, true);

      // timeout이면 “열림으로 간주”
      if ((node_->now() - rt.last_cmd_time).seconds() > feedback_timeout_s_)
      {
        rt.opened = true;
        rt.opening = false;
      }

      continue;
    }

    // 3) inside=false이고 opened/opening도 아니면 아무것도 안 함
  }

  return BT::NodeStatus::RUNNING;
}

void DoorPlanController::onHalted()
{
}
