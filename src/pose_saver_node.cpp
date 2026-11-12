#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <curl/curl.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "pose_saver_pkg/cloud_db_client.hpp"
#include "pose_saver_pkg/pose_logic.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;
using SteadyClock = std::chrono::steady_clock;
using PoseStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

enum class State { WAIT_READY, LOAD_POSE, CONFIRM, GLOBAL_LOCALIZE, LOCALIZED, FAILED };

static const char* toStr(State s) {
  switch (s) {
    case State::WAIT_READY: return "WAIT_READY";
    case State::LOAD_POSE: return "LOAD_POSE";
    case State::CONFIRM: return "CONFIRM";
    case State::GLOBAL_LOCALIZE: return "GLOBAL_LOCALIZE";
    case State::LOCALIZED: return "LOCALIZED";
    case State::FAILED: return "FAILED";
  }
  return "?";
}

static double unixNow() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

struct LoadResult {
  bool found{false};
  PoseRecord rec;
  std::string source;
  std::string reason{"no_record"};
};

struct Pose2D { double x, y, yaw; };

class PoseSaverNode : public rclcpp::Node {
public:
  PoseSaverNode() : Node("pose_saver_node") {
    db_url_ = declare_parameter<std::string>("db_url", "");
    auth_token_ = declare_parameter<std::string>("auth_token", "");
    if (const char* e = std::getenv("POSE_DB_URL")) if (*e) db_url_ = e;
    if (auth_token_.empty()) if (const char* e = std::getenv("POSE_DB_TOKEN")) auth_token_ = e;
    robot_id_ = declare_parameter<std::string>("robot_id", "turtlebot3_burger_01");
    map_id_ = declare_parameter<std::string>("map_id", "turtlebot3_world");
    use_cache_ = declare_parameter<bool>("use_local_cache", true);
    std::string cd = declare_parameter<std::string>("cache_dir", "");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    const std::string cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    use_stamped_ = declare_parameter<bool>("use_stamped_cmd_vel", true);
    const double save_interval = declare_parameter<double>("save_interval_sec", 2.0);
    save_min_delta_ = declare_parameter<double>("save_min_delta_m", 0.02);
    save_heartbeat_ = declare_parameter<double>("save_heartbeat_sec", 30.0);
    max_age_ = declare_parameter<double>("max_pose_age_sec", 604800.0);
    match_thr_ = declare_parameter<double>("scan_match_threshold", 0.60);
    nbr_ = static_cast<int>(declare_parameter<int>("scan_hit_neighbor_cells", 2));
    min_beams_ = static_cast<int>(declare_parameter<int>("scan_min_beams", 30));
    stride_ = static_cast<int>(declare_parameter<int>("scan_stride", 2));
    max_sigma_ = declare_parameter<double>("max_sigma_m", 0.25);
    max_drift_ = declare_parameter<double>("max_drift_m", 0.30);
    max_drift_yaw_ = declare_parameter<double>("max_drift_yaw_rad", 0.40);
    init_cov_xy_ = declare_parameter<double>("initial_cov_xy", 0.01);
    init_cov_yaw_ = declare_parameter<double>("initial_cov_yaw", 0.01);
    startup_timeout_ = declare_parameter<double>("startup_timeout_sec", 60.0);
    confirm_timeout_ = declare_parameter<double>("confirm_timeout_sec", 10.0);
    global_timeout_ = declare_parameter<double>("global_timeout_sec", 90.0);
    hold_sec_ = declare_parameter<double>("global_hold_sec", 2.0);
    spin_speed_ = declare_parameter<double>("spin_speed_rad_s", 0.5);

    if (cd.empty()) {
      const char* h = std::getenv("HOME");
      cd = std::string(h ? h : "/tmp") + "/.ros/pose_saver";
    }
    cache_path_ = fs::path(cd) / (robot_id_ + ".json");
    std::error_code ec;
    fs::create_directories(cd, ec);

    db_ = std::make_unique<CloudDBClient>(db_url_, auth_token_);
    save_thread_ = std::thread(&PoseSaverNode::saveLoop, this);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", rclcpp::QoS(1).transient_local().reliable());
    initial_pose_pub_ = create_publisher<PoseStamped>("/initialpose", 10);
    if (use_stamped_)
      cmd_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_topic, 10);
    else
      cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { map_ = m; });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) { scan_ = m; });
    amcl_sub_ = create_subscription<PoseStamped>("/amcl_pose", 10, [this](PoseStamped::ConstSharedPtr m) {
      amcl_ = *m;
      ++amcl_count_;
    });

    global_loc_client_ = create_client<std_srvs::srv::Empty>("/reinitialize_global_localization");
    nomotion_client_ = create_client<std_srvs::srv::Empty>("/request_nomotion_update");

    tick_timer_ = create_wall_timer(100ms, [this] { tick(); });
    save_timer_ = create_wall_timer(std::chrono::duration<double>(save_interval), [this] { saveTick(); });

    setState(State::WAIT_READY, "waiting for clock, map, scan, TF and AMCL");
  }

  ~PoseSaverNode() override {
    if (state_ == State::LOCALIZED) maybeSave(true);  // flush last good pose
    {
      std::lock_guard<std::mutex> l(save_mx_);
      stop_ = true;
    }
    save_cv_.notify_all();
    if (save_thread_.joinable()) save_thread_.join();
  }

private:
  // ---------------- state machine ----------------
  void tick() {
    switch (state_) {
      case State::WAIT_READY: tickWaitReady(); break;
      case State::LOAD_POSE: tickLoad(); break;
      case State::CONFIRM: tickConfirm(); break;
      case State::GLOBAL_LOCALIZE: tickGlobal(); break;
      case State::LOCALIZED:
      case State::FAILED: break;
    }
  }

  void tickWaitReady() {
    const bool ready = now().nanoseconds() > 0 && map_ && scan_ &&
                       global_loc_client_->service_is_ready() &&
                       initial_pose_pub_->get_subscription_count() > 0 &&
                       tf_buffer_->canTransform(base_frame_, scan_->header.frame_id, tf2::TimePointZero);
    if (ready) {
      load_future_ = std::async(std::launch::async, [this] { return loadBest(); });
      setState(State::LOAD_POSE, "reading saved pose");
      return;
    }
    if (elapsed() > startup_timeout_)
      fail("startup_timeout: clock/map/scan/TF/AMCL not available");
  }

  void tickLoad() {
    if (load_future_.wait_for(0s) != std::future_status::ready) return;
    if (elapsed() < 1.5) return;  // Wait 1.5s for scan/TF/physics to stabilize after spawn
    evaluateSaved(load_future_.get());
  }

  void evaluateSaved(const LoadResult& r) {
    if (!r.found) {
      scenario_ = "3_NO_USABLE_POSE";
      startGlobal("no saved pose (" + r.reason + ")");
      return;
    }
    const PoseRecord& rec = r.rec;
    if (!isValidPose(rec)) {
      scenario_ = "3_NO_USABLE_POSE";
      startGlobal("saved pose invalid (NaN or bad quaternion)");
      return;
    }
    if (!rec.map_id.empty() && rec.map_id != map_id_) {
      scenario_ = "3_NO_USABLE_POSE";
      startGlobal("saved pose belongs to map '" + rec.map_id + "', current map is '" + map_id_ + "'");
      return;
    }
    const double age = unixNow() - rec.stamp_unix;
    if (max_age_ > 0.0 && age > max_age_) {
      scenario_ = "3_NO_USABLE_POSE";
      startGlobal("saved pose is stale (" + std::to_string(static_cast<long>(age)) + " s old)");
      return;
    }
    const Pose2D p{rec.x, rec.y, yawFromQuat(rec.qx, rec.qy, rec.qz, rec.qw)};
    const double score = scoreAtPose(p);
    RCLCPP_INFO(get_logger(), "Saved pose from %s: x=%.2f y=%.2f yaw=%.2f | scan match=%.2f (need %.2f)",
                r.source.c_str(), p.x, p.y, p.yaw, score, match_thr_);
    if (score < 0.0) {
      scenario_ = "3_NO_USABLE_POSE";
      startGlobal("cannot verify saved pose (too few valid beams or TF missing)");
      return;
    }
    if (score < match_thr_) {
      scenario_ = "2_MOVED_WHILE_OFF";
      startGlobal("saved pose does not match current scan (score " + fmt(score) + ")");
      return;
    }
    scenario_ = "1_NORMAL_RESTART";
    saved_ = p;
    publishInitialPose(p);
    baseline_ = amcl_count_;
    nomotion_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>(),
                                         [](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {});
    setState(State::CONFIRM, "initial pose published; confirming with AMCL");
  }

  void tickConfirm() {
    if (amcl_count_ <= baseline_) {
      if (elapsed() > confirm_timeout_) {
        scenario_ = "2_MOVED_WHILE_OFF";
        startGlobal("AMCL did not respond to the restored pose");
      }
      return;
    }
    const Pose2D a = amclPose();
    const double sigma = sigmaXY();
    const double drift = std::hypot(a.x - saved_.x, a.y - saved_.y);
    const double dyaw = std::fabs(normalizeAngle(a.yaw - saved_.yaw));
    const double score = scoreAtPose(a);
    RCLCPP_INFO(get_logger(), "Confirm: sigma=%.3f drift=%.3f dyaw=%.3f scan=%.2f", sigma, drift, dyaw, score);
    if (sigma <= max_sigma_ && drift <= max_drift_ && dyaw <= max_drift_yaw_ && score >= match_thr_) {
      setState(State::LOCALIZED, "restored saved pose (sigma=" + fmt(sigma) + ")");
      return;
    }
    if (elapsed() > confirm_timeout_) {
      scenario_ = "2_MOVED_WHILE_OFF";
      startGlobal("AMCL estimate disagrees with saved pose");
    }
  }

  void startGlobal(const std::string& reason) {
    RCLCPP_WARN(get_logger(), "Global localization: %s", reason.c_str());
    baseline_ = amcl_count_;
    hold_active_ = false;
    global_loc_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>(),
                                           [](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {});
    setState(State::GLOBAL_LOCALIZE, reason);
  }

  void tickGlobal() {
    publishCmd(spin_speed_);
    if (elapsed() > global_timeout_) {
      fail("global localization did not converge within " + fmt(global_timeout_) + " s");
      return;
    }
    if (amcl_count_ <= baseline_) return;
    const double sigma = sigmaXY();
    const double score = scoreAtPose(amclPose());
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Global: sigma=%.3f scan=%.2f", sigma, score);
    if (sigma <= max_sigma_ && score >= match_thr_) {
      if (!hold_active_) { hold_active_ = true; hold_start_ = SteadyClock::now(); }
      else if (std::chrono::duration<double>(SteadyClock::now() - hold_start_).count() >= hold_sec_) {
        stopRobot();
        setState(State::LOCALIZED, "global localization converged (sigma=" + fmt(sigma) + ")");
      }
    } else {
      hold_active_ = false;
    }
  }

  void fail(const std::string& reason) {
    stopRobot();
    RCLCPP_ERROR(get_logger(), "LOCALIZATION FAILED: %s", reason.c_str());
    setState(State::FAILED, reason);
  }

  // ---------------- helpers ----------------
  Pose2D amclPose() const {
    const auto& p = amcl_.pose.pose;
    return {p.position.x, p.position.y,
            yawFromQuat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)};
  }
  double sigmaXY() const {
    return std::sqrt(std::max(0.0, amcl_.pose.covariance[0]) + std::max(0.0, amcl_.pose.covariance[7]));
  }
  static std::string fmt(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.2f", v);
    return b;
  }
  double elapsed() const {
    return std::chrono::duration<double>(SteadyClock::now() - state_since_).count();
  }

  void setState(State s, const std::string& detail) {
    state_ = s;
    state_since_ = SteadyClock::now();
    detail_ = detail;
    publishStatus();
    RCLCPP_INFO(get_logger(), "STATE -> %s | scenario=%s | %s", toStr(s), scenario_.c_str(), detail.c_str());
  }

  void publishStatus() {
    std_msgs::msg::String m;
    m.data = std::string("state=") + toStr(state_) + "; scenario=" + scenario_ + "; detail=" + detail_ +
             "; db=" + (cloud_ok_ ? "ok" : "unreachable_or_untested");
    status_pub_->publish(m);
  }

  double scoreAtPose(const Pose2D& p) {
    if (!map_ || !scan_) return -1.0;
    double sx, sy, syaw;
    try {
      auto t = tf_buffer_->lookupTransform(base_frame_, scan_->header.frame_id, tf2::TimePointZero);
      sx = t.transform.translation.x;
      sy = t.transform.translation.y;
      syaw = yawFromQuat(t.transform.rotation.x, t.transform.rotation.y,
                         t.transform.rotation.z, t.transform.rotation.w);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", e.what());
      return -1.0;
    }
    return scanMatchScore(*map_, *scan_, p.x, p.y, p.yaw, sx, sy, syaw, stride_, nbr_, min_beams_);
  }

  void publishInitialPose(const Pose2D& p) {
    PoseStamped m;
    m.header.stamp = now();  // sim time (use_sim_time is set in the launch file)
    m.header.frame_id = "map";
    m.pose.pose.position.x = p.x;
    m.pose.pose.position.y = p.y;
    m.pose.pose.orientation.z = std::sin(p.yaw / 2.0);
    m.pose.pose.orientation.w = std::cos(p.yaw / 2.0);
    m.pose.covariance[0] = init_cov_xy_;
    m.pose.covariance[7] = init_cov_xy_;
    m.pose.covariance[35] = init_cov_yaw_;
    initial_pose_pub_->publish(m);
  }

  void publishCmd(double wz) {
    if (use_stamped_) {
      geometry_msgs::msg::TwistStamped m;
      m.header.stamp = now();
      m.header.frame_id = base_frame_;
      m.twist.angular.z = wz;
      cmd_stamped_pub_->publish(m);
    } else {
      geometry_msgs::msg::Twist m;
      m.angular.z = wz;
      cmd_pub_->publish(m);
    }
  }
  void stopRobot() { for (int i = 0; i < 3; ++i) publishCmd(0.0); }

  // ---------------- persistence ----------------
  LoadResult loadBest() {
    LoadResult out;
    PoseRecord cloud, cache;
    ReadStatus st = ReadStatus::UNREACHABLE;
    for (int attempt = 0; attempt < 3; ++attempt) {  // runs off the executor thread
      st = db_->readPose(robot_id_, cloud);
      if (st != ReadStatus::UNREACHABLE) break;
      std::this_thread::sleep_for(1s);
    }
    const bool have_cloud = (st == ReadStatus::OK);
    cloud_ok_ = (st == ReadStatus::OK || st == ReadStatus::NOT_FOUND);
    if (st == ReadStatus::UNREACHABLE) out.reason = "db_unreachable";
    else if (st == ReadStatus::ERROR) out.reason = "db_error";
    const bool have_cache = use_cache_ && readCache(cache);

    if (!have_cloud && !have_cache) return out;
    out.found = true;
    if (have_cloud && (!have_cache || cloud.stamp_unix >= cache.stamp_unix)) {
      out.rec = cloud; out.source = "cloud";
    } else {
      out.rec = cache; out.source = "local cache";
    }
    return out;
  }

  bool readCache(PoseRecord& rec) const {
    std::ifstream f(cache_path_);
    if (!f.is_open()) return false;
    auto j = nlohmann::json::parse(f, nullptr, false);
    return !j.is_discarded() && CloudDBClient::fromJson(j, rec);
  }

  void writeCache(const PoseRecord& rec) const {
    if (!use_cache_) return;
    const fs::path tmp = cache_path_.string() + ".tmp";
    {
      std::ofstream o(tmp);
      o << CloudDBClient::toJson(rec).dump(2);
    }
    std::error_code ec;
    fs::rename(tmp, cache_path_, ec);  // atomic replace
  }

  void saveTick() {
    if (state_ == State::LOCALIZED) maybeSave(false);
  }

  void maybeSave(bool force) {
    if (amcl_count_ == 0 || sigmaXY() > max_sigma_) return;  // never persist a lost pose
    const Pose2D p = amclPose();
    const double since = std::chrono::duration<double>(SteadyClock::now() - last_save_).count();
    const bool moved = !have_saved_ || std::hypot(p.x - last_x_, p.y - last_y_) > save_min_delta_ ||
                       std::fabs(normalizeAngle(p.yaw - last_yaw_)) > 0.05;
    if (!force && !moved && since < save_heartbeat_) return;

    PoseRecord rec;
    rec.robot_id = robot_id_;
    rec.map_id = map_id_;
    const auto& q = amcl_.pose.pose;
    rec.x = q.position.x; rec.y = q.position.y; rec.z = q.position.z;
    rec.qx = q.orientation.x; rec.qy = q.orientation.y; rec.qz = q.orientation.z; rec.qw = q.orientation.w;
    rec.cov_xx = amcl_.pose.covariance[0];
    rec.cov_yy = amcl_.pose.covariance[7];
    rec.cov_yaw = amcl_.pose.covariance[35];
    rec.stamp_unix = unixNow();
    if (!isValidPose(rec)) return;

    writeCache(rec);
    {
      std::lock_guard<std::mutex> l(save_mx_);
      pending_ = rec;
      has_pending_ = true;
    }
    save_cv_.notify_one();
    have_saved_ = true; last_x_ = p.x; last_y_ = p.y; last_yaw_ = p.yaw;
    last_save_ = SteadyClock::now();
  }

  void saveLoop() {  // worker thread: slow network I/O never blocks the executor
    for (;;) {
      PoseRecord rec;
      {
        std::unique_lock<std::mutex> l(save_mx_);
        save_cv_.wait(l, [this] { return has_pending_ || stop_; });
        if (!has_pending_) return;  // stop requested and nothing left to flush
        rec = pending_;
        has_pending_ = false;
      }
      const bool ok = db_->upsertPose(rec);
      if (ok != cloud_ok_.exchange(ok))
        RCLCPP_WARN(get_logger(), ok ? "Cloud DB reachable again" : "Cloud DB unreachable; using local cache only");
    }
  }

  // ---------------- members ----------------
  std::string db_url_, auth_token_, robot_id_, map_id_, base_frame_;
  bool use_cache_{true}, use_stamped_{true};
  double save_min_delta_, save_heartbeat_, max_age_, match_thr_, max_sigma_, max_drift_, max_drift_yaw_;
  double init_cov_xy_, init_cov_yaw_, startup_timeout_, confirm_timeout_, global_timeout_, hold_sec_, spin_speed_;
  int nbr_, min_beams_, stride_;
  fs::path cache_path_;

  State state_{State::WAIT_READY};
  std::string scenario_{"PENDING"}, detail_;
  SteadyClock::time_point state_since_{SteadyClock::now()}, hold_start_, last_save_{SteadyClock::now()};
  bool hold_active_{false}, have_saved_{false};
  double last_x_{0}, last_y_{0}, last_yaw_{0};
  Pose2D saved_{0, 0, 0};

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  PoseStamped amcl_;
  uint64_t amcl_count_{0}, baseline_{0};

  std::unique_ptr<CloudDBClient> db_;
  std::future<LoadResult> load_future_;
  std::thread save_thread_;
  std::mutex save_mx_;
  std::condition_variable save_cv_;
  PoseRecord pending_;
  bool has_pending_{false}, stop_{false};
  std::atomic<bool> cloud_ok_{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr amcl_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_loc_client_, nomotion_client_;
  rclcpp::TimerBase::SharedPtr tick_timer_, save_timer_;
};

int main(int argc, char** argv) {
  curl_global_init(CURL_GLOBAL_DEFAULT);
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<PoseSaverNode>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  curl_global_cleanup();
  return 0;
}