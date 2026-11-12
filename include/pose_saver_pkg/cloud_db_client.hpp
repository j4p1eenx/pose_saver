#pragma once
#include <cmath>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

struct PoseRecord {
  std::string robot_id;
  std::string map_id;
  double x{0}, y{0}, z{0};
  double qx{0}, qy{0}, qz{0}, qw{1};
  double cov_xx{0}, cov_yy{0}, cov_yaw{0};
  double stamp_unix{0};  // seconds since epoch, UTC
};

struct HttpResult {
  long status{0};  // 0 = network failure
  std::string body;
  bool ok() const { return status >= 200 && status < 300; }
};

enum class ReadStatus { OK, NOT_FOUND, UNREACHABLE, ERROR };

class CloudDBClient {
public:
  explicit CloudDBClient(std::string db_url, std::string auth_token = "");

  // CRUD. Records live at /poses/<robot_id>.json (Firebase Realtime DB REST).
  bool createPose(const PoseRecord& rec);                       // fails if it already exists
  ReadStatus readPose(const std::string& robot_id, PoseRecord& rec);
  std::vector<PoseRecord> readAllPoses();
  bool updatePose(const std::string& robot_id, const PoseRecord& rec);  // fails if missing
  bool deletePose(const std::string& robot_id);                 // idempotent
  bool upsertPose(const PoseRecord& rec);                       // used by the node

  static nlohmann::json toJson(const PoseRecord& rec);
  static bool fromJson(const nlohmann::json& j, PoseRecord& rec);

private:
  HttpResult request(const std::string& method, const std::string& endpoint,
                     const std::string& payload = "") const;
  std::string db_url_, auth_token_;
};
