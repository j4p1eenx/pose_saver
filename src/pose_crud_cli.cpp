#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <curl/curl.h>
#include "pose_saver_pkg/cloud_db_client.hpp"
#include "pose_saver_pkg/pose_logic.hpp"

static int usage() {
  std::cerr <<
    "Usage: pose_crud_cli <db_url> <command> [args]   (auth: env POSE_DB_TOKEN)\n"
    "  create <robot_id> <x> <y> <yaw_rad> [map_id]   fails if the record exists\n"
    "  update <robot_id> <x> <y> <yaw_rad> [map_id]   fails if the record is missing\n"
    "  read   <robot_id>\n"
    "  readall\n"
    "  delete <robot_id>\n";
  return 1;
}

int main(int argc, char** argv) {
  if (argc < 3) return usage();
  curl_global_init(CURL_GLOBAL_DEFAULT);
  struct Guard { ~Guard() { curl_global_cleanup(); } } guard;

  const std::string cmd = argv[2];
  const char* tok = std::getenv("POSE_DB_TOKEN");
  CloudDBClient client(argv[1], tok ? tok : "");

  try {
    if (cmd == "create" || cmd == "update") {
      if (argc < 7) return usage();
      PoseRecord r;
      r.robot_id = argv[3];
      r.x = std::stod(argv[4]);
      r.y = std::stod(argv[5]);
      const double yaw = std::stod(argv[6]);
      r.qz = std::sin(yaw / 2.0);
      r.qw = std::cos(yaw / 2.0);
      r.map_id = argc > 7 ? argv[7] : "turtlebot3_world";
      r.stamp_unix = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      const bool ok = cmd == "create" ? client.createPose(r) : client.updatePose(r.robot_id, r);
      std::cout << (ok ? "OK" : "FAILED (unreachable, auth error, exists, or missing)") << "\n";
      return ok ? 0 : 2;
    }
    if (cmd == "read") {
      if (argc < 4) return usage();
      PoseRecord r;
      switch (client.readPose(argv[3], r)) {
        case ReadStatus::OK:
          std::cout << r.robot_id << " map=" << r.map_id << " x=" << r.x << " y=" << r.y
                    << " yaw=" << yawFromQuat(r.qx, r.qy, r.qz, r.qw)
                    << " stamp=" << static_cast<long>(r.stamp_unix) << "\n";
          return 0;
        case ReadStatus::NOT_FOUND: std::cout << "Not found\n"; return 3;
        case ReadStatus::UNREACHABLE: std::cout << "DB unreachable\n"; return 4;
        default: std::cout << "DB error (auth/rules?)\n"; return 5;
      }
    }
    if (cmd == "readall") {
      for (const auto& r : client.readAllPoses())
        std::cout << r.robot_id << " map=" << r.map_id << " (" << r.x << ", " << r.y << ")\n";
      return 0;
    }
    if (cmd == "delete") {
      if (argc < 4) return usage();
      const bool ok = client.deletePose(argv[3]);
      std::cout << (ok ? "DELETED" : "FAILED") << "\n";
      return ok ? 0 : 2;
    }
  } catch (const std::exception& e) {
    std::cerr << "Bad arguments: " << e.what() << "\n";
    return 1;
  }
  return usage();
}
