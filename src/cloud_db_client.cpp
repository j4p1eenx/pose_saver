#include "pose_saver_pkg/cloud_db_client.hpp"
#include <curl/curl.h>
#include <utility>
#include <cstdio>

using json = nlohmann::json;

namespace {
size_t writeCb(void* c, size_t s, size_t n, void* u) {
  static_cast<std::string*>(u)->append(static_cast<char*>(c), s * n);
  return s * n;
}
}  // namespace

CloudDBClient::CloudDBClient(std::string db_url, std::string auth_token)
    : db_url_(std::move(db_url)), auth_token_(std::move(auth_token)) {
  while (!db_url_.empty() && db_url_.back() == '/') db_url_.pop_back();
}

HttpResult CloudDBClient::request(const std::string& method, const std::string& endpoint,
                                  const std::string& payload) const {
  HttpResult r;
  CURL* curl = curl_easy_init();
  if (!curl) return r;
  std::string url = db_url_ + endpoint + ".json";
  if (!auth_token_.empty()) url += "?auth=" + auth_token_;

  curl_slist* headers = curl_slist_append(nullptr, "Content-Type: application/json");
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, method.c_str());
  if (!payload.empty()) curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &r.body);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 6000L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 10000L);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

  const CURLcode rc = curl_easy_perform(curl);
  if (rc == CURLE_OK)
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &r.status);
  else
    std::fprintf(stderr, "[CloudDB] %s %s failed: %s\n", method.c_str(), endpoint.c_str(),
                 curl_easy_strerror(rc));

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  return r;
}

json CloudDBClient::toJson(const PoseRecord& r) {
  return {{"robot_id", r.robot_id}, {"map_id", r.map_id},
          {"x", r.x}, {"y", r.y}, {"z", r.z},
          {"qx", r.qx}, {"qy", r.qy}, {"qz", r.qz}, {"qw", r.qw},
          {"cov_xx", r.cov_xx}, {"cov_yy", r.cov_yy}, {"cov_yaw", r.cov_yaw},
          {"stamp_unix", r.stamp_unix}};
}

bool CloudDBClient::fromJson(const json& j, PoseRecord& r) {
  try {
    r.robot_id = j.value("robot_id", std::string());
    r.map_id = j.value("map_id", std::string());
    r.x = j.at("x").get<double>();   // required: a missing key must not become (0,0)
    r.y = j.at("y").get<double>();
    r.qw = j.at("qw").get<double>();
    r.z = j.value("z", 0.0);
    r.qx = j.value("qx", 0.0);
    r.qy = j.value("qy", 0.0);
    r.qz = j.value("qz", 0.0);
    r.cov_xx = j.value("cov_xx", 0.0);
    r.cov_yy = j.value("cov_yy", 0.0);
    r.cov_yaw = j.value("cov_yaw", 0.0);
    r.stamp_unix = j.value("stamp_unix", 0.0);
    return true;
  } catch (...) {
    return false;
  }
}

bool CloudDBClient::upsertPose(const PoseRecord& rec) {
  return request("PUT", "/poses/" + rec.robot_id, toJson(rec).dump()).ok();
}

bool CloudDBClient::createPose(const PoseRecord& rec) {
  auto r = request("GET", "/poses/" + rec.robot_id);
  if (!r.ok() || r.body != "null") return false;  // unreachable, or already exists
  return upsertPose(rec);
}

bool CloudDBClient::updatePose(const std::string& id, const PoseRecord& rec) {
  auto r = request("GET", "/poses/" + id);
  if (!r.ok() || r.body == "null") return false;
  PoseRecord copy = rec;
  copy.robot_id = id;
  return upsertPose(copy);
}

ReadStatus CloudDBClient::readPose(const std::string& id, PoseRecord& rec) {
  auto r = request("GET", "/poses/" + id);
  if (r.status == 0) return ReadStatus::UNREACHABLE;
  if (!r.ok()) return ReadStatus::ERROR;
  if (r.body.empty() || r.body == "null") return ReadStatus::NOT_FOUND;
  auto j = json::parse(r.body, nullptr, false);
  if (j.is_discarded() || !fromJson(j, rec)) return ReadStatus::ERROR;
  if (rec.robot_id.empty()) rec.robot_id = id;
  return ReadStatus::OK;
}

std::vector<PoseRecord> CloudDBClient::readAllPoses() {
  std::vector<PoseRecord> out;
  auto r = request("GET", "/poses");
  if (!r.ok() || r.body.empty() || r.body == "null") return out;
  auto j = json::parse(r.body, nullptr, false);
  if (j.is_discarded() || !j.is_object()) return out;
  for (auto it = j.begin(); it != j.end(); ++it) {
    PoseRecord rec;
    if (fromJson(it.value(), rec)) {
      if (rec.robot_id.empty()) rec.robot_id = it.key();
      out.push_back(rec);
    }
  }
  return out;
}

bool CloudDBClient::deletePose(const std::string& id) {
  return request("DELETE", "/poses/" + id).ok();
}
