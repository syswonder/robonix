#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "robonix_sdk/srv/ping_pong.hpp"

using namespace std::chrono_literals;

class StressTestClient : public rclcpp::Node {
public:
  StressTestClient(int client_id)
      : Node("stress_test_cpp_" + std::to_string(client_id)) {
    client_ = this->create_client<robonix_sdk::srv::PingPong>("/rbnx/ping");
  }

  bool wait_for_service(std::chrono::seconds timeout) {
    return client_->wait_for_service(timeout);
  }

  std::shared_ptr<robonix_sdk::srv::PingPong::Response> call_service(int task_id, double& latency_us) {
    auto request = std::make_shared<robonix_sdk::srv::PingPong::Request>();
    request->message = "ping";
    request->sequence = task_id;

    auto start = std::chrono::high_resolution_clock::now();
    auto result_future = client_->async_send_request(request);

    // Wait for the result with a timeout
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, 5s) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto end = std::chrono::high_resolution_clock::now();
      latency_us = std::chrono::duration<double, std::micro>(end - start).count();
      return result_future.get();
    } else {
      return nullptr;
    }
  }

private:
  rclcpp::Client<robonix_sdk::srv::PingPong>::SharedPtr client_;
};

int main(int argc, char** argv) {
  int client_id = 0;
  int requests = 1000;
  int rate = 100;
  int duration = 0;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--client-id" && i + 1 < argc) client_id = std::stoi(argv[++i]);
    else if (arg == "--requests" && i + 1 < argc) requests = std::stoi(argv[++i]);
    else if (arg == "--rate" && i + 1 < argc) rate = std::stoi(argv[++i]);
    else if (arg == "--duration" && i + 1 < argc) duration = std::stoi(argv[++i]);
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<StressTestClient>(client_id);

  if (!node->wait_for_service(10s)) {
    std::cerr << "Service not available" << std::endl;
    return 1;
  }

  std::vector<double> latencies;
  int success = 0;
  int failed = 0;
  
  auto start_time = std::chrono::high_resolution_clock::now();
  auto end_time = duration > 0 ? 
    start_time + std::chrono::seconds(duration) : 
    std::chrono::high_resolution_clock::time_point::max();
    
  double interval_s = rate > 0 ? 1.0 / rate : 0;
  auto last_req = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < requests; ++i) {
    if (std::chrono::high_resolution_clock::now() >= end_time) break;
    
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration<double>(now - last_req).count();
    if (elapsed < interval_s) {
      std::this_thread::sleep_for(std::chrono::duration<double>(interval_s - elapsed));
    }
    last_req = std::chrono::high_resolution_clock::now();

    double lat_us = 0;
    auto response = node->call_service(i, lat_us);
    if (response) {
      success++;
      latencies.push_back(lat_us);
    } else {
      failed++;
    }
  }

  std::sort(latencies.begin(), latencies.end());
  int total = success + failed;
  auto p = [&](double pct) {
    if (latencies.empty()) return 0.0;
    return latencies[static_cast<int>((latencies.size() - 1) * pct / 100.0)];
  };

  std::cout << "\nOverall Statistics:" << std::endl;
  std::cout << "Total: " << total << ", Success: " << success << ", Failed: " << failed << std::endl;
  
  double avg = latencies.empty() ? 0.0 : std::accumulate(latencies.begin(), latencies.end(), 0.0) / latencies.size();
  
  printf("Latency (us): Avg: %.2f, Min: %.2f, Max: %.2f, P50: %.2f, P95: %.2f, P99: %.2f, P999: %.2f\n",
         avg, p(0), p(100), p(50), p(95), p(99), p(99.9));

  rclcpp::shutdown();
  return 0;
}
