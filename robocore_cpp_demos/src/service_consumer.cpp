#include <rclcpp/rclcpp.hpp>
#include "robocore_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <memory>


using namespace std::chrono_literals;
using std::placeholders::_1;


class SimpleServiceClient : public rclcpp::Node 
{
public:
    SimpleServiceClient(int a, int b) : Node("service_consumer") {
        client_ = create_client<robocore_interfaces::srv::AddTwoInts>("add_two_ints");

        auto request = std::make_shared<robocore_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = client_->async_send_request(request, std::bind(&SimpleServiceClient::responseCallback, this, _1));
    }

private:
    rclcpp::Client<robocore_interfaces::srv::AddTwoInts>::SharedPtr client_;

    void responseCallback(rclcpp::Client<robocore_interfaces::srv::AddTwoInts>::SharedFuture future) {
        if(future.valid()) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Service Response " << future.get()->sum);
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service Failure");
        }
    }
};


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  if (argc != 3) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Wrong number of arguments! Usage: service_consumer A B");
    return 1;
  }

  auto node = std::make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2]));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}