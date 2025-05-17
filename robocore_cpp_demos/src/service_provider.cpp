#include <rclcpp/rclcpp.hpp>
#include "robocore_interfaces/srv/add_two_ints.hpp"

#include <memory>


using namespace std::placeholders;

class SimpleServiceServer : public rclcpp::Node
{
public:
    SimpleServiceServer() : Node("service_provider")
{
        service_ = create_service<robocore_interfaces::srv::AddTwoInts>("add_two_ints", std::bind(&SimpleServiceServer::serviceCallback, this, _1, _2));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service add_two_ints Ready");
    }


private:
    rclcpp::Service<robocore_interfaces::srv::AddTwoInts>::SharedPtr service_;

    void serviceCallback(const std::shared_ptr<robocore_interfaces::srv::AddTwoInts::Request> req,
                         const std::shared_ptr<robocore_interfaces::srv::AddTwoInts::Response> res)
{
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "New Request Received a: " << req->a << " b: " << req->b);
        res->sum = req->a + req->b;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Returning sum: " << res->sum);
    }

};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleServiceServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}