#include <chrono>

#include "pch.h"
#include "Node.h"
#include "Node.g.cpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using namespace winrt::Windows::Foundation;

namespace winrt::HoloLensRos2::implementation
{
	class RosNode : public rclcpp::Node
	{
	public:
		RosNode(std::string const& name)
			: Node(name), count_(0)
		{
			publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
			auto timer_callback =
				[this]() -> void
				{
					auto message = std_msgs::msg::String();
					message.data = "Hello, world: " + std::to_string(this->count_++);
					this->publisher_->publish(message);
				};

			timer_ = this->create_wall_timer(500ms, timer_callback);
		}

	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
		size_t count_;
	};

	Node::Node(hstring const& name) : node_name_(name)
	{ }

	IAsyncAction Node::StartPublishing()
	{
		co_await winrt::resume_background();

		rclcpp::init(0, nullptr);
		rclcpp::spin(std::make_shared<RosNode>(winrt::to_string(node_name_)));
		rclcpp::shutdown();
	}
}
