#include "pch.h"
#include "CameraStreamer.h"
#include "CameraStreamer.g.cpp"

#include "winrt/HoloLensForCV.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>


namespace winrt::HoloLensRos2::implementation
{
	class RosNode : public rclcpp::Node
	{
	public:
		RosNode(std::string const& name, std::shared_ptr<HoloLensForCV::MediaFrameSourceGroup> media_frame_source_group)
			: Node(name), frame_count_(0), media_frame_source_group_(media_frame_source_group)
		{

		}

		Windows::Foundation::IAsyncAction RunAsync()
		{
			co_await winrt::resume_background();

			rclcpp::init(0, nullptr);

			frame_cnt_publisher_ = this->create_publisher<std_msgs::msg::String>("frame_counter", 10);
			image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("vlc_leftleft", rmw_qos_profile_sensor_data);

	
			while (rclcpp::ok())
			{
				auto frame = media_frame_source_group_->GetLatestSensorFrame(HoloLensForCV::SensorType::VisibleLightLeftLeft);
				Windows::Graphics::Imaging::SoftwareBitmap bmp = frame.SoftwareBitmap();
				Windows::Storage::Streams::Buffer frame_buffer(bmp.PixelHeight() * bmp.PixelWidth() * 4);
				bmp.CopyToBuffer(frame_buffer);
				auto size = frame_buffer.Length();
				winrt::Windows::Storage::Streams::DataReader reader =
					winrt::Windows::Storage::Streams::DataReader::FromBuffer(
						frame_buffer);
				std::vector<std::uint8_t> bytes(bmp.PixelHeight() * bmp.PixelWidth() * 4);
				reader.ReadBytes(bytes);

				// Populate the ROS messages
				msg_->header.frame_id = frame_count_++;
				msg_->height = bmp.PixelHeight();
				msg_->width = bmp.PixelWidth() * 4;
				msg_->encoding = sensor_msgs::image_encodings::MONO8;
				msg_->step = bmp.PixelWidth() * 4; // Assume 8 bit pixels!
				msg_->is_bigendian = false;
				msg_->data = bytes;
				image_publisher_->publish(msg_);

				std_msgs::msg::String msg;
				msg.data = "Frame count: " + std::to_string(frame_count_);
				frame_cnt_publisher_->publish(msg);
			}

			rclcpp::shutdown();
			co_return;
		}

	private:
		std::shared_ptr<HoloLensForCV::MediaFrameSourceGroup> media_frame_source_group_;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr frame_cnt_publisher_;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
		sensor_msgs::msg::Image::SharedPtr msg_;
		size_t frame_count_;
	};
	

    Windows::Foundation::IAsyncAction CameraStreamer::StartPublishing()
    {
		std::vector<HoloLensForCV::SensorType> enabled_sensor_types;

		enabled_sensor_types.emplace_back(
			HoloLensForCV::SensorType::VisibleLightLeftLeft);

		sensor_frame_streamer_ = std::make_shared<HoloLensForCV::SensorFrameStreamer>();

		for (auto const& type : enabled_sensor_types)
		{
			sensor_frame_streamer_->Enable(type);
		}

		media_frame_source_group_ = std::make_shared<HoloLensForCV::MediaFrameSourceGroup>(
			HoloLensForCV::MediaFrameSourceGroupType::HoloLensResearchModeSensors,
			spatial_perception_,
			*sensor_frame_streamer_);

		for (auto const& type : enabled_sensor_types)
		{
			media_frame_source_group_->Enable(type);
		}

		co_await media_frame_source_group_->StartAsync();

		auto n = std::make_shared<RosNode>("HoloLensResearch", this->media_frame_source_group_);

		co_await n->RunAsync();
    }
}
