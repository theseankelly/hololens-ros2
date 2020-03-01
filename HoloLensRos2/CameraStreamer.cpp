#include "pch.h"
#include "CameraStreamer.h"
#include "CameraStreamer.g.cpp"

#include "winrt/HoloLensForCV.h"

#include "HoloCameraStreamer.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;

namespace winrt::HoloLensRos2::implementation
{
	class CameraNode : public rclcpp::Node
	{
	public:
		CameraNode(std::string const& name) 
			: rclcpp::Node(name), count_(0), streamer_(std::make_shared<HoloCameraStreamer>())
		{
			publisher_ = this->create_publisher<sensor_msgs::msg::Image>("vlcll", rclcpp::SensorDataQoS());
			auto frame_callback =
				[this](const MediaFrameReader& sender, const MediaFrameArrivedEventArgs& args) -> void
			{
				// Get the frame!
				auto frame = sender.TryAcquireLatestFrame();
				if (frame)
				{
					//frame.BufferMediaFrame().Buffer().data();

					// Read the data from the IBuffer with a DataReader
					// There's GOT to be a more efficient way to do this, but
					// this is the safest as I don't think MF buffers are
					// guaranteed to be contiguous


					winrt::Windows::Graphics::Imaging::SoftwareBitmap bmp = frame.VideoMediaFrame().SoftwareBitmap();

					winrt::Windows::Storage::Streams::Buffer frame_buffer(bmp.PixelHeight() * bmp.PixelWidth() * 4);

					//std::vector<std::uint8_t> pixel_data;
					//pixel_data.reserve(bmp.PixelHeight() * bmp.PixelWidth() * 4);

					//winrt::array_view<std::uint8_t> pixel_array{ pixel_data };

					//pixel_array.

					bmp.CopyToBuffer(frame_buffer);
					//frame.VideoMediaFrame().SoftwareBitmap().CopyToBuffer(frame_buffer);


					auto size = frame_buffer.Length();
					winrt::Windows::Storage::Streams::DataReader reader =
						winrt::Windows::Storage::Streams::DataReader::FromBuffer(
							frame_buffer);
					std::vector<std::uint8_t> bytes(bmp.PixelHeight() * bmp.PixelWidth() * 4);
					reader.ReadBytes(bytes);


					// Populate the ROS message
					auto message = sensor_msgs::msg::Image();
					message.header.frame_id = count_++;
					message.height = frame.Format().VideoFormat().Height();
					message.width = frame.Format().VideoFormat().Width() * 4;
					message.encoding = sensor_msgs::image_encodings::MONO8;
					message.step = frame.Format().VideoFormat().Width() * 4; // Assume 8 bit pixels!
					message.is_bigendian = false;
					message.data = bytes;
					publisher_->publish(message);
				}
			};

			streamer_->StartStreamingAsync(frame_callback);
		}

	private:
		std::shared_ptr<HoloCameraStreamer> streamer_;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
		size_t count_;
	};
    
	Windows::Foundation::IAsyncAction CameraStreamer::StartPublishing()
    {
		co_await winrt::resume_background();
		
		rclcpp::init(0, nullptr);
		rclcpp::spin(std::make_shared<CameraNode>("HoloLensCameras"));
		rclcpp::shutdown();
    }
}
