#pragma once
#include "CameraStreamer.g.h"

#include <winrt/HoloLensForCV.h>

namespace winrt::HoloLensRos2::implementation
{
    class CameraStreamer : public CameraStreamerT<CameraStreamer>
    {
	public:
		CameraStreamer() = default;

        Windows::Foundation::IAsyncAction StartPublishing();

	private:
		std::shared_ptr<HoloLensForCV::SensorFrameStreamer> sensor_frame_streamer_;
		std::shared_ptr<HoloLensForCV::MediaFrameSourceGroup> media_frame_source_group_;
		HoloLensForCV::SpatialPerception spatial_perception_;
    };
}
namespace winrt::HoloLensRos2::factory_implementation
{
    struct CameraStreamer : CameraStreamerT<CameraStreamer, implementation::CameraStreamer>
    {
    };
}