#pragma once
#include "CameraStreamer.g.h"

#include <winrt/HoloLensForCV.h>

namespace winrt::HoloLensRos2::implementation
{
	struct CameraStreamer : public CameraStreamerT<CameraStreamer>
    {
		CameraStreamer() = default;

        Windows::Foundation::IAsyncAction StartPublishing();
    };
}
namespace winrt::HoloLensRos2::factory_implementation
{
    struct CameraStreamer : CameraStreamerT<CameraStreamer, implementation::CameraStreamer>
    {
    };
}