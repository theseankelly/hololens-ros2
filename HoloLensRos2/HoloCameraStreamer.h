#pragma once

class HoloCameraStreamer
{
public:
	HoloCameraStreamer();
	~HoloCameraStreamer();

	winrt::Windows::Foundation::IAsyncAction StartStreamingAsync(
		winrt::Windows::Foundation::TypedEventHandler<winrt::Windows::Media::Capture::Frames::MediaFrameReader, winrt::Windows::Media::Capture::Frames::MediaFrameArrivedEventArgs> handler);

private:
	winrt::Windows::Media::Capture::Frames::MediaFrameSourceGroup _source_group = nullptr;
	winrt::Windows::Media::Capture::Frames::MediaFrameSource _source = nullptr;
};



